import rosbag
import argparse
import tempfile
import glob
import os
import pysftp
import shutil
import tarfile
import progressbar
import cv_bridge
import cv2
import json


import rospy

def unbag_images(bag, directory):
    """ Unbags images into a directory.

    Args:
        bag: The RosBag to unbag.
        directory: The path to the directory to store images into.
    """

    bridge = cv_bridge.CvBridge()
    msg_count = bag.get_message_count(topic_filters=['/camera/left/undistorted',
            '/camera/right/undistorted',
            '/camera/bottom/undistorted'])
    bar = progressbar.ProgressBar(max_value=msg_count)
    i = 0
    print 'Unbagging images...'
    for topic, msg, t in bag.read_messages(
            topics=['/camera/left/undistorted',
                    '/camera/right/undistorted',
                    '/camera/bottom/undistorted']):
        bar.update(i)
        i += 1
        cv_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
        prefix = 'left' if 'left' in topic else 'right' if 'right' in topic else 'bottom'
        timestr = '%.6f' % msg.header.stamp.to_sec()
        image_name = '{}/{}_{}.jpg'.format(directory, prefix, timestr)
        cv2.imwrite(image_name, cv_image)
    bar.finish()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Unbags images and uploads them')
    parser.add_argument('bag_file', type=str, help='The bag file to upload')
    parser.add_argument('--files-per-tar', type=int, default=100, help='The number of images per tar archive')

    args = parser.parse_args()

    password = os.environ.get('ROBOSUB_SFTP_PASSWORD')
    if password is None:
        print 'To suppress this prompt, please set the ROBOSUB_SFTP_PASSWORD environment variable.'
        password = raw_input('Please enter the Robosub SFTP password: ')

    working_directory = tempfile.mkdtemp(dir='/tmp')

    # Unbag all of the images into a temporary directory.
    ros_bag = rosbag.Bag(args.bag_file, 'r')
    temp_dir = tempfile.mkdtemp(dir=working_directory)
    unbag_images(ros_bag, temp_dir)

    print 'Images unbagged to {}'.format(temp_dir)

    # Get a list of all the unbagged images.
    images = glob.glob('{}/*.jpg'.format(temp_dir))

    bag_base = os.path.splitext(os.path.basename(args.bag_file))[0]

    # Move the jpgs into individual archives.
    print 'Creating image archives...'
    bar = progressbar.ProgressBar(max_value=len(images))

    for i in range(0, len(images), args.files_per_tar):
        bar.update(i)
        end = i +  args.files_per_tar
        if end >= len(images):
            end = len(images)

        image_dir = '{}/{}_{}'.format(working_directory, bag_base, i)
        os.mkdir(image_dir)

        annotations = []

        # Loop through all of the images in the current step. Move them into a
        # directory for archiving.
        for img in images[i : end]:
            shutil.move(img, image_dir)

        # Create an archive with the JSON annotation file and the images.
        with tarfile.TarFile('{}/{}.tar'.format(working_directory, os.path.basename(image_dir)), mode='w') as tf:
            tf.add(image_dir, arcname=os.path.basename(image_dir))

        # Remove the image directory after we have archived it.
        shutil.rmtree(image_dir)

    bar.finish()

    # Upload the tar archives to the Robosub server using the PySFTP client.
    print 'Uploading archives to server...'
    bar_tar = progressbar.ProgressBar(max_value=len(glob.glob('{}/*.tar'.format(working_directory))))
    with pysftp.Connection('robosub.eecs.wsu.edu',
            username='sftp_user',
            password=password,
            default_path='/data/vision/labeling/new') as sftp:
        for i, f in enumerate(glob.glob('{}/*.tar'.format(working_directory))):
            bar_tar.update(i)
            sftp.put(f)
    bar_tar.finish()

    # Remove the temporary working directory we created.
    shutil.rmtree(working_directory)
