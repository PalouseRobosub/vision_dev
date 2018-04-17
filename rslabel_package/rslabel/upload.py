#!/usr/bin/python
"""
@author Ryan Summers
@date 2-27-2018

@brief Unbags images from a ROS Bag and uploads them to the Robosub server.
"""

from __future__ import print_function

enabled = True

import glob
import json
import os
import progressbar
import pysftp
import shutil
import tarfile
import tempfile

try:
    import cv2
except:
    print('OpenCV installation not detected. Upload command disabled.')
    enabled = False

try:
    import rosbag
    import cv_bridge
except:
    print('ROS installation not detected. Upload command disabled')
    enabled = False

try:
    input = raw_input
except:
    pass


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
    print('Unbagging images...')
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


def app(args):
    """ Main entry point to the unbag_upload application.

    Arguments
        args: Passed in by argparse. Must have member `bag_file` that denotes
              the path to the Rosbag. Must have member `files_per_tar` that
              denotes how many images should be put into each archive.
    """
    if not enabled:
        print('The upload command is not enabled because ROS or OpenCV is not')
        print('properly installed on the host system.')
        return

    password = os.environ.get('ROBOSUB_SFTP_PASSWORD')
    if password is None:
        print('To suppress this prompt, please set the ROBOSUB_SFTP_PASSWORD ',
                end='')
        print('environment variable.')
        password = input('Please enter the Robosub SFTP password: ')

    working_directory = tempfile.mkdtemp(dir='/tmp')

    # Unbag all of the images into a temporary directory.
    ros_bag = rosbag.Bag(args.bag_file, 'r')
    temp_dir = tempfile.mkdtemp(dir=working_directory)
    unbag_images(ros_bag, temp_dir)

    print('Images unbagged to {}'.format(temp_dir))

    # Get a list of all the unbagged images.
    images = glob.glob('{}/*.jpg'.format(temp_dir))
    images.sort()

    bag_base = os.path.splitext(os.path.basename(args.bag_file))[0]

    # Move the jpgs into individual archives.
    print('Creating image archives...')
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
        with tarfile.TarFile('{}/{}.tar'.format(working_directory,
                                os.path.basename(image_dir)), mode='w') as tf:
            tf.add(image_dir, arcname=os.path.basename(image_dir))

        # Remove the image directory after we have archived it.
        shutil.rmtree(image_dir)

    bar.finish()

    # Upload the tar archives to the Robosub server using the PySFTP client.
    print('Uploading archives to server...')
    files = glob.glob('{}/*.tar'.format(working_directory))
    bar_tar = progressbar.ProgressBar(max_value=len(files))
    with pysftp.Connection('robosub.eecs.wsu.edu',
            username='sftp_user',
            password=password,
            default_path='/data/vision/labeling/clarification') as sftp:
        for i, f in enumerate(files):
            bar_tar.update(i)
            sftp.put(f)
    bar_tar.finish()

    # Remove the temporary working directory we created.
    shutil.rmtree(working_directory)
