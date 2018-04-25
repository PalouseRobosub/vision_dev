#!/usr/bin/python
# This script is designed to remove bad images given tar name and json file to be present

import argparse
import json
import os
import sys
import shutil
import glob
import tarfile

# Main function
def app(args):
    if args.remote:
        path = "/data/vision/labeling/in_progress/clarification/temp/"
    else:
        path = ""
    base_name = os.path.splitext(os.path.basename(args.tarname))[0]
    print("Working on {}".format(args.tarname))
    os.rename(path + base_name + ".json",path + base_name + "-delete.json")

    # Extract tar
    print("Extracting")
    with tarfile.TarFile(path + args.tarname) as tf:
        tf.extractall(path)
    os.remove(path + args.tarname)
    try:
        os.remove("{}/{}".format(path + base_name, base_name + ".json"))
    except:
        pass
    with open(path + base_name + '-delete.json', 'r') as f:
        json_contents = json.load(f)

    # Looping over to see if there are bad images, and see if the tar is complete
    for annotation in json_contents:
        try:
            ann = annotation['status']
            if ann == 'Bad':
                print("Removing {}".format(annotation['filename']))
                os.remove("{}/{}".format(path + base_name, annotation['filename']))
        except:
            pass
    annotations = []
    print("Creating new json")
    for f in sorted(glob.glob('{}/*.jpg'.format(path + base_name))):
        annotations.append({'annotations': [],
                            'class': 'image',
                            'filename': os.path.basename(f),
                            'unlabeled': True})
    os.remove(path + base_name + '-delete.json')
    with open("{}/{}".format(path + base_name, base_name + '.json'), 'w') as f:
        json.dump(annotations, f, indent=4)
    print("Putting everything back into tar")
    with tarfile.open(path + base_name + '.tar', "w") as tar:
        tar.add(path + base_name, arcname=base_name)
    shutil.rmtree(path + base_name)
    print("Putting tar up for labeling")
    os.rename(path + base_name + '.tar', "/data/vision/labeling/new/{}".format(base_name + '.tar'))
    print("Done!")
if __name__ == '__main__':
    # Arguement handler
    parser = argparse.ArgumentParser(description='Utility for deleting bad images.')
    parser.add_argument('tarname', type=str, help='Name of the tar with bad images')
    parser.add_argument('--remote', action='store_true', help='To run remotely by other script')
    parser.set_defaults(func=app)
    args = parser.parse_args()
    args.func(args)
