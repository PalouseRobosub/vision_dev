#!/usr/bin/python
# This script is designed to remove bad images given tar name and json file to be present

import argparse
import datetime
import json
import os
import pysftp
import shutil
import sys
import tempfile
import glob
import tarfile


# Main function
def app(args):
    base_name = os.path.splitext(os.path.basename(args.tarname))[0]

    # Extract tar
    with tarfile.TarFile(args.tarname) as tf:
        tf.extractall()
    os.remove(args.tarname)

    with open(base_name + '-delete.json', 'r') as f:
        json_contents = json.load(f)

    # Looping over to see if there are bad images, and see if the tar is complete
    for annotation in json_contents:
        try:
            ann = annotation['status']
            if ann == 'Bad':
                os.remove("{}/{}".format(base_name, annotation['filename']))
        except:
            pass
    annotations = []
    for f in sorted(glob.glob('{}/*.jpg'.format(base_name))):
        annotations.append({'annotations': [],
                            'class': 'image',
                            'filename': os.path.basename(f),
                            'unlabeled': True})
    os.remove(base_name + '-delete.json')
    with open("{}/{}".format(base_name, base_name + '.json'), 'w') as f:
        json.dump(annotations, f, indent=4)
    with tarfile.open(base_name + '.tar', "w") as tar:
        tar.add(base_name)

    os.rename(base_name + '.tar', "../../new/{}".format(base_name + '.tar'))

if __name__ == '__main__':
    # Arguement handler
    parser = argparse.ArgumentParser(description='Utility for deleting bad images.')
    parser.add_argument('tarname', type=str, help='Name of the tar with bad images')
    parser.set_defaults(func=app)
    args = parser.parse_args()
    args.func(args)
