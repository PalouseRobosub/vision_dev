import argparse
import json
import os
import shutil
import logging
import sys


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Utility for working with the vision data set.')
    parser.add_argument('dataset_annotations', type=str, help='The current data set to modify.')
    parser.add_argument('--add', type=str, help='The JSON containing the new annotations to add to the dataset.', required=True)
    parser.add_argument('--ignore-duplicates', action='store_true', help='Ignore the files already exist in the data set and do not add them.')

    args = parser.parse_args()

    with open(args.dataset_annotations, 'r') as f:
        annotations = json.load(f)

    with open(args.add, 'r') as f:
        new_annotations = json.load(f)

    new_files = []
    for entry in new_annotations:
        new_files.append(os.path.basename(entry['filename']))

    files = []
    for entry in annotations:
        files.append(os.path.basename(entry['filename']))

    intersections = set(files).intersection(new_files)

    if args.ignore_duplicates is False:
        for f in intersections:
            logging.error('File {} already exists in the dataset!'.format(f))

        if len(intersections) != 0:
            sys.exit(-1)

    image_dst_path = os.path.dirname(os.path.realpath(__file__)) + '/images/'

    for entry in new_annotations:
        file_entry = entry['filename']
        if os.path.basename(file_entry) is in intersections:
            continue

        file_dst = image_dst_path + os.path.basename(file_entry)
        try:
            if entry['status'] == 'Good':
                shutil.copyfile(file_entry, file_dst)
                entry['filename'] = 'images/' + os.path.basename(file_entry)
                annotations.append(entry)
        except KeyError:
            pass

    with open(args.dataset_annotations, 'w') as f:
        json.dump(annotations, f, indent=2)
