#!/usr/bin/python
import json
import os
import sys
import argparse
import tarfile
import glob

# This script is used to extract labels and images from a dataset.

def app(args):
    with open(args.name, 'r') as f:
        json_contents = json.load(f)

    tar_name = os.path.splitext(os.path.basename(args.name))[0]

    annotations = []
    for annotation in json_contents:
        if annotation['annotations'] != []:
            found = []
            for p in annotation['annotations']:
                if p['class'] in args.labels:
                    if p['class'] != 'start_gate_post' and p in found:
                        break
                    found.append(p)
            if found != []:
                try:
                    os.rename(annotation['filename'], "new/" + annotation['filename'])
                    annotations.append({'annotations': found,
                                        'class': 'image',
                                        'filename': annotation['filename']})
                except:
                    pass

    with open(args.name, 'w') as f:
        json.dump(annotations, f, indent=4)

    if args.tar:
        with tarfile.open(tar_name, "w") as tar:
            tar.add(os.path.dirname(args.name))

if __name__ == '__main__':

    extract_parser = argparse.ArgumentParser(description='Utility for robosub labeling task to extract images')
    extract_parser.add_argument('name', type=str, help='name of the json file')
    extract_parser.add_argument('--labels', nargs='+', help='type labels names seperated by spaces to be extracted')
    extract_parser.add_argument('--tar', type="store_true", help='if you want to put them into tar')
    extract_parser.set_defaults(func=app)

    args = extract_parser.parse_args()

    # Call the appropriate handler function
    try:
        args.func(args)
    except:
        extract_parser.print_help()
