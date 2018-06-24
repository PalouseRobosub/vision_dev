#!/usr/bin/python
import json
import os
import sys
import argparse
import tarfile
import glob

# This script is used to extract labels and images from a dataset.

# Main function
def app(args):
    # Open json in read only way and load its context
    with open(args.name, 'r') as f:
        json_contents = json.load(f)
        
    # Get tar name without '.json' part
    tar_name = os.path.splitext(os.path.basename(args.name))[0]

    annotations = []
    
    # Loop over json file.
    for annotation in json_contents:
        if annotation['annotations'] != []:
            found = []
            
            # Loop over individual annotations. If annotation is same as in args.labels list() - extract it.
            for p in annotation['annotations']:
                if p['class'] in args.labels:
                    if p['class'] != 'start_gate_post' and p in found:
                        break
                    found.append(p)
            
            # If we found what we were looking for - format our data into json format. If not, ignore it.
            if found != []:
                try:
                    os.rename(annotation['filename'], tar_name + "/" + annotation['filename'])
                    annotations.append({'annotations': found,
                                        'class': 'image',
                                        'filename': annotation['filename']})
                except:
                    pass
         
    # Remove old json file
    os.remove(args.name)
    
    # Puts all annotations into json file
    with open(tar_name, 'w') as f:
        json.dump(annotations, f, indent=4)

    # Optional flag that will put things into tar
    if args.tar:
        with tarfile.open(tar_name, "w") as tar:
            tar.add(tar_name)
        os.remove(tar_name)

if __name__ == '__main__':

    # Add arguments.
    extract_parser = argparse.ArgumentParser(description='Utility for robosub labeling task to extract images')
    extract_parser.add_argument('name', type=str, help='name of the json file')
    extract_parser.add_argument('--labels', nargs='+', help='type labels names seperated by spaces to be extracted')
    extract_parser.add_argument('--tar', action='store_true', help='if you want to put them into tar')
    extract_parser.set_defaults(func=app)

    args = extract_parser.parse_args()

    # Call the appropriate handler function
    try:
        args.func(args)
    except:
        extract_parser.print_help()
