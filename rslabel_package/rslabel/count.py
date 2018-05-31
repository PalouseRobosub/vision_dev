#!/usr/bin/python
# To count stats for data and store it

import json
import os
import shutil
import sys
import tempfile

def main():

    jsons = [x for x in os.listdir('..') if x.endswith('json')]

    output_dictionary = dict()

    print("Starting scanning.")
    for f in jsons:

        with open('{}/{}'.format('..', f), 'r') as f:
            images = json.load(f)

        for image in images:
            if 'annotations' not in image:
                continue

            # Count the annotations (total and correct) marked for each image.
            for annotation in image['annotations']:
                label_type = annotation['class']
                if label_type not in output_dictionary:
                    output_dictionary[label_type] = {'count': 0,
                                                     'good': 0}

                output_dictionary[label_type]['count'] += 1
                if 'status' in image and image['status'] == 'Good':
                    output_dictionary[label_type]['good'] += 1

    with open("count.json", 'w') as m:
        json.dump(output_dictionary, m, indent=4)
    print("\nDone.")

if __name__ == '__main__':
    main()
