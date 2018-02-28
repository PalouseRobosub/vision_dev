#!/usr/bin/python
"""
@author Ryan Summers
@date 2-27-2018

@brief Gets information about available datasets on the robosub server.
"""

import json
import os
import progressbar
import pysftp
import shutil
import sys
import tempfile


def get_json_stats(sftp):
    """ Collects metadata about the completed datasets. """
    jsons = [x for x in sftp.listdir() if x.endswith('json')]

    working_directory = tempfile.mkdtemp()

    output_dictionary = dict()

    # Get information about each JSON in the directory.
    print 'Grabbing JSON information...'
    bar = progressbar.ProgressBar(max_value=len(jsons))
    for i, f in enumerate(jsons):
        bar.update(i)
        sftp.get(f, localpath='{}/{}'.format(working_directory, f))
        with open('{}/{}'.format(working_directory, f), 'r') as f:
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
    bar.finish()

    # Remove the temporary folder where the JSONs were stored.
    shutil.rmtree(working_directory)

    return output_dictionary


def app(args):
    """ Main entry point for returning data to the server.

    Arguments
        args: [Unused]

    """
    password = os.environ.get('ROBOSUB_SFTP_PASSWORD')
    if password is None:
        print 'To suppress this prompt, please set the ROBOSUB_SFTP_PASSWORD ',
        print 'environment variable.'
        password = raw_input('Please enter the Robosub SFTP password: ')

    # Open an SFTP connection with the robosub server.
    with pysftp.Connection('robosub.eecs.wsu.edu',
                username='sftp_user',
                password=password,
                default_path='/data/vision/labeling') as sftp:

        # Check to see if there is an in-progress labeling or validation
        # session.
        with sftp.cd('in_progress/labeling/'):
            current_labeling_tars = [x for x in sftp.listdir() if x.endswith('.tar')]

        with sftp.cd('in_progress/validation/'):
            current_validation_tars = [x for x in sftp.listdir() if x.endswith('.tar')]

        with sftp.cd('new/'):
            new_tars = [x for x in sftp.listdir() if x.endswith('.tar')]

        with sftp.cd('unvalidated/'):
            unvalidated_tars = [x for x in sftp.listdir() if x.endswith('.tar')]

        with sftp.cd('done/'):
            done_tars = [x for x in sftp.listdir() if x.endswith('.tar')]
            done_dict = get_json_stats(sftp)

        print 'Image sets waiting to be labeled: {}'.format(len(new_tars))
        print 'Image sets waiting to be validated: {}'.format(len(unvalidated_tars))
        print 'Image sets being validated: {}'.format(len(current_validation_tars))
        print 'Image sets being labeled: {}'.format(len(current_labeling_tars))
        print ''
        print 'Done dataset stats:'
        print 'Label\tNumber\tCorrect'
        for key in done_dict:
            print '{}\t{}\t{}'.format(key, done_dict[key]['count'], done_dict[key]['good'])
