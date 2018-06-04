#!/usr/bin/python
"""
@author Ryan Summers
@date 2-27-2018

@brief Gets information about available datasets on the robosub server.
"""

from __future__ import print_function

import json
import os
import progressbar
import pysftp
import shutil
import sys
import tempfile

try:
    input = raw_input
except:
    pass

# download progress bar
bar = progressbar.ProgressBar()

# update progress bar
def progress(done, total):
    bar.update(done)

def get_json_stats(sftp):
    """ Collects metadata about the completed datasets. """

    print('Grabbing JSON information...')
    global bar
    total_size = sftp.stat("count/count.json")
    bar = progressbar.ProgressBar(max_value=total_size.st_size)
    sftp.get("count/count.json", callback=progress)
    bar.finish()

    output_dictionary = dict()

    with open("count.json", 'r') as f:
        output_dictionary = json.load(f)

    os.remove("count.json")

    return output_dictionary


def app(args):
    """ Main entry point for returning data to the server.

    Arguments
        args: [Unused]

    """
    password = os.environ.get('ROBOSUB_SFTP_PASSWORD')
    if password is None:
        print('To suppress this prompt, please set the ROBOSUB_SFTP_PASSWORD ',
                end='')
        print('environment variable.')
        password = input('Please enter the Robosub SFTP password: ')

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

        with sftp.cd('in_progress/clarification/'):
            current_clarification_tars = [x for x in sftp.listdir() if x.endswith('.tar')]

        with sftp.cd('new/'):
            new_tars = [x for x in sftp.listdir() if x.endswith('.tar')]

        with sftp.cd('unvalidated/'):
            unvalidated_tars = [x for x in sftp.listdir() if x.endswith('.tar')]

        with sftp.cd('done/'):
            done_tars = [x for x in sftp.listdir() if x.endswith('.tar')]
            done_dict = get_json_stats(sftp)

        with sftp.cd('clarification/'):
            clarified_tars = [x for x in sftp.listdir() if x.endswith('.tar')]

        print('Image sets waiting to be labeled: {}'.format(len(new_tars)))
        print('Image sets waiting to be validated: {}'.format(len(unvalidated_tars)))
        print('Image sets waiting to be clarified: {}'.format(len(clarified_tars)))
        print('Image sets being validated: {}'.format(len(current_validation_tars)))
        print('Image sets being labeled: {}'.format(len(current_labeling_tars)))
        print('Image sets being clarified: {}'.format(len(current_clarification_tars)))
        print('')
        print('Done dataset stats:')
        print('Label\tNumber\tCorrect')
        for key in done_dict:
            print('{}\t{}\t{}'.format(key, done_dict[key]['count'], done_dict[key]['good']))
