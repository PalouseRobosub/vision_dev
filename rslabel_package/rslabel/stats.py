#!/usr/bin/python
"""
@author Ryan Summers
@date 4-4-2018

@brief Gets statistics about labeling progress for all users.
"""

from __future__ import print_function

import glob
import json
import os
import pysftp
import shutil
import tempfile

try:
    input = raw_input
except:
    pass


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

        label_stats = dict()
        dname = tempfile.mkdtemp()
        sftp.get_d('history', dname)
        for fname in glob.glob('{}/*.log'.format(dname)):
            with open(fname, 'r') as f:
                data = json.load(f)
            for entry in data:
                user = entry['owner']
                log = entry['stats']
                if user not in label_stats:
                    label_stats[user] = log
                else:
                    for key in log:
                        label_stats[user][key] += log[key]
        shutil.rmtree(dname)

    headings = ['User', 'Images Labeled', 'Labels Added', 'Images Validated', 'Labels Validated']
    row_format = '{:>20}' * len(headings)
    print(row_format.format(*headings))

    for user in label_stats:
        images_labeled = label_stats[user]['images_labeled']
        labels_added = label_stats[user]['labels_added']
        images_validated = label_stats[user]['images_validated']
        labels_validated = label_stats[user]['labels_validated']
        print(row_format.format(user, images_labeled, labels_added, images_validated, labels_validated))
