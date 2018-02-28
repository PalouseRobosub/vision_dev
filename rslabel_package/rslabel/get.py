#!/usr/bin/python
"""
@author Ryan Summers
@date 2-27-2018

@brief Gets data for labelling or validation from the Robosub server.
"""

import glob
import json
import os
import pysftp
import shutil
import sys
import tarfile
import tempfile


def app(args):
    """ Main entry point for the dataset getter application.

    Arguments
        args: Passed in from argparse. Should have a named element `validation`
              that specifies whether or not the dataset to get should be for
              validation or labeling.
    """

    if args.validation:
        src_dir = 'unvalidated/'
        dest_dir = 'in_progress/validation/'
    else:
        src_dir = 'new/'
        dest_dir = 'in_progress/labeling/'

    # Grab the username for data ownership and the SFTP password.
    data_owner = os.environ.get('ROBOSUB_WHO_AM_I')
    if data_owner is None:
        print 'To suppress this prompt, please set the environment variable ',
        print 'ROBOSUB_WHO_AM_I to your name.'
        data_owner = raw_input('Please enter your name (First Last): ')

    password = os.environ.get('ROBOSUB_SFTP_PASSWORD')
    if password is None:
        print 'To suppress this prompt, please set the ROBOSUB_SFTP_PASSWORD ',
        print 'environment variable.'
        password = raw_input('Please enter the Robosub SFTP password: ')

    # Create an SFTP connection to the robosub server for getting data.
    with pysftp.Connection('robosub.eecs.wsu.edu',
            username='sftp_user',
            password=password,
            default_path='/data/vision/labeling') as sftp:
        with sftp.cd(src_dir):
            tars = [x for x in sftp.listdir() if str(x).endswith('.tar')]

        if len(tars) == 0:
            print 'No data is available at ',
            print 'robosub.eecs.wsu.edu/data/vision/labeling/' + src_dir
            sys.exit(0)

        tar = str(tars[0])

        print 'Grabbing {}'.format(tar)

        tar_name = os.path.splitext(tar)[0]

        # Move tar to in_progress - This prevents other users from working on
        # the same data as us.
        sftp.rename(src_dir + tar, dest_dir + tar)

        # Make an owner file on the server to specify who currently has the
        # dataset checked out.
        owner_file = '{}.owner'.format(tar)
        temp_dir = tempfile.mkdtemp()
        with open('{}/{}'.format(temp_dir, owner_file), 'w') as f:
            f.write(data_owner + '\n')

        with sftp.cd(dest_dir):
            sftp.put('{}/{}'.format(temp_dir, owner_file))

        shutil.rmtree(temp_dir)

        # Get the tar and untar it.
        sftp.get(dest_dir + tar)

        with tarfile.TarFile(tar) as tf:
            tf.extractall()

        # If this is validation (or is partially completed), grab the JSON from
        # the server.
        tar_base = os.path.splitext(tar)[0]
        json_name = tar_base + '.json'

        if args.validation:
            sftp.rename(src_dir + json_name, dest_dir + json_name)
            with sftp.cd(dest_dir):
                print 'Grabbing {}'.format(json_name)
                sftp.get(json_name,
                         localpath='{}/{}'.format(tar_base, json_name))
        else:
            with sftp.cd(src_dir):
                elements = sftp.listdir()

            if json_name in elements:
                print 'Grabbing {}'.format(json_name)
                with sftp.cd(src_dir):
                    sftp.get(json_name,
                             localpath='{}/{}'.format(tar_base, json_name))
                sftp.rename('{}/{}'.format(src_dir, json_name),
                            '{}/{}'.format(dest_dir, json_name))
            else:
                # Generate labeling JSON for data to begin labeling if it is not
                # available on the server.
                print 'Generating JSON for the dataset.'
                annotations = []
                for f in sorted(glob.glob('{}/*.jpg'.format(tar_name))):
                    annotations.append({'annotations': [],
                                        'class': 'image',
                                        'filename': os.path.basename(f),
                                        'unlabeled': True})

                with open('{}/{}.json'.format(tar_base, json_name), 'w') as f:
                    json.dump(annotations, f, indent=4)

    # Remove the tar file - We don't need it anymore.
    os.remove(tar)

    print 'Images available in {}'.format(tar_name)
