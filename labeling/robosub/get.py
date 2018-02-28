#!/usr/bin/python

import glob
import json
import pysftp
import tarfile
import tempfile
import sys
import os

def app(args):
    password = os.environ.get('ROBOSUB_SFTP_PASSWORD')
    if password is None:
        print 'To suppress this prompt, please set the ROBOSUB_SFTP_PASSWORD environment variable.'
        password = raw_input('Please enter the Robosub SFTP password: ')

    if args.validation:
        src_dir = 'unvalidated/'
        dest_dir = 'in_progress/validation/'
    else:
        src_dir = 'new/'
        dest_dir = 'in_progress/labeling/'

    data_owner = os.environ.get('ROBOSUB_WHO_AM_I')
    if data_owner is None:
        print 'To suppress this prompt, please set the environment variable ROBOSUB_WHO_AM_I to your name.'
        data_owner = raw_input('Please enter your name (First Last): ')

    with pysftp.Connection('robosub.eecs.wsu.edu',
            username='sftp_user',
            password=password,
            default_path='/data/vision/labeling') as sftp:
        with sftp.cd(src_dir):
            tars = [x for x in sftp.listdir() if str(x).endswith('.tar')]

        if len(tars) == 0:
            print 'No data was available at robosub.eecs.wsu.edu:/data/vision/labeling/{}'.format(src_dir)
            sys.exit(0)

        tar = str(tars[0])

        print 'Grabbing {}'.format(tar)

        tar_name = os.path.splitext(tar)[0]

        # Move tar to in_progress -> It's ours now.
        sftp.rename(src_dir + tar, dest_dir + tar)

        # Make an owner file on the server to denote that we own the tar.
        owner_file = '{}.owner'.format(tar)
        temp_dir = tempfile.mkdtemp()
        with open('{}/{}'.format(temp_dir, owner_file), 'w') as f:
            f.write(data_owner + '\n')

        with sftp.cd(dest_dir):
            sftp.put('{}/{}'.format(temp_dir, owner_file))

        # Get the tar.
        sftp.get(dest_dir + tar)

        # Untar the data
        with tarfile.TarFile(tar) as tf:
            tf.extractall()

        # If this is validation, grab the JSON from the server.
        if args.validation:
            tar_base = os.path.splitext(tar)[0]
            json_name = tar_base + '.json'

            sftp.rename(src_dir + json_name, dest_dir + json_name)
            with sftp.cd(dest_dir):
                print 'Grabbing {}'.format(json_name)
                sftp.get(json_name, localpath='{}/{}'.format(tar_base, json_name))
        else:
            # Generate labeling JSON for data to begin labeling.
            annotations = []
            for f in sorted(glob.glob('{}/*.jpg'.format(tar_name))):
                annotations.append({'annotations': [],
                                    'class': 'image',
                                    'filename': os.path.basename(f),
                                    'unlabeled': True})

            with open('{}/{}.json'.format(tar_name, tar_name), 'w') as f:
                json.dump(annotations, f, indent=4)

    # Remove the tar file - We don't need it anymore.
    os.remove(tar)

    print 'Images available in {}'.format(tar_name)
