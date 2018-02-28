#!/usr/bin/python

import pysftp
import os
import sys
import shutil

def app(args):
    password = os.environ.get('ROBOSUB_SFTP_PASSWORD')
    if password is None:
        print 'To suppress this prompt, please set the ROBOSUB_SFTP_PASSWORD environment variable.'
        password = raw_input('Please enter the Robosub SFTP password: ')

    tar_name = os.path.splitext(os.path.basename(args.annotation))[0] + '.tar'

    with pysftp.Connection('robosub.eecs.wsu.edu',
                username='sftp_user',
                password=password,
                default_path='/data/vision/labeling') as sftp:

        # Check to see if there is an in-progress labeling or validation
        # session.
        with sftp.cd('in_progress/labeling/'):
            labeling_tars = [x for x in sftp.listdir() if x.endswith('.tar')]

        with sftp.cd('in_progress/validation/'):
            validation_tars = [x for x in sftp.listdir() if x.endswith('.tar')]

        in_validation = False

        if tar_name in labeling_tars:
            src_dir = 'in_progress/labeling/'
            dest_dir = 'unvalidated/'
        elif tar_name in validation_tars:
            in_validation = True
            src_dir = 'in_progress/validation/'
            dest_dir = 'done/'
        else:
            print 'The supplied JSON name does not match any in-progress validation or labeling sessions.'
            print 'Current labeling sessions: {}'.format(labeling_tars)
            print 'Current validation sessions: {}'.format(validation_tars)
            sys.exit(-1)

        with sftp.cd(src_dir):
            tars = [x for x in sftp.listdir() if x.endswith('.tar')]

        if tar_name not in tars:
            print 'The provided annotations dataset was not found in '
            print 'robosub.eecs.wsu.edu:/data/vision/labeling/{}'.format(src_dir)
            sys.exit(-1)

        # Upload the JSON to the server.
        with sftp.cd(dest_dir):
            sftp.put(args.annotations)

        # Move the found tar into unvalidated/
        sftp.rename('{}/{}'.format(src_dir, tar_name),
                    '{}/{}'.format(dest_dir, tar_name))

        # Remove the ownership and annotation file.
        if in_validation:
            sftp.remove('{}/{}'.format(src_dir, os.path.basename(args.annotations)))
        sftp.remove('{}/{}.owner'.format(src_dir, tar_name))

        if args.auto_delete:
            delete = True
        else:
            user_input = raw_input('Delete folder `{}/`? (y/n): '.format(os.path.dirname(args.annotations)))
            delete = user_input == 'y' or user_input == 'Y'

        if delete:
            print 'Deleting {}/'.format(os.path.dirname(args.annotations))
            shutil.rmtree(os.path.dirname(args.annotations))
