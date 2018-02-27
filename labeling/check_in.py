#!/usr/bin/python

import argparse
import pysftp
import os
import sys
import shutil

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Check in Robosub image labels.')
    parser.add_argument('annotations', type=str, help='The annotations file to upload.')
    parser.add_argument('--validation', action='store_true')
    parser.add_argument('--auto-delete', action='store_true')

    args = parser.parse_args()

    password = os.environ.get('ROBOSUB_SFTP_PASSWORD')
    if password is None:
        print 'To suppress this prompt, please set the ROBOSUB_SFTP_PASSWORD environment variable.'
        password = raw_input('Please enter the Robosub SFTP password: ')

    # Figure out the source and destination directory based upon input flag.
    if args.validation:
        dest_dir = 'done/'
        src_dir = 'in_progress/validation/'
    else:
        dest_dir = 'unvalidated/'
        src_dir = 'in_progress/labeling/'

    with pysftp.Connection('robosub.eecs.wsu.edu',
                username='sftp_user',
                password=password,
                default_path='/data/vision/labeling') as sftp:
        with sftp.cd(src_dir):
            tars = [x for x in sftp.listdir() if x.endswith('.tar')]

        annotation_tar = os.path.splitext(os.path.basename(args.annotations))[0] + '.tar'
        if annotation_tar not in tars:
            print 'The provided annotations dataset was not found robosub.eecs.wsu.edu:/data/vision/labeling/{}'.format(src_dir)
            sys.exit(-1)

        # Upload the JSON to the server.
        with sftp.cd(dest_dir):
            sftp.put(args.annotations)

        # Move the found tar into unvalidated/
        sftp.rename('{}/{}'.format(src_dir, annotation_tar),
                    '{}/{}'.format(dest_dir, annotation_tar))

        # Remove the ownership and annotation file.
        if args.validation:
            sftp.remove('{}/{}'.format(src_dir, os.path.basename(args.annotations)))
        sftp.remove('{}/{}.owner'.format(src_dir, annotation_tar))

        if args.auto_delete:
            delete = True
        else:
            user_input = raw_input('Delete folder `{}/`? (y/N): '.format(os.path.dirname(args.annotations)))
            delete = user_input == 'y' or user_input == 'Y'

        if delete:
            shutil.rmtree(os.path.dirname(args.annotations))

