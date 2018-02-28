#!/usr/bin/python

import os
import tempfile
import progressbar
import tarfile
import json
import glob
import shutil
import pysftp


def app(args):

    password = os.environ.get('ROBOSUB_SFTP_PASSWORD')
    if password is None:
        print 'To suppress this prompt, please set the ROBOSUB_SFTP_PASSWORD environment variable.'
        password = raw_input('Please enter the Robosub SFTP password: ')

    with pysftp.Connection('robosub.eecs.wsu.edu',
                username='sftp_user',
                password=password,
                default_path='/data/vision/labeling/done') as sftp:
        files = sftp.listdir()

        tarballs = [x for x in files if x.endswith('.tar')]
        jsons = [x for x in files if x.endswith('.json')]

        tar_bases = [os.path.splitext(x)[0] for x in tarballs]
        json_bases = [os.path.splitext(x)[0] for x in jsons]

        groups = set(tar_bases).intersection(json_bases)

        tar_directory = tempfile.mkdtemp()

        # Download the tarballs.
        print 'Downloading tarballs...'
        download_bar = progressbar.ProgressBar(max_value=len(groups))
        for i, tar_base in enumerate(groups):
            download_bar.update(i)
            tar_name = tar_base + '.tar'
            json_name = tar_base + '.json'
            sftp.get(tar_name, localpath='{}/{}'.format(tar_directory, tar_name))
            sftp.get(json_name, localpath='{}/{}'.format(tar_directory, json_name))

        download_bar.finish()

    working_directory = tempfile.mkdtemp()

    print 'Combining tarballs...'
    bar = progressbar.ProgressBar(max_value=len(groups))
    annotations = []
    for i, dataset in enumerate(groups):
        bar.update(i)
        json_fname = tar_directory + '/' + dataset + '.json'
        tar_fname = tar_directory + '/' + dataset + '.tar'

        with tarfile.TarFile(tar_fname, mode='r') as tf:
            tf.extractall(working_directory)

        with open(json_fname, 'r') as f:
            new_annotations = json.load(f)

        tar_dir = os.path.splitext(os.path.basename(tar_fname))[0]

        for annotation in new_annotations:
            fname = annotation['filename']
            annotation['filename'] = tar_dir + '/' + fname

        annotations += new_annotations

    bar.finish()

    with open('{}/labels.json'.format(working_directory), 'w') as f:
        json.dump(annotations, f, indent=4)

    print 'Writing final tarball...'
    with tarfile.TarFile(args.tarball, mode='w') as tf:
        tf.add(working_directory, arcname='dataset')

    shutil.rmtree(working_directory)
    shutil.rmtree(tar_directory)

    print 'Tarfile saved to {}.'.format(args.tarball)
