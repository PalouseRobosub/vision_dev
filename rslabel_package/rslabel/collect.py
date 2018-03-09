#!/usr/bin/python
"""
@author Ryan Summers
@date 2-27-2018

@brief Collects all completed tarballs into a single labelled dataset.
"""

import glob
import json
import os
import progressbar
import pysftp
import shutil
import tarfile
import tempfile

try:
    input = raw_input
except:
    pass


def app(args):
    """ The main entry point to the collection application.

    Arguments
        arg: A structure passed in from argparse containing a single named
             element, tarball. Tarball indicates the location to save the final
             unified tarball locally.
    """

    password = os.environ.get('ROBOSUB_SFTP_PASSWORD')
    if password is None:
        print('To suppress this prompt, please set the ROBOSUB_SFTP_PASSWORD ',
                end='')
        print('environment variable.')
        password = input('Please enter the Robosub SFTP password: ')

    # Generate the SFTP connection to the robosub server for grabbing tarballs.
    with pysftp.Connection('robosub.eecs.wsu.edu',
                username='sftp_user',
                password=password,
                default_path='/data/vision/labeling/done') as sftp:
        files = sftp.listdir()

        # Determine which tarballs are valid labelled datasets by intersecting
        # available JSONs and tar archives.
        tarballs = [x for x in files if x.endswith('.tar')]
        jsons = [x for x in files if x.endswith('.json')]

        tar_bases = [os.path.splitext(x)[0] for x in tarballs]
        json_bases = [os.path.splitext(x)[0] for x in jsons]

        groups = set(tar_bases).intersection(json_bases)

        # Download the tarballs into a temporary directory.
        tar_directory = tempfile.mkdtemp()

        print('Downloading tarballs...')
        download_bar = progressbar.ProgressBar(max_value=len(groups))
        for i, tar_base in enumerate(groups):
            download_bar.update(i)
            tar_name = tar_base + '.tar'
            json_name = tar_base + '.json'
            sftp.get(tar_name, localpath='{}/{}'.format(tar_directory,
                                                        tar_name))
            sftp.get(json_name, localpath='{}/{}'.format(tar_directory,
                                                         json_name))

        download_bar.finish()

    # Untar everything into a temporary working directory
    working_directory = tempfile.mkdtemp()

    print('Combining tarballs...')
    bar = progressbar.ProgressBar(max_value=len(groups))
    annotations = []
    for i, dataset in enumerate(groups):
        bar.update(i)
        json_fname = tar_directory + '/' + dataset + '.json'
        tar_fname = tar_directory + '/' + dataset + '.tar'

        # Extract the tarball and read the annotations.
        with tarfile.TarFile(tar_fname, mode='r') as tf:
            tf.extractall(working_directory)

        with open(json_fname, 'r') as f:
            new_annotations = json.load(f)

        tar_dir = os.path.splitext(os.path.basename(tar_fname))[0]

        # Prepend the tar directory name with each annotation filename to keep
        # the original directory structure.
        for annotation in new_annotations:
            fname = annotation['filename']
            annotation['filename'] = tar_dir + '/' + fname

        annotations += new_annotations

    bar.finish()

    # Write all of the annotations with the new directory structure into the
    # final tarball.
    with open('{}/labels.json'.format(working_directory), 'w') as f:
        json.dump(annotations, f, indent=4)

    print('Writing final tarball...')
    with tarfile.TarFile(args.tarball, mode='w') as tf:
        tf.add(working_directory, arcname='dataset')

    # Remove the temporary working directories.
    shutil.rmtree(working_directory)
    shutil.rmtree(tar_directory)

    print('Tarfile saved to {}.'.format(args.tarball))
