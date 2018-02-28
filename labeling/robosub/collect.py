#!/usr/bin/python

import os
import tempfile
import progressbar
import tarfile
import json
import glob
import shutil


def app(args):
    files = glob.glob(args.directory + '/*')

    tarballs = [x for x in files if x.endswith('.tar')]
    jsons = [x for x in files if x.endswith('.json')]

    tar_bases = [os.path.splitext(x)[0] for x in tarballs]
    json_bases = [os.path.splitext(x)[0] for x in jsons]

    groups = set(tar_bases).intersection(json_bases)

    working_directory = tempfile.mkdtemp()

    print 'Combining tarballs...'
    bar = progressbar.ProgressBar(max_value=len(groups))
    annotations = []
    for i, dataset in enumerate(groups):
        bar.update(i)
        json_fname = dataset + '.json'
        tar_fname = dataset + '.tar'

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

    print 'Tarfile saved to {}.'.format(args.tarball)
