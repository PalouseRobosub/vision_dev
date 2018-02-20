import argparse
import os
import tempfile
import progressbar
import tarfile
import json
import glob
import shutil


def move(src, dest):
    print 'Moving {} -> {}'.format(src, dest)
    for f in os.listdir(src):
        f = src + '/' + f
        if os.path.isfile(f):
            if src != dest:
                shutil.move(f, dest)
        else:
            move(f, dest)
            os.rmdir(f)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Collects multiple tarballs into a single dataset.')
    parser.add_argument('directory', type=str, help='The directory containing tarballs and JSONs')
    parser.add_argument('tarball', type=str, help='The name of the final tarball to create.')

    args = parser.parse_args()

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
            annotations += json.load(f)

    bar.finish()

    with open('{}/labels.json'.format(working_directory), 'w') as f:
        json.dump(annotations, f, indent=4)

    # Flatten the tarfile directory structure
    move(working_directory, working_directory)

    print 'Writing final tarbal...'
    with tarfile.TarFile(args.tarball, mode='w') as tf:
        tf.add(working_directory, arcname='dataset')

    shutil.rmtree(working_directory)

    print 'Tarfile saved to {}.'.format(args.tarball)
