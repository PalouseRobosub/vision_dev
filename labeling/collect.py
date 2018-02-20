import argparse
import os
import tempfile
import progressbar
import tarfile
import json


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Collects multiple tarballs into a single dataset.')
    parser.add_argument('directory', type=str, help='The directory containing tarballs and JSONs')
    parser.add_argument('tarball', type=str, help='The name of the final tarball to create.')

    args = parser.parse_args()

    files = glob.glob(directory)

    tarballs = [x for x in files if x.endswith('.tar')]
    jsons = [x for x in files if x.endsiwth('.json')]

    tar_bases = [os.path.splitext(x)[0] for x in tarballs]
    json_bases = [os.path.splitext(x)[0] for x in jsons]

    groups = set(tar_bases).intersection(json_bases)

    working_directory = tempfile.mkdtemp()

    print 'Combining tarballs...'
    bar = progressbar.ProgressBar(max_value=len(groups))
    annotations = []
    for dataset in groups:
        bar.next()
        json_fname = dataset + '.json'
        tar_fname = dataset + '.tar'

        with tarfile.TarFile(tar_fname, 'r') as tf:
            tf.extractall(working_directory)

        with open(json_fname, 'r') as f:
            annotations.append(json.load(f))

    bar.finish()

    with open('{}/labels.json'.format(working_directory), 'w') as f:
        json.dump(annotations, f)

    with tarfile.TarFile(args.tarfile, 'w') as tf:
        tf.add(working_directory, arcname=os.path.basename(working_directory))

    shutil.rmtree(working_directory)

    print 'Tarfile saved to {}.'.format(working_directory)
