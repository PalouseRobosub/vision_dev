import argparse
import json
import os
import shutil
import random
import sys
import progressbar

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Converts a sloth annotation into KITTI format.')
    parser.add_argument('annotation', type=str, help='Annotation file location')
    parser.add_argument('output', type=str, help='Location to place the KITTI dataset.')
    parser.add_argument('--validate', type=float, help='Percentage of images to select for validation.', default=10)
    args = parser.parse_args()

    if args.validate >= 100:
        print '--validate must be less than 100'
        sys.exit(-1)

    with open(args.annotation) as f:
        data_raw = json.load(f)
        data = [x for x in data_raw if 'status' in x and str(x['status']) == 'Good']

    train_path = '{}/train'.format(args.output)
    validation_path = '{}/val'.format(args.output)
    annotation_path = os.path.dirname(args.annotation)

    os.mkdir(args.output)
    os.mkdir(train_path)
    os.mkdir('{}/images'.format(train_path))
    os.mkdir('{}/labels'.format(train_path))

    os.mkdir(validation_path)
    os.mkdir('{}/images'.format(validation_path))
    os.mkdir('{}/labels'.format(validation_path))

    random.shuffle(data)

    split_index = int(len(data) * (args.validate / 100.0))

    print '{} images total. {} set aside for validation'.format(len(data), split_index)

    training_data = data[split_index:]
    validation_data = data[0:split_index]

    bar = progressbar.ProgressBar(max_value=len(training_data))
    for i, annotation in enumerate(training_data):
        bar.update(i)
        fname = annotation['filename']
        fpath = annotation_path + '/' + fname

        shutil.copy(fpath, '{}/images/{}'.format(train_path, fname))

        file_base = os.path.splitext(fname)[0]
        with open('{}/labels/{}.txt'.format(train_path, file_base), 'w') as f:
            for label in annotation['annotations']:
                label_name = label['class']
                left = label['x']
                right = left + label['width']
                top = label['y']
                bottom = label['height']
                f.write('{} 0 0 0 {} {} {} {} 0 0 0 0 0 0 0\n'.format(
                            label_name, left, top, right, bottom))

    bar = progressbar.ProgressBar(max_value=len(validation_data))
    for i, annotation in enumerate(validation_data):
        bar.update(i)
        fname = annotation['filename']
        fpath = annotation_path + '/' + fname

        shutil.copy(fpath, '{}/images/{}'.format(validation_path, fname))

        file_base = os.path.splitext(fname)[0]
        with open('{}/labels/{}.txt'.format(validation_path, file_base), 'w') as f:
            for label in annotation['annotations']:
                label_name = label['class']
                left = label['x']
                right = left + label['width']
                top = label['y']
                bottom = label['height']
                f.write('{} 0 0 0 {} {} {} {} 0 0 0 0 0 0 0\n'.format(
                            label_name, left, top, right, bottom))
