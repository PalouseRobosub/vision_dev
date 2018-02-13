#!/usr/bin/python
"""
Code based on:

http://github.com/allanzelener/YAD2K/blob/master/voc_conversion_scripts/voc_to_tfrecords.py
http://github.com/tensorflow/models/blob/master/research/object_detection/dataset_tools/create_pet_tf_record.py
"""

import os
import io
import hashlib
import PIL
import sys
from datetime import datetime
from progress.bar import IncrementalBar as Bar
import random

import numpy as np
import json
from object_detection.utils import dataset_util

import argparse

parser = argparse.ArgumentParser(
    description='Convert sloth json dataset to TFRecords.',
    formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
parser.add_argument('input',
    help='input json file')
parser.add_argument('outdir',
    help='directory to place output files')
parser.add_argument('--test', type=float,
    help = "(0-1), portion of dataset to use for testing",
    default=0.1)
args = parser.parse_args()

import tensorflow as tf

# Small graph for image decoding
decoder_sess = tf.Session()
image_placeholder = tf.placeholder(dtype=tf.string)
decoded_jpg = tf.image.decode_jpeg(image_placeholder, channels=3)


def process_image(image_path):
    """Decode image at given path."""
    with open(image_path, 'rb') as f:
        image_data = f.read()
    image = decoder_sess.run(decoded_jpg,
                             feed_dict={image_placeholder: image_data})
    assert len(image.shape) == 3
    height = image.shape[0]
    width = image.shape[1]
    assert image.shape[2] == 3
    return image_data, height, width

def convert_to_example(image_path, boxes):
    """Convert Pascal VOC ground truth to TFExample protobuf.

    Parameters
    ----------
    image_data : bytes
        Encoded image bytes.
    boxes : dict
        Bounding box corners and class labels
    filename : string
        Path to image file.
    height : int
        Image height.
    width : int
        Image width.

    Returns
    -------
    example : protobuf
        Tensorflow Example protobuf containing image and bounding boxes.
    """
    with tf.gfile.GFile(image_path, 'rb') as fid:
        encoded_jpg = fid.read()
    encoded_jpg_io = io.BytesIO(encoded_jpg)
    image = PIL.Image.open(encoded_jpg_io)
    if image.format != 'JPEG':
        raise ValueError('Image format not jpeg')
    key = hashlib.sha256(encoded_jpg).hexdigest()

    (width, height) = image.size

    class_label = [b['class_label'].encode('utf8') for b in boxes]
    class_index = [b['class_index'] for b in boxes]
    ymin = [b['y_min'] for b in boxes]
    xmin = [b['x_min'] for b in boxes]
    ymax = [b['y_max'] for b in boxes]
    xmax = [b['x_max'] for b in boxes]

    example = tf.train.Example(features=tf.train.Features(feature={
        'image/height': dataset_util.int64_feature(height),
        'image/width': dataset_util.int64_feature(width),
        'image/filename': dataset_util.bytes_feature(
            os.path.basename(image_path).encode('utf8')),
        'image/source_id': dataset_util.bytes_feature(
            os.path.basename(image_path).encode('utf8')),
        'image/key/sha256': dataset_util.bytes_feature(key.encode('utf8')),
        'image/encoded': dataset_util.bytes_feature(encoded_jpg),
        'image/format': dataset_util.bytes_feature('jpeg'.encode('utf8')),
        'image/object/bbox/xmin': dataset_util.float_list_feature(xmin),
        'image/object/bbox/xmax': dataset_util.float_list_feature(xmax),
        'image/object/bbox/ymin': dataset_util.float_list_feature(ymin),
        'image/object/bbox/ymax': dataset_util.float_list_feature(ymax),
        'image/object/class/text': dataset_util.bytes_list_feature(class_label),
        'image/object/class/label': dataset_util.int64_list_feature(class_index),
        }))

    return example

def extract_boxes(annotations, img_height, img_width, classes):

    boxes = list()
    for annotation in annotations:
        # tfrecord wants numerical index for class labels
        class_label = str(annotation['class'])
        class_index = classes.index(class_label) + 1
        x = annotation['x']
        y = annotation['y']
        width = annotation['width']
        height = annotation['height']
        xmin = float(x) / img_width
        ymin = float(y) / img_height
        xmax = float(x + width) / img_width
        ymax = float(y + height) / img_height

        # bounds checking
        if (xmin < 0.0):
            xmin = 0.0
        if (ymin < 0.0):
            ymin = 0.0
        if (xmax > 1.0):
            xmax = 1.0
        if (ymax > 1.0):
            ymax = 1.0

        assert (xmin >= 0.0) and (xmin <= 1.0), "xmin: {}".format(xmin)
        assert (xmax >= 0.0) and (xmax <= 1.0), "xmax: {}".format(xmax)
        assert (ymin >= 0.0) and (ymin <= 1.0), "ymin: {}".format(ymin)
        assert (ymax >= 0.0) and (ymax <= 1.0), "ymax: {}".format(ymax)

        box = {
                'class_label': class_label,
                'class_index': class_index,
                'y_min': ymin,
                'x_min': xmin,
                'y_max': ymax,
                'x_max': xmax
              }

        boxes.append(box)

    return boxes

def find_classes(json_data):
    classes = list()
    for item in json_data:
        for annotation in item['annotations']:
            mclass = str(annotation['class'])
            if  mclass not in classes:
                classes.append(mclass)
    classes.sort()
    return classes

def write_label_map(classes, outfile):
    f = open(outfile, "w")
    for index, label in enumerate(classes, 1):
        f.write("item {\n")
        f.write("  id:{}\n".format(index))
        f.write("  name: '{}'\n".format(label))
        f.write("}\n\n")

def create_record(outfile, json_data, classes, input_path, progress):
    # Record keeping
    processed = 0
    skipped_not_good = 0
    skipped_not_found = 0
    writer = tf.python_io.TFRecordWriter(outfile)
    #counter = 0
    for item in json_data:
        # Some nice progress updates
        #counter += 1
        progress.next()

        # Skip files marked as bad
        if str(item["status"]) != "Good":
            skipped_not_good += 1
            continue

        filepath = os.path.join(input_path, str(item['filename']))
        filename = os.path.basename(filepath)
        try:
            image_data, height, width = process_image(filepath)
        except IOError:
            #print "could not find {}, skipping".format(filepath)
            skipped_not_found += 1
            continue

        boxes = extract_boxes(item['annotations'],
                img_height=height,
                img_width=width,
                classes=classes)

        tfexample = convert_to_example(filepath, boxes)

        writer.write(tfexample.SerializeToString())
        processed += 1
    return processed, skipped_not_good, skipped_not_found

def clean_dataset(l):
    tmplist = list()
    skipped = 0

    for entry in l:
        if str(entry["status"]) == "Good" and \
           len(entry['annotations']) > 0:
           tmplist.append(entry)
        else:
            skipped += 1

    return tmplist, skipped

def _main(args):
    """Locate files for train and test sets and then generate TFRecords."""
    input_path = args.input
    input_path = os.path.expanduser(input_path)
    output_path = args.outdir

    train_path = os.path.join(output_path, 'train.record')
    test_path = os.path.join(output_path, 'test.record')
    label_path = os.path.join(output_path, 'label_map.pbtxt')

    json_data = json.load(open(args.input, 'r'))
    json_data, skipped = clean_dataset(json_data)

    classes =  find_classes(json_data)
    print "Found the following classes:"
    for x in classes:
        print "  {}".format(x)
    print ""

    write_label_map(classes, label_path)
    print "label map saved to {}".format(label_path)

    # shuffle our data, split into training and testing segments
    random.shuffle(json_data)
    split_index = int(len(json_data)*(1.0 - args.test))
    training_data = json_data[0 : split_index]
    testing_data = json_data[split_index:]

    # Create the training data
    progress = Bar('Creating training data', max=len(training_data),
            suffix='%(percent)d%%')
    processed1, skipped_not_good1, skipped_not_found1 = \
        create_record(train_path, training_data, classes,
                os.path.dirname(input_path), progress)
    progress.finish()
    print "{} images saved to {}.".format(len(training_data), train_path)

    # Create the testing data
    progress = Bar('Creating testing data ', max=len(testing_data),
            suffix='%(percent)d%%')
    processed2, skipped_not_good2, skipped_not_found2 = \
    create_record(test_path, testing_data, classes,
            os.path.dirname(input_path), progress)
    progress.finish()
    print "{} images saved to {}.".format(len(testing_data), test_path)

    # Print some file statistics
    print "{} images successfuly processed".format(processed1+processed2)
    print "{} images skipped for not being marked good".format(
            skipped_not_good1+skipped_not_good2)
    print "{} images skipped".format(skipped)
    print "{} images not found".format(skipped_not_found1+skipped_not_found2)

if __name__ == '__main__':
        _main(args)
