#!/usr/bin/python
"""Convert Pascal VOC 2007+2012 detection dataset to TFRecords.
Does not preserve full XML annotations.
Combines all VOC 2007 subsets (train, val) with VOC2012 for training.
Uses VOC2012 val for val and VOC2007 test for test.

Code based on:
https://github.com/pjreddie/darknet/blob/master/scripts/voc_label.py
https://github.com/tensorflow/models/blob/master/inception/inception/data/build_image_data.py
"""

import os
import sys
import xml.etree.ElementTree as ElementTree
from datetime import datetime
from progress.bar import IncrementalBar as Bar
import random

import numpy as np
import json

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
decoded_png = tf.image.decode_png(image_placeholder, channels=3)


def process_image(image_path):
    """Decode image at given path."""
    with open(image_path, 'rb') as f:
        image_data = f.read()
    image = decoder_sess.run(decoded_png,
                             feed_dict={image_placeholder: image_data})
    assert len(image.shape) == 3
    height = image.shape[0]
    width = image.shape[1]
    assert image.shape[2] == 3
    return image_data, height, width


def process_anno(anno_path):
    """Process Pascal VOC annotations."""
    with open(anno_path) as f:
        xml_tree = ElementTree.parse(f)
    root = xml_tree.getroot()
    size = root.find('size')
    height = float(size.find('height').text)
    width = float(size.find('width').text)
    boxes = []
    for obj in root.iter('object'):
        difficult = obj.find('difficult').text
        label = obj.find('name').text
        if label not in classes or int(
                difficult) == 1:  # exclude difficult or unlisted classes
            continue
        xml_box = obj.find('bndbox')
        bbox = {
            'class': classes.index(label),
            'y_min': float(xml_box.find('ymin').text) / height,
            'x_min': float(xml_box.find('xmin').text) / width,
            'y_max': float(xml_box.find('ymax').text) / height,
            'x_max': float(xml_box.find('xmax').text) / width
        }
        boxes.append(bbox)
    return boxes


def convert_to_example(image_data, boxes, filename, height, width):
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
    box_classes = [b['class'] for b in boxes]
    box_ymin = [b['y_min'] for b in boxes]
    box_xmin = [b['x_min'] for b in boxes]
    box_ymax = [b['y_max'] for b in boxes]
    box_xmax = [b['x_max'] for b in boxes]
    encoded_image = [tf.compat.as_bytes(image_data)]
    base_name = [tf.compat.as_bytes(os.path.basename(filename))]

    example = tf.train.Example(features=tf.train.Features(feature={
        'filename':
        tf.train.Feature(bytes_list=tf.train.BytesList(value=base_name)),
        'height':
        tf.train.Feature(int64_list=tf.train.Int64List(value=[height])),
        'width':
        tf.train.Feature(int64_list=tf.train.Int64List(value=[width])),
        'classes':
        tf.train.Feature(int64_list=tf.train.Int64List(value=box_classes)),
        'y_mins':
        tf.train.Feature(float_list=tf.train.FloatList(value=box_ymin)),
        'x_mins':
        tf.train.Feature(float_list=tf.train.FloatList(value=box_xmin)),
        'y_maxes':
        tf.train.Feature(float_list=tf.train.FloatList(value=box_ymax)),
        'x_maxes':
        tf.train.Feature(float_list=tf.train.FloatList(value=box_xmax)),
        'encoded':
        tf.train.Feature(bytes_list=tf.train.BytesList(value=encoded_image))
    }))
    return example

def process_dataset(name, image_paths, anno_paths, result_path, num_shards):
    """Process selected Pascal VOC dataset to generate TFRecords files.

    Parameters
    ----------
    name : string
        Name of resulting dataset 'train' or 'test'.
    image_paths : list
        List of paths to images to include in dataset.
    anno_paths : list
        List of paths to corresponding image annotations.
    result_path : string
        Path to put resulting TFRecord files.
    num_shards : int
        Number of shards to split TFRecord files into.
    """
    shard_ranges = np.linspace(0, len(image_paths), num_shards + 1).astype(int)
    counter = 0
    for shard in range(num_shards):
        # Generate shard file name
        output_filename = '{}-{:05d}-of-{:05d}'.format(name, shard, num_shards)
        output_file = os.path.join(result_path, output_filename)
        writer = tf.python_io.TFRecordWriter(output_file)

        shard_counter = 0
        files_in_shard = range(shard_ranges[shard], shard_ranges[shard + 1])
        for i in files_in_shard:
            image_file = image_paths[i]
            anno_file = anno_paths[i]

            # processes image + anno
            image_data, height, width = process_image(image_file)
            boxes = process_anno(anno_file)

            # convert to example
            example = convert_to_example(image_data, boxes, image_file, height,
                                         width)

            # write to writer
            writer.write(example.SerializeToString())

            shard_counter += 1
            counter += 1

            if not counter % 1000:
                print('{} : Processed {:d} of {:d} images.'.format(
                    datetime.now(), counter, len(image_paths)))
        writer.close()
        print('{} : Wrote {} images to {}'.format(
            datetime.now(), shard_counter, output_filename))

    print('{} : Wrote {} images to {} shards'.format(datetime.now(), counter,
                                                     num_shards))

def extract_boxes(annotations, img_height, img_width, classes):

    boxes = list()
    for annotation in annotations:
        # tfrecord wants numerical index for class labels
        label_index = classes.index(str(annotation['class']))
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
                'class': label_index,
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

def create_record(outfile, json_data, classes, input_path, progress):
    writer = tf.python_io.TFRecordWriter(outfile)
    #counter = 0
    for item in json_data:
        # Some nice progress updates
        #counter += 1
        progress.next()

        filepath = os.path.join(input_path, str(item['filename']))
        filename = os.path.basename(filepath)
        try:
            image_data, height, width = process_image(filepath)
        except IOError:
            #print "could not find {}, skipping".format(filepath)
            continue
        boxes = extract_boxes(item['annotations'],
                img_height=height,
                img_width=width,
                classes=classes)

        tfexample = convert_to_example(image_data, boxes, filename,
                height, width)

        writer.write(tfexample.SerializeToString())


def _main(args):
    """Locate files for train and test sets and then generate TFRecords."""
    input_path = args.input
    input_path = os.path.expanduser(input_path)
    output_path = args.outdir

    train_path = os.path.join(output_path, 'train.tfrecord')
    test_path = os.path.join(output_path, 'test.tfrecord')

    json_data = json.load(open(args.input, 'r'))

    classes =  find_classes(json_data)
    print "Found the following classes:"
    for x in classes:
        print "  {}".format(x)
    print ""


    # shuffle our data, split into training and testing segments
    random.shuffle(json_data)
    split_index = int(len(json_data)*(1.0 - args.test))
    training_data = json_data[0 : split_index]
    testing_data = json_data[split_index:]

    progress = Bar('Creating training data', max=len(training_data),
            suffix='%(percent)d%%')
    create_record(train_path, training_data, classes, os.path.dirname(input_path), progress)
    progress.finish()
    print "training data saved to {}".format(train_path)

    progress = Bar('Creating testing data ', max=len(testing_data),
            suffix='%(percent)d%%')
    create_record(test_path, testing_data, classes, os.path.dirname(input_path), progress)

    progress.finish()
    print "testing data saved to {}".format(test_path)

if __name__ == '__main__':
        _main(args)
