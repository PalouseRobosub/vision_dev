#!/usr/bin/python
"""
@author Ryan Summers
@date 3-14-2018

@brief Provides a UI for marking labels as valid or invalid.
"""

from __future__ import print_function

import cv2
import json
import logging
import operator
import os
import shutil
import sys
import yaml


annotation_config = dict()
"""
A dictionary of colors and alpha values for specific label names.
"""


default_colors = [(255, 0, 0),
                  (0, 255, 0),
                  (0, 0, 255),
                  (0, 255, 255),
                  (255, 0, 255),
                  (0x66, 0, 0x66),
                  (0, 0x33, 0),
                  (0, 0, 0x99),
                  (0, 0, 0),
                  (255, 255, 255),
                  (0xcc, 0, 0x66),
                  (0xff, 0xcc, 0),
                  (0x99, 0, 0x99)]
"""
A list of default colors to use in case they are not specified in the config.
"""


default_color_index = 0
"""
The default color index for automatically assigning label colors.
"""


def show_help():
    """ Displays the keyboard controls for image control. """
    print('Controls:')
    print('\ta         - Go back to the previous image.')
    print('\te/q       - Exit the application.')
    print('\ts         - Skip current image (Does not mark)')
    print('\tb         - Marks an image as \'Bad\'')
    print('\tg/[space] - Marks an image as \'Good\'')


def annotation_class_to_color(annotation_name):
    """
    Checks for alpha and overlay color for a specific annotation. If one was
        not specified in a configuration file, one is automatically assigned.

    Return:
        The color as a tuple and the alpha as a normalized value.
    """
    global default_color_index
    global annotation_config

    try:
        color = annotation_config[annotation_name]['color']
    except KeyError:
        color = default_colors[default_color_index]
        default_color_index += 1
        if default_color_index >= len(default_colors):
            default_color_index = 0

        annotation_config[annotation_name] = dict()
        annotation_config[annotation_name]['color'] = color

    try:
        alpha = annotation_config[annotation_name]['alpha']
    except KeyError:
        annotation_config[annotation_name]['alpha'] = 0.35
        alpha = annotation_config[annotation_name]['alpha']

    return color, alpha


class PhotoShow:
    """ Handles annotating and displaying an image.

    Attributes:
        img: The original OpenCV image. This may be modified by functions.
        annotations: A list of annotations for the image from Sloth.
        scale: The scale of the image that is displayed to the user.
    """

    def __init__(self, img_name, annotations, scale=0.6):
        """ Initializes the image display.

        Args:
            img_name The filename fo the image to use.
            annotations: A list of the annotations for the image from Sloth.
            scale: The scale of the image display from original size.
        """
        self.img = cv2.imread(img_name)
        self.annotations = annotations
        self.scale = scale


    def draw_annotations(self):
        """ Draws the annotations onto the source image. """
        for annotation in self.annotations:
            color, alpha = annotation_class_to_color(annotation['class'])

            origin = (int(annotation['x']), int(annotation['y']))
            end = tuple(map(operator.add, origin,
                        (int(annotation['width']), int(annotation['height']))))

            # Draw an overlay box and add it to the annotated image.
            overlay = self.img.copy()
            cv2.rectangle(overlay, origin, end, color, -1)
            self.img = cv2.addWeighted(self.img, 1 - alpha, overlay, alpha, 0)
            cv2.putText(self.img, annotation['class'], origin, \
                        cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        return


    def display(self):
        """ Displays a scaled version of the source image.

        Returns:
            An ASCII code of the key pressed on the OpenCV window.
        """
        img_small = cv2.resize(self.img, (0,0), fx=self.scale, fy=self.scale)

        cv2.namedWindow('Image Annotated')
        cv2.imshow('Image Annotated', img_small)
        code = cv2.waitKey()

        return chr(code & 0xFF)


def app(args):
    """ Main application entry point.

    Args:
        args: Provided by argparse command line arguments.
    """
    # Load an optionally provided configuration file.
    global annotation_config
    if args.config is not None:
        with open(args.config, 'r') as f:
            annotation_config = yaml.load(f)
        print(annotation_config)

    # Load the annotations from the provided file.
    with open(args.annotations, 'r') as annotation_file:
        annotations = json.load(annotation_file)

    # Backup the annotations to an original annotation file.
    fname = os.path.splitext(args.annotations)[0]
    backup_fname = '{}-original.json'.format(fname)
    shutil.copyfile(args.annotations, backup_fname)

    # Determine the proper starting index.
    i = 0
    if args.beginning is False:
        for j in range(0, len(annotations)):
            try:
                if os.path.isfile(annotations[j]['filename']):
                    status = annotations[j]['status']
            except KeyError:
                i = j
                break

    annotation_path = os.path.dirname(args.annotations)
    if annotation_path == '':
        annotation_path = '.'

    # Loop through every image specified in the annotation file.
    count = 0
    history = []
    show_help()
    while i < len(annotations):
        annotation = annotations[i]

        if annotation['class'] != 'image':
            logging.warning('Non image annotation found: {}'.format(annotation))
            i = i + 1
            continue

        # If the label filter is active, ensure that the current image contains
        # the label. If it does not contain the label, skip it.
        if args.label is not None:
            show = False
            for label in annotation['annotations']:
                if label['class'] == args.label:
                    show = True
                    break

            if show is False:
                i = i + 1
                continue

        # Get input from the user about the status of the image if it is being
        # displayed.
        try:
            current_img = PhotoShow(
                    '{}/{}'.format(annotation_path, annotation['filename']),
                    annotation['annotations'])
        except:
            logging.warning('Failed to read {}'.format(annotation['filename']))
            i = i + 1
            continue

        # Draw the annotations onto the current image and display it.
        current_img.draw_annotations()
        status = current_img.display()

        # Figure out what the user wants to do with the image and respond
        # accordingly.
        if status == 'b':
            print('BAD: {}'.format(annotation['filename']))
            annotations[i]['status'] = 'Bad'

        elif status == 'g' or status == ' ':
            annotations[i]['status'] = 'Good'

        elif status == 'a':
            if len(history) > 0:
                i = history[-1]
                history = history[:-1]
            else:
                print('There is no previous image to display.')
            continue

        elif status == 's':
            print('SKIP: {}'.format(annotation['filename']))
            continue

        elif status == 'e' or status == 'q':
            break

        else:
            show_help()
            continue

        # Increment the image index.
        history.append(i)
        i = i + 1

    print('{} / {} labels marked.'.format(i, len(annotations)))

    # Save the updated JSON to the annotation file.
    with open(args.annotations, 'w') as output:
        json.dump(annotations, output, indent=2)

    print('Original JSON is available at {}'.format(backup_fname))
    print('Validation JSON saved to {}'.format(args.annotations))
