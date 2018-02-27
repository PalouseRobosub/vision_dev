import argparse
import logging
import cv2
import json
import operator
import sys
import os
import photoshow
import logging

def annotation_class_to_color(annotation_name):
    if annotation_name == 'nav_channel_post':
        return (255, 81, 255)
    elif annotation_name == 'nav_channel_bar':
        return (204, 0, 102)
    elif annotation_name == 'red_buoy':
        return (0, 0, 255)
    elif annotation_name == 'green_buoy':
        return (0, 255, 0)
    elif annotation_name == 'yellow_buoy':
        return (0, 255, 255)
    elif annotation_name == 'path_marker':
        return (102, 0, 204)
    elif annotation_name == 'start_gate_post':
        return (0, 102, 255)
    else:
        return (255, 255, 255)


class PhotoShow:
    def __init__(self, img_name):
        self.img = cv2.imread(img_name)
        self.file_name = img_name
        self.draw_annotations()


    def draw_annotations(self, annotations, alpha=0.2, scale=0.7):
        self.annotations = annotations
        for annotation in self.annotations:
            if annotation['type'] != 'rect':
                logging.warning('Encountered non-rectangular annotation: {} for file {}', annotation, self.file_name)
            else:
                color = annotation_class_to_color(annotation['class'])
                origin = (int(annotation['x']), int(annotation['y']))
                end = tuple(map(operator.add, origin, (int(annotation['width']), int(annotation['height']))))

                overlay = self.img.copy()
                cv2.rectangle(overlay, origin, end, color, -1)
                self.img = cv2.addWeighted(self.img, 1 - alpha, overlay, alpha, 0)

                cv2.putText(self.img, annotation['class'], origin, cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

        self.img_small = cv2.resize(self.img, (0,0), fx=scale, fy=scale)


    def display(self):
        cv2.namedWindow('Image Annotated')
        cv2.imshow('Image Annotated', self.img_small)
        code = cv2.waitKey()
        return chr(code)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Utility for validating labels'
            ' on test data sets. Mark an image label as invalid with the \'b\''
            ' key, go back an image with \'a\', and mark images as valid with any'
            ' other key.')
    parser.add_argument('annotation', help='The annotations file to use for validation.', type=str)
    parser.add_argument('--output', help='The location to store the updated annotation file', type=str)
    parser.add_argument('--label', help='Only show images with the specified label.', type=str)
    parser.add_argument('--count', help='Only count images with \'Good\' as the status and with the specified label.', action='store_true')
    parser.add_argument('--noshow', action='store_true', help='Do not show any images.')
    parser.add_argument('--beginning', action='store_true', help='Start at the beginning of the images.')

    args = parser.parse_args()

    """
    Load the annotation JSON into memory.
    """
    with open(args.annotation, 'r') as annotation_file:
        annotations = json.load(annotation_file)

    """
    Determine the proper starting index.
    """
    i = 0
    if args.beginning is False and args.count is False:
        for j in range(0, len(annotations)):
            try:
                if os.path.isfile(annotations[j]['filename']):
                    status = annotations[j]['status']
            except KeyError:
                i = j
                break

    """
    Loop through every image specified in the annotation file.
    """
    count = 0
    while i < len(annotations):
        annotation = annotations[i]

        if annotation['class'] != 'image':
            logging.warning('Non image annotation found: {}'.format(annotation))
            i = i + 1
            continue

        """
        If the label filter is active, ensure that the current image
        contains the label. If it does not contain the label, skip it.
        """
        if args.label is not None:
            show = False
            for label in annotation['annotations']:
                if label['class'] == args.label:
                    show = True
                    break

            if show is False:
                i = i + 1
                continue

        """
        Get input from the user about the status of the image if it is being
        displayed.
        """
        if args.noshow is False:
            try:
                current_img = photoshow.PhotoShow(annotation['filename'])
                current_img.draw_annotations(annotation['annotations'])
            except:
                logging.warning('Failed to show annotation for {}.'.format(annotation['filename']))
                i = i + 1
                continue

            status = current_img.display()
            if status == 'b':
                print '{}'.format(annotation['filename'])
                annotations[i]['status'] = 'Bad'
            elif status == 'a':
                i -= 2
            elif status == 'e':
                break
            elif status != 's':
                annotations[i]['status'] = 'Good'

        """
        Count the image if it is labelled correctly and displayable.
        """
        if args.count is True:
            try:
                if annotation['status'] == 'Good':
                    count = count + 1
            except:
                count = count


        """
        Increment the image index.
        """
        i = i + 1

    """
    Display information about number of labels found matching the required
    status.
    """
    if args.count is True:
        label = '[Any]'
        if args.label is not None:
            label = args.label
        print 'Found {} images with correct label {}.'.format(count, label)


    """
    Save the updated JSON to an optional new annotation file.
    """
    if args.output is not None:
        output_name = args.output
    else:
        output_name = args.annotation

    with open(output_name, 'w') as output:
        json.dump(annotations, output, indent=2)

