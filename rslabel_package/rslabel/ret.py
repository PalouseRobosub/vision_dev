#!/usr/bin/python
"""
@author Ryan Summers
@date 2-27-2018

@brief Returns labeled or validated datasets to the robosub server.
"""

from __future__ import print_function

import datetime
import json
import os
import pysftp
import shutil
import sys
import tempfile
import glob
import tarfile

try:
    input = raw_input
except:
    pass


def app(args):
    """ Main entry point for returning data to the server.

    Arguments:
        args: Passed in from argparse. Must have a named member `annotations`
              that specifies the path to the annotation JSON.
    """

    # Grab the username for data ownership and the SFTP password.
    data_owner = os.environ.get('ROBOSUB_WHO_AM_I')
    if data_owner is None:
        print('To suppress this prompt, please set the environment variable ',
                end='')
        print('ROBOSUB_WHO_AM_I to your name.')
        data_owner = input('Please enter your name (First Last): ')

    password = os.environ.get('ROBOSUB_SFTP_PASSWORD')
    if password is None:
        print('To suppress this prompt, please set the ROBOSUB_SFTP_PASSWORD ',
                end='')
        print('environment variable.')
        password = input('Please enter the Robosub SFTP password: ')

    # Read the annotations to check if validation or labeling have been
    # completed.
    with open(args.annotations, 'r') as f:
        json_contents = json.load(f)
    tar_name = os.path.splitext(os.path.basename(args.annotations))[0] + '.tar'

    # Open an SFTP connection with the robosub server.
    with pysftp.Connection('robosub.eecs.wsu.edu',
                username='sftp_user',
                password=password,
                default_path='/data/vision/labeling') as sftp:

        # Check to see if there is an in-progress labeling or validation
        # session.
        with sftp.cd('in_progress/labeling/'):
            labeling_tars = [x for x in sftp.listdir() if x.endswith('.tar')]

        with sftp.cd('in_progress/validation/'):
            validation_tars = [x for x in sftp.listdir() if x.endswith('.tar')]

        with sftp.cd('in_progress/clarification/'):
            clarification_tars = [x for x in sftp.listdir() if x.endswith('.tar')]

        in_validation = False
        in_clarification = False
        delete = False

        # If the dataset is currently being labeled, set up the proper source
        # and destination paths on the server. If labeling, validation or
        # clarification is not fully completed, move from in_progress back
        # into the intermediate step.
        if tar_name in labeling_tars:

            complete = True
            for annotation in json_contents:
                try:
                    complete = not annotation['unlabeled']
                    if not complete:
                        break
                except:
                    pass

            src_dir = 'in_progress/labeling/'
            dest_dir = 'unvalidated/' if complete else 'new/'
        elif tar_name in validation_tars:

            in_validation = True
            complete = True
            for annotation in json_contents:
                try:
                    status = annotation['status']
                except:
                    complete = False
                    break
            src_dir = 'in_progress/validation/'
            dest_dir = 'done/' if complete else 'unvalidated/'
        elif tar_name in clarification_tars:

            in_clarification = True
            complete = True
            dir_name = os.path.dirname(args.annotations)

            # Looping over to see if there are bad images, and see if the tar is complete
            for annotation in json_contents:
                try:
                    ann = annotation['status']
                    if ann == 'Bad':
                        delete = True
                except:
                    complete = False
                    pass

            if not delete:
                src_dir = 'in_progress/clarification/'
                dest_dir = 'new/' if complete else 'clarification/'
            else:
                src_dir = 'in_progress/clarification/'
                dest_dir = 'in_progress/clarification/temp/' if complete else 'clarification/'
        else:
            print('The supplied JSON name does not match any in-progress ',
                    end='')
            print('validation, labeling sessions or clarification.')
            print('Current labeling sessions: {}'.format(labeling_tars))
            print('Current validation sessions: {}'.format(validation_tars))
            print('Current clarification sessions: {}'.format(clarification_tars))
            sys.exit(-1)

        if not complete:
            if in_validation:
                print('Validation was not completed. Returning progress to ',
                        end='')
                print('unvalidated datasets.')
            elif in_clarification:
                print('clarification was not completed. Returning progress to new ',
                        end='')
                print('datasets.')
            else:
                print('Labeling was not completed. Returning progress to new ',
                        end='')
                print('datasets.')

        with sftp.cd(src_dir):
            tars = [x for x in sftp.listdir() if x.endswith('.tar')]

        if tar_name not in tars:
            print('The provided annotations dataset was not found in ',
                    end='')
            print('robosub.eecs.wsu.edu:/data/vision/labeling/' + src_dir)
            sys.exit(-1)

            # If the annotations already exist on the server, pull them down to
            # figure out what new labels were added for stat tracking.
        if tar_name not in clarification_tars:
            previous_annotations = []
            with sftp.cd(src_dir):
                if os.path.basename(args.annotations) in sftp.listdir():
                    with tempfile.NamedTemporaryFile() as tempf:
                        sftp.get(os.path.basename(args.annotations), tempf.name)
                        with open(tempf.name, 'r') as f:
                            previous_annotations = json.load(f)

            with open(args.annotations, 'r') as f:
                new_annotations = json.load(f)

            if len(previous_annotations) and len(new_annotations) != len(previous_annotations):
                print('Provided annotation file and server annotation file differ.')
                sys.exit(-1)

            log = {'labels_added': 0,
                   'images_labeled': 0,
                   'labels_validated': 0,
                   'images_validated': 0}

            if len(previous_annotations) == 0:
                for annotation in new_annotations:
                    if in_validation:
                        log['images_validated'] += 1
                        log['labels_validated'] += len(annotation['annotations'])
                    else:
                        labels_added = len(annotation['annotations'])

                        log['labels_added'] += labels_added
                        if labels_added > 0:
                            log['images_labeled'] += 1
            else:
                for old, new in zip(previous_annotations, new_annotations):
                    if in_validation:
                        log['images_validated'] += 1
                        try:
                            status = new['status']
                            try:
                                old_status = old['status']
                                if old_status != status:
                                    log['labels_validated'] += len(new['annotations'])
                                    log['images_validated'] += 1
                            except:
                                log['labels_validated'] += len(new['annotations'])
                                log['images_validated'] += 1
                        except:
                            pass
                    else:
                        labels_added = len(new['annotations']) - len(old['annotations'])
                        if labels_added > 0:
                            log['labels_added'] += labels_added
                            log['images_labeled'] += 1

        # Upload the JSON to the server. Or tar if the images were bad
        # in clarification proccess
        with sftp.cd(dest_dir):
            sftp.put(args.annotations)

        # Move the tar from in_progress to the proper destination.
        # or delete tar if images were bad in clarification
        sftp.rename('{}/{}'.format(src_dir, tar_name),
                    '{}/{}'.format(dest_dir, tar_name))

        if delete and complete:
            with sftp.cd(dest_dir):
                sftp.execute("python /data/vision/labeling/in_progress/clarification/temp/delete.py {} --remote".format(tar_name))
        # Remove the ownership and annotation files.
        with sftp.cd(src_dir):
            if os.path.basename(args.annotations) in sftp.listdir():
                sftp.remove(os.path.basename(args.annotations))
            sftp.remove('{}.owner'.format(tar_name))

        # Delete the folder containing the JSON if the user would like. Ensure
        # the dirname is a valid path.
        directory = os.path.dirname(args.annotations)
        if directory != '':
            if args.auto_delete:
                delete = True
            else:
                user_input = input('Delete folder `{}/`? (y/n): '.format(
                            directory))
                delete = user_input == 'y' or user_input == 'Y'

            if delete:
                print('Deleting {}/'.format(directory))
                shutil.rmtree(directory)

            # Finally, upload the stats to the history folder on the
            # server for stats tracking.
        if tar_name not in clarification_tars:
            stats = [{'owner': data_owner,
                      'stats': log,
                      'date-time': datetime.datetime.now().isoformat()}]

            # Only upload stats if someone has modified the annotations.
            if log['images_labeled'] != 0 or log['images_validated'] != 0:
                with tempfile.NamedTemporaryFile(prefix=data_owner) as tempf:
                    with open(tempf.name, 'w') as f:
                        json.dump(stats, f)

                    with sftp.cd('history'):
                        log_files = [x for x in sftp.listdir() if x.startswith(data_owner.replace(' ', '_'))]
                        file_numbers = [int(os.path.splitext(x)[0].split('-')[-1]) for x in log_files]
                        if len(file_numbers) == 0:
                            f_number = 0
                        else:
                            f_number = max(file_numbers) + 1

                        dst = os.path.basename(tempf.name)
                        sftp.put(tempf.name)
                        sftp.rename(dst, '{}-{}.log'.format(data_owner.replace(' ', '_'), f_number))

        print('Data has been successfully returned.')
