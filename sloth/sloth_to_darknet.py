#!/usr/bin/env python

import os
import argparse
import random
import shutil
#import logging as log

def extract_data(fname):
    pairs = []
    labels = {}
    with open(fname, 'r') as f:
        filename = ""
        text = []
        readingHeader = False
        for line in f:
            if "---" in line and not readingHeader:
                readingHeader = True
            elif "---" in line:
                break
            else:
                tmp = line.split(':')
                labels[int(tmp[1])] = tmp[0]
        for line in f:
            if ">>" in line:
                filename = line.split(" ")[1][1:-2]
            elif "<<" in line:
               pairs.append((filename, text))
               text = []
               filename = ""
            else:
                text.append(line) 
    return pairs, labels

def write_to_file(fname, text):
    #log.debug("Writing to {}".format(fname))
    if not os.path.exists(os.path.split(fname)[0]):
        os.makedirs(os.path.split(fname)[0])
    with open(fname, 'w') as f:
        for line in text:
            f.write(line)

def copy_image(src, dst):
    #log.debug("Copying {} to {}".format(src, dst))
    if not os.path.exists(dst):
        os.makedirs(dst)
    shutil.copy(src, dst)

def create_training_list(tlist, outname):
    #log.debug("Writing training list")
    if not os.path.exists(os.path.split(outname)[0]) and os.path.split(outname)[0] is not "":
        os.makedirs(os.path.split(outname)[0])
    with open(outname, 'w') as f:
        for line in tlist:
            f.write(line)
            f.write("\n")

def create_names(names, filename):
    #log.debug("Writing names list")
    if not os.path.exists(os.path.split(filename)[0]) and os.path.split(filename)[0] is not "":
        os.makedirs(os.path.split(filename)[0])

    i = 0
    with open(filename, 'w') as f:
        while len(names) > 0:
            if i not in names.keys():
                continue
            f.write(names[i] + "\n")
            del names[i]
            i = i + 1

def create_data(num_names, training_filename, validation_filename, names_filename, backup_filename, data_filename, outputdir):
    #log.debug("Writing data file")
    if not os.path.exists(os.path.split(data_filename)[0]) and os.path.split(data_filename)[0] is not "":
        os.makedirs(os.path.split(data_filename)[0])
    
    with open(data_filename, 'w') as f:
        f.write("classes = " + str(num_names) + "\n")
        f.write("train = " + os.path.relpath(training_filename, outputdir) + "\n")
        f.write("valid = " + os.path.relpath(validation_filename, outputdir) + "\n")
        f.write("names = " + os.path.relpath(names_filename, outputdir) + "\n")
        f.write("backup = " + backup_filename + "\n")

def restricted_float(x):
    x = float(x)
    if x < 0.0 or x >= 1.0:
        raise argparse.ArgumentTypeError("%r not in range [0.0, 1.0)"%(x,))
    
    return x

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert from sloth *.darknet format, to proper darknet used files")
    try:
        parser.add_argument("--filename", "-f", type=str, help="A relative path to the *.darknet file", required=True)
        parser.add_argument("--output-dir","-o", type=str, help="the path where the annotation files will be saved", required=True)
        parser.add_argument("--percent-validation", "-v", type=restricted_float, help="The percentage of data as a float (0.0 - 1.0) to use as validation data. (default: 0.1)", required=False, default=0.1)
        #parser.add_argument("--training-name", "-t", type=str, help="The filename of the output training list", default="training-list.txt")
        parser.add_argument("--backup-file", "-b", type=str, help="The filename to use as a backup. (Only writes to *.data file)", required=False, default="backup.bak")
    except:
        parser.print_help()
        exit(1)

    #log.basicConfig(filename='out.log', filemode='w', level=logging.DEBUG, format='%(asctime)s %(message)s')

    args = parser.parse_args()

    if "darknet" not in args.filename.split(".")[-1]:
        print("Invalid input filename, must be of file format *.darknet")
        exit()

    data, names = extract_data(args.filename)
    percentValidation = float(args.percent_validation)
    args.output_dir = os.path.normpath(args.output_dir)
    
    training_filename = os.path.join(args.output_dir, "train.txt")
    validation_filename = os.path.join(args.output_dir, "validation.txt")
    names_filename = os.path.join(args.output_dir, os.path.split(args.output_dir)[1] + ".names")
    data_filename = os.path.join(args.output_dir, os.path.split(args.output_dir)[1] + ".data")
    backup_filename = args.backup_file
    num_names = len(names)

    training_list = []
    validation_list = []

    for f, text in data:
        filename = ".".join(f.split(".")[0:-1]) + ".txt"
        pathname = os.path.join(os.path.split(args.output_dir)[1], "images/" + os.path.split(f)[1])
        if random.random() < percentValidation:
            validation_list.append(pathname)
        else:
            training_list.append(pathname)
        write_to_file(os.path.normpath(args.output_dir + "/labels/" + os.path.split(filename)[1]), text)
        image_location = os.path.normpath(os.path.abspath(os.path.normpath(os.path.join(os.path.split(args.filename)[0], f))))
        copy_image(image_location, os.path.abspath(os.path.join(args.output_dir, "images/")))

    create_training_list(training_list, training_filename)
    create_training_list(validation_list, validation_filename)
    create_names(names, names_filename)

    create_data(num_names, training_filename, validation_filename, names_filename, backup_filename, data_filename, args.output_dir)
