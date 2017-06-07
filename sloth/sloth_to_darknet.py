#!/usr/bin/env python

import os
import argparse

def extract_data(fname):
    pairs = []
    with open(fname, 'r') as f:
        filename = ""
        text = []
        trashingHeader = False
        for line in f:
            if "---" in line and not trashingHeader:
                trashingHeader = True
            elif "---" in line:
                break
            else:
                pass
        for line in f:
            if ">>" in line:
                filename = line.split(" ")[1][1:-2]
            elif "<<" in line:
               pairs.append((filename, text))
               text = []
               filename = ""
            else:
                text.append(line) 
    return pairs

def write_to_file(fname, text):
    print("Writing to {}".format(fname))
    if not os.path.exists(os.path.split(fname)[0]):
        os.makedirs(os.path.split(fname)[0])
    with open(fname, 'w') as f:
        for line in text:
            f.write(line)

def create_training_list(tlist, outname):
    print("Writing training list")
    if not os.path.exists(os.path.split(outname)[0]) and os.path.split(outname)[0] is not "":
        os.makedirs(os.path.split(outname)[0])
    with open(outname, 'w') as f:
        for line in tlist:
            f.write(line)
            f.write("\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert from sloth *.darknet format, to proper darknet used files")
    parser.add_argument("--filename", "-f", type=str, help="A relative path to the *.darknet file", required=True)
    parser.add_argument("--output-dir","-o", type=str, help="the path where the annotation files will be saved", required=True)
    parser.add_argument("--training-name", "-t", type=str, help="The filename of the output training list", default="training-list.txt")

    args = parser.parse_args()

    if "darknet" not in args.filename.split(".")[-1]:
        print("Invalid input filename, must be of file format *.darknet")
        exit()

    data = extract_data(args.filename)

    args.output_dir = os.path.normpath(args.output_dir)
    training_list = []
    for f, text in data:
        filename = ".".join(f.split(".")[0:-1]) + ".txt"
        training_list.append(os.path.normpath(os.path.join(os.path.abspath(os.path.split(args.filename)[0]), f)))
        write_to_file(os.path.normpath(args.output_dir + "/" + filename), text)

    print(training_list)
    create_training_list(training_list, args.training_name)
