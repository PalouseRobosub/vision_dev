#!/bin/bash
# given a folder, this script will split it into subfolders with 100 images
# each inside, and also prepare sloth annotation files inside each subfolder.
# This script is designed for splitting a large folder of images so that
# multiple people can annotate it.


# directory we are operating on
src_dir=$1

# how many files per folder
fpf=100

if [ "$src_dir" == "" ] ; then
    echo "must supply directory to operate on"
    exit 1
fi

echo "operating on ${src_dir}"

# split images into folders with 100 images each
i=0;
for f in ${src_dir}/*; do
    d=${src_dir}_sub_$(printf %03d $((i/${fpf})))
    mkdir -p $d
    cp -v "$f" $d
    let i++
done

#create sloth annotation files
for d in ${src_dir}_sub_* ; do
    sloth appendfiles -u $d/lables.json $d/*
done
