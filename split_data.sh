#!/bin/bash
# Given a folder, this script will split it into tar files with 100 images
# each inside, and also prepare sloth annotation files inside each one.
# This script is designed for splitting a large folder of images so that
# multiple people can later download and annotate them.


# directory we are operating on
src_dir=${1%/}

# how many files per folder
fpf=100

if [ "$src_dir" == "" ] ; then
    echo "must supply directory to operate on"
    exit 1
fi

echo "operating on ${src_dir}/"

# split images into folders with 100 images each
i=0;
echo "splitting into folders"
for f in ${src_dir}/*; do
    d=${src_dir}_sub_$(printf %03d $((i/${fpf})))
    if [ ! -d "$d" ] ; then
        echo "  $d"
        mkdir $d
    fi
    cp "$f" $d
    let i++
done

#create sloth annotation files
echo "creating sloth annotation files"
for d in ${src_dir}_sub_* ; do
    echo "  ${d}_labels.json"
    sloth appendfiles -u ${d}/${d}_labels.json ${d}/*
done

#create tarballs
echo "creating tarballs"
for d in ${src_dir}_sub_* ; do
    echo "  ${d}.tar"
    tar -cf ${d}.tar ${d}
done

#remove temporary subfolders
echo "cleaning up"
for d in ${src_dir}_sub_*/ ; do
    rm -rf ${d}
done

echo "done!"
