#!/bin/bash
# This script will download tarballs of image data that needs to be tagged.
# You supply a partial url and then multiple numbers
# usage example:
#   To download the following files,
#     http://robosub.eecs.wsu.edu/data/2017/vision_labeling/initial/red_buoy_sub_000.tar
#     http://robosub.eecs.wsu.edu/data/2017/vision_labeling/initial/red_buoy_sub_001.tar
#     http://robosub.eecs.wsu.edu/data/2017/vision_labeling/initial/red_buoy_sub_003.tar
#
#   you can run ./download_data.sh initial/red_buoy_sub_00 0 1 3


BASE_URL='http://robosub-vm.eecs.wsu.edu/data/2018/vision/labeling/labeling_todo'
SUB_PATH=$1

if [ "$SUB_PATH" == "" ] ; then
    echo "error: must supply partial path"
    exit
fi

# prompt for login information
echo -n "user: "
read USER
echo -n "password: "
read -s PASSWORD
echo

# download and extract tarballs
echo "downloading files..."
for i in ${@:2} ; do
    wget --user=${USER} --password=${PASSWORD} -c $BASE_URL/${SUB_PATH}${i}.tar
done

echo "extracting tarballs"
for i in ./*.tar ; do
    tar -xf $i
done


