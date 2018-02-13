#!/bin/bash
# This program will upload json to the server, and move tar to "done" folder

name=${1}

if [ "$1" == "" ]; then
    echo -n "Please name of .json file: "
    read name
    echo
fi

# prompt for login information
echo -n "user: "
read USER

echo -n "Uploading .json to the server"
echo
rsync -Ph $name ${USER}@robosub.eecs.wsu.edu:/data/vision/labeling/labeling_todo/current/done/
echo -n "Moving tar to done folder"
ssh ${USER}@robosub.eecs.wsu.edu 'mv /data/vision/labeling/labeling_todo/current/working_on/'${name%.json}.tar ' /data/vision/labeling/labeling_todo/current/done/'
echo -n "Done!"
echo
