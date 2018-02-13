#!/bin/bash
# This program will check if there are any tars on the server and fetch one.

# prompt for login information
echo -n "user: "
read USER

# fetch list of tars
tar=$(ssh ${USER}@robosub.eecs.wsu.edu 'ls /data/vision/labeling/labeling_todo/current | grep .tar')
tarArray=(${tar})
# If there are no tars in the folder, exit.
if [ "$tar" == "" ]; then
    echo -n "There are no tars available for download"
    exit
fi

echo -n 'Available tars for download: ' ${tar}
echo
echo -n "Grabbing: " ${tarArray[0]}
echo
rsync -Ph ${USER}@robosub.eecs.wsu.edu:/data/vision/labeling/labeling_todo/current/${tarArray[0]} "."
echo -n "Moving tar to working_on folder"
echo
ssh ${USER}@robosub.eecs.wsu.edu 'mv /data/vision/labeling/labeling_todo/current/'${tarArray[0]} ' /data/vision/labeling/labeling_todo/current/working_on/'
echo -n "done!"
echo
