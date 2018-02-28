#!/bin/bash
# This program will check if there are any tars on the server and fetch one.
# for use run just ./get_bag.sh

# Help flag
if [ "$1" == "-h" ]; then
    echo -n "run get_bag.sh to get bags"
    exit
fi

# prompt for login information
echo -n "user: "
read USER
echo -n "password: "
read -s PASS

# fetch list of tars
tar=$(sshpass -p ${PASS} ssh -o StrictHostKeyChecking=no ${USER}@robosub.eecs.wsu.edu 'ls /data/vision/labeling/new/ | grep .tar')
tarArray=(${tar})

# If there are no tars in the folder, exit.
if [ "$tar" == "" ]; then
    echo -n "There are no tars available for download"
    exit
fi

# notify of all tars available and notify which will be downloaded
echo -n 'Available tars for download: ' ${tar}
echo
echo -n "Grabbing: " ${tarArray[0]}
echo

# download tar
sshpass rsync -Ph ${USER}@robosub.eecs.wsu.edu:/data/vision/labeling/new/${tarArray[0]} "."
echo -n "Moving tar to working_on folder"
echo

# move downloaded tar to working_on folder
sshpass -p ${PASS} ssh -o StrictHostKeyChecking=no ${USER}@robosub.eecs.wsu.edu 'mv /data/vision/labeling/new/'${tarArray[0]} ' /data/vision/labeling/in_progress/'
echo -n "done!"
echo
