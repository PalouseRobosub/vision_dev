#!/bin/bash
# This program will upload json to the server, and move tar to "done" folder
# for use run ./upload_json.sh {json_name}

name=${1}

# Help flag
if [ "$1" == "-h" ]; then
    echo -n "run upload_json.sh {json name}"
    exit
fi

# check if name was given
if [ "$1" == "" ]; then
    echo -n "Please name of .json file: "
    read name
    echo
fi

# prompt for login information
echo -n "user: "
read USER
echo -n "password: "
read -s PASS

# rsync file to the server
echo -n "Uploading .json to the server"
echo
sshpass -p ${PASS} rsync -Ph $name ${USER}@robosub.eecs.wsu.edu:/data/vision/labeling/unvalidated
echo -n "Moving tar to done folder"

# move tar with same name to done folder
sshpass -p ${PASS} ssh -o StrictHostKeyChecking=no ${USER}@robosub.eecs.wsu.edu 'mv /data/vision/labeling/in_progress/'${name%.json}.tar ' /data/vision/labeling/unvalidated/'
echo -n "Done!"
echo
