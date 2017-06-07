#!/bin/bash

DIR="`dirname \"$0\"`"
DIR="`( cd \"$DIR\" && pwd )`"

echo "$DIR"

if [ -z "$DIR" ] ; then
    echo "Cannot access filepath: \"$DIR\""
    exit 1
fi

sloth $1 -c "$DIR"/robosub_config.py --pythonpath="$DIR" "${@:2}"
