#!/bin/bash

#TEST_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

SOURCE="${BASH_SOURCE[0]}"

while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
    DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
    SOURCE="$(readlink "$SOURCE")"
    [[ $SOURCE != /* ]] && SOURCE="$TEST_DIR/$SOURCE" # if SOURCE was a relative symlink, resolve it
done

DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"

echo "$DIR"

if [ -z "$DIR" ] ; then
    echo "Cannot access filepath: \"$DIR\""
    exit 1
fi

sloth $1 -c "$DIR"/robosub_config.py --pythonpath="$DIR" "${@:2}"
