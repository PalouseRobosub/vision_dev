# This script runs 
# "sloth convert -c stuff/json"
# and then runs 
# "./sloth_to_darknet.py

json=${1}
output=${2}

folder_path_json=$(dirname "$json")
filename_json=$(basename "$json")
config="robosub_config.py"

SOURCE="${BASH_SOURCE[0]}"
DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"

sloth convert -c $config --pythonpath="$DIR" $json $folder_path_json/dataset.darknet

./sloth_to_darknet.py -f $folder_path_json/dataset.darknet -o dataset
tar -cf ${output}.tar dataset
