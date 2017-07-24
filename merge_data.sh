#!/bin/bash
# Given a folder, this script will untar a folder of tar files containing separate datasets and merge all of their jsons together.

# arguments: <merge directory> <name of merged dataset>

# directory we are operating on
src_dir=${1%/}
output_name=${2%/}

# subfolder for merging
merge_dir_name="merge_dir"
merge_dir=${src_dir}/${merge_dir_name}


# check we are given a directory for merging
if [ "${src_dir}" == "" ] ; then
    echo "must supply directory to operate on"
    exit 1
fi

echo "operating on ${src_dir}/"

# check the output name
if [ "${output_name}" == "" ] ; then
    output_name="merged"
fi

echo "output is ${output_name}"

# untar all files in the directory to the merge directory
echo "untaring all files"
mkdir ${merge_dir}

for f in ${src_dir}/*.tar; do
    echo "untaring: ${f}"
    tar --strip-components=1 -xf ${f} -C ${merge_dir}
done

# copy any lingering .jsons to the merge directory while we are at it 
\cp ${src_dir}/*.json ${merge_dir}


# create dummy json for all files to merge into
echo "[]" > ${merge_dir}/${output_name}.json

# merge all the jsons together
echo "merging jsons"
for f in ${merge_dir}/*.json; do
    if [ "${f}" != "${merge_dir}/${output_name}.json" ] ; then
        echo "merging ${f} into ${output_name}.json"
        sloth mergefiles -c sloth/robosub_config.py ${f} ${merge_dir}/${output_name}.json ${merge_dir}/${output_name}.json
        rm ${f}
    fi
done

# tar the whole thing
echo "creating tar"
tar -cf ${src_dir}/${output_name}.tar --directory ${merge_dir} .

# remove the created directory
rm -rf ${merge_dir}


echo "done!"
