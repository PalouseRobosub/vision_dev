



BASE_URL='http://robosub-vm.eecs.wsu.edu/data/2018/vision/labeling/labeling_todo'
BASE_TAR="NavChannel_PathMarker/images_sub_0"
SUB_PATH=$BASE_TAR

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
for i in ${@:1} ; do
    wget --user=${USER} --password=${PASSWORD} -c $BASE_URL/${SUB_PATH}${i}.tar
    #wget -c $BASE_URL/${SUB_PATH}${i}.tar
done

echo "extracting tarballs"
for i in ./*.tar ; do
    tar -xf $i
done
