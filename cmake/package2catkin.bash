#!/bin/bash
# Convert the "followme_laser" into a catkin package

# get script path - https://stackoverflow.com/questions/4774054/reliable-way-for-a-bash-script-to-get-the-full-path-to-itself
pushd `dirname $0` > /dev/null
SCRIPTPATH=`pwd`
popd > /dev/null

cd ..
rm -f manifest.xml # clean for rosmake files
ln -s --force $SCRIPTPATH/CMakeLists_catkin.txt  CMakeLists.txt
ln -s --force $SCRIPTPATH/package.xml
ls -al
rospack profile
