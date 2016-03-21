#!/bin/bash
# Convert the "followme_laser" into a rosmake package

# get script path - https://stackoverflow.com/questions/4774054/reliable-way-for-a-bash-script-to-get-the-full-path-to-itself
pushd `dirname $0` > /dev/null
SCRIPTPATH=`pwd`
popd > /dev/null

cd ..
rm -f package.xml # clean for catkin files
ln -s --force $SCRIPTPATH/CMakeLists_rosmake.txt  CMakeLists.txt
ln -s --force $SCRIPTPATH/Makefile
ln -s --force $SCRIPTPATH/manifest.xml
ls -al
rospack profile
