#!/bin/bash
if [ ! -d vision_utils ] ; then
  echo "Checking out \"vision_utils\"."
  svn co https://github.com/UC3MSocialRobots/vision_utils/trunk/include/vision_utils
elif ping -c  1 github.com >> /dev/null ; then
  echo "Github reachable, updating \"vision_utils\"."
  svn up vision_utils
else
  echo "/!\ Github not reachable, not updating \"vision_utils\"."
fi
