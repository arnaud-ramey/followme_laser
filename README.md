# followme_laser

[![Build Status](https://travis-ci.org/arnaud-ramey/followme_laser.svg)](https://travis-ci.org/arnaud-ramey/followme_laser)

followme_laser is used to follow a moving goal.

How to install
==============

Dependencies from sources
-------------------------

Dependencies handling is based on the [wstool](http://wiki.ros.org/wstool) tool.
Run the following instructions:

```bash
$ sudo apt-get install python-wstool
$ roscd ; cd src
$ wstool init
$ wstool merge `rospack find followme_laser`/dependencies.rosinstall
$ wstool update
```

Dependencies included in the Ubuntu packages
--------------------------------------------

Please run the ```rosdep``` utility:

```bash
$ sudo apt-get install python-rosdep
$ sudo rosdep init
$ rosdep install followme_laser --ignore-src
```

Build package with Catkin
-------------------------

```bash
$ catkin_make --only-pkg-with-deps followme_laser
```
