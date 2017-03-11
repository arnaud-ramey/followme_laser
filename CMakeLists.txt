### 1 - Required CMake Version
cmake_minimum_required(VERSION 2.8.3)
### 2- Package Name
project(followme_laser)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra") # add extra warnings
### 3 - Find other CMake/Catkin packages needed for build
find_package(catkin REQUIRED COMPONENTS  geometry_msgs  roscpp  sensor_msgs  tf  visualization_msgs  vision_utils)
### 4 - Message/Service/Action Generators (add_message_files(), add_service_files(), add_action_files())
add_message_files(
  FILES
  TrackingStatus.msg
)
### 5 - Invoke message/service/action generation (generate_messages())
generate_messages(
  DEPENDENCIES
  std_msgs sensor_msgs geometry_msgs visualization_msgs # Or other packages containing msgs
)
### 6 - Specify package build info export (catkin_package())
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp  geometry_msgs  roscpp  sensor_msgs  tf  visualization_msgs
)
### 7 - Libraries/Executables to build (add_library()/add_executable()/target_link_libraries())
include_directories( include ${catkin_INCLUDE_DIRS})
add_executable(ros_laser_blob_tracker.exe
               src/ros_laser_blob_tracker.cpp
               include/followme_laser/laser_blob_tracker.h)
target_link_libraries(ros_laser_blob_tracker.exe ${catkin_LIBRARIES})
add_dependencies     (ros_laser_blob_tracker.exe
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

add_executable(ros_goal_dynamic_window_tracker.exe
               src/ros_goal_dynamic_window_tracker.cpp
               include/followme_laser/goal_dynamic_window_tracker.h)
target_link_libraries(ros_goal_dynamic_window_tracker.exe ${catkin_LIBRARIES})
add_dependencies     (ros_goal_dynamic_window_tracker.exe
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

### 8 - Tests to build (catkin_add_gtest())
# Nothing in this package
### 9 - Install rules (install())
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
