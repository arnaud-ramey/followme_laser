/*!
  \file         ros_goal_dynamic_window_tracker.cpp
  \author       Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date         2015/10/14

  ______________________________________________________________________________

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  ______________________________________________________________________________

  This node enables the following and tracking of a moving goal.
  The free space is obtained thanks to the local costmap
  delivered by the move_base componnet.

  Many parameters enable to configure the behaviour of the robot.
  Most notably minimum and maximum speed

\section Parameters
  - \b min_vel_lin
    [double, m.s-1] (default: .1)
    The minimum linear speed.

  - \b max_vel_lin
    [double, rad.s-1] (default: .3)
    The maximum linear speed.

  - \b max_vel_ang
    [double, rad.s-1] (default: .5)
    The maximum absolute angular speed. The minimum angular speed is 0.
    The search domain is then: [-max_vel_ang .. max_vel_ang]

  - \b robot_radius
    [double, meters] (default: .5)
    The robot radius, in meters.

  - \b min_goal_distance
    [double] m (default: .6)
    The minimum distance between the robot center and the goal center
    When this distance is reached, the robot stopped.
    (do not forget that it includes the robot radius!)

  - \b max_goal_angle
    [double] m (default: .2)
    The maximum angle allowed with the goal.
    If the nose of the robot is pointing to a direction making an angle
    higher than this value, it will rotate on place.

\section Subscriptions
 - \b {scan_topic}
      [sensor_msgs::LaserScan]
      The data from the laser.

 - \b "moving_goal"
      [geometry_msgs::PoseStamped]
      The pose of the object being tracked.
      Send an empty geometry_msgs::PoseStamped to stop tracking.

\section Publications
  - \b "cmd_vel"
    [geometry_msgs::Twist]
    The orders sent to the base.
*/
#include <followme_laser/ros_goal_dynamic_window_tracker.h>

int main (int argc, char** argv) {
  ros::init(argc, argv, "ros_goal_dynamic_window_tracker"); //Initialise and create a ROS node
  ROSGoalDynamicWindowTracker GoalDynamicWindowTracker;
  ros::spin();
}
