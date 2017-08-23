/*!
  \file        test_goal_dynamic_window_tracker.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2017/06/13

________________________________________________________________________________

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
________________________________________________________________________________

\todo Description of the file
 */
// Bring in gtest
#include <gtest/gtest.h>
#include <followme_laser/ros_goal_dynamic_window_tracker.h>

#define ASSERT_TRUE_TIMEOUT(cond, timeout) { vision_utils::Timer timer; while (timer.getTimeSeconds() < timeout && !(cond)) usleep(50 * 1000); } ASSERT_TRUE(cond)
typedef ROSGoalDynamicWindowTracker DWT;

TEST(TestSuite, empty) {
  ROS_INFO("test1");
  DWT t;
  ASSERT_TRUE(t.get_current_action() == DWT::ACTION_RECOMPUTE_SPEED);
  ROS_INFO("test2");
}

////////////////////////////////////////////////////////////////////////////////


void scan_factory(sensor_msgs::LaserScan & scan,
                  const unsigned int nvalues = 360) {
  scan.header.frame_id = "laser_frame";
  scan.header.stamp = ros::Time::now();
  scan.angle_min = -M_PI_2;
  scan.angle_max = +M_PI_2;
  scan.angle_increment = M_PI / nvalues; // angular resolution = 0.5 degree
  scan.range_min = 1;
  scan.range_max = 10;
  scan.ranges.resize(nvalues, 2.);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, nogoal) {
  ROS_INFO("nogoal");
  DWT t;
  ros::NodeHandle nh_public;
  ros::Publisher scan_pub = nh_public.advertise<sensor_msgs::LaserScan>("scan", 1);
  ASSERT_TRUE_TIMEOUT(scan_pub.getNumSubscribers() == 1, 1);
  sensor_msgs::LaserScan scan;
  scan_factory(scan, 360);
  scan_pub.publish(scan); // should not do anything, there is no goal
  ASSERT_TRUE_TIMEOUT(t.get_current_action() == DWT::ACTION_RECOMPUTE_SPEED, 1)
      << "action:" << t.get_current_action();
}


////////////////////////////////////////////////////////////////////////////////

void test_simple(const unsigned int nvalues = 360) {
  ROS_INFO("test_simple(%i)", nvalues);
  DWT t;
  ros::NodeHandle nh_public;
  // make scan
  sensor_msgs::LaserScan scan;
  scan_factory(scan, nvalues);
  // make goal
  geometry_msgs::PoseStamped goal;
  goal.header = scan.header;
  goal.pose.position.x = 1;
  goal.pose.orientation.z = 1;
  // publish goal
  ros::Publisher goal_pub = nh_public.advertise<geometry_msgs::PoseStamped>
              ("moving_goal", 1);
  ASSERT_TRUE_TIMEOUT(goal_pub.getNumSubscribers() == 1, 1);
  goal_pub.publish(goal);
  // publish scan
  ros::Publisher scan_pub = nh_public.advertise<sensor_msgs::LaserScan>("scan", 1);
  ASSERT_TRUE_TIMEOUT(scan_pub.getNumSubscribers() == 1, 1);
  scan_pub.publish(scan);
  // check we computed speeds
  ASSERT_TRUE_TIMEOUT(t.get_current_action() == DWT::ACTION_KEEP_SAME_SPEED, 1)
      << "action:" << t.get_current_action();
}

TEST(TestSuite, simple0)      { test_simple(0); }
TEST(TestSuite, simple1)      { test_simple(1); }
TEST(TestSuite, simple10)     { test_simple(10); }
TEST(TestSuite, simple100)    { test_simple(100); }
TEST(TestSuite, simple1000)   { test_simple(1000); }
TEST(TestSuite, simple10000)  { test_simple(10000); }
//TEST(TestSuite, simple100000) { test_simple(100000); }

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "gtest");
  ros::AsyncSpinner spinner(0);
  spinner.start();;
  return RUN_ALL_TESTS();
}
