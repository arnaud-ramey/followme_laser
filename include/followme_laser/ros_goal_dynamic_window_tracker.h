#ifndef ROS_GOAL_DYNAMIC_WINDOW_TRACKER_H
#define ROS_GOAL_DYNAMIC_WINDOW_TRACKER_H

// pkg
#include <followme_laser/goal_dynamic_window_tracker.h>
#include <vision_utils/convert_sensor_data_to_xy.h>
#include <vision_utils/convert_xy_vec_frame.h>
#include <vision_utils/vector2path.h>
// ROS
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>

typedef geometry_msgs::Point Pt2;

////////////////////////////////////////////////////////////////////////////////

class ROSGoalDynamicWindowTracker : public GoalDynamicWindowTracker<Pt2> {
public:
  ROSGoalDynamicWindowTracker() : _nh_private("~") {
    // get params
    // default values of _speed_recomputation_timeout, _time_pred, _time_step
    // are set by set_simulation_parameters()
    // default values of _min_goal_distance, _max_goal_angle
    // are set by set_goal_parameters()
    double min_vel_lin,max_vel_lin, max_vel_ang;
    _nh_private.param("max_vel_ang", max_vel_ang, .5);
    _nh_private.param("max_vel_lin", max_vel_lin, .3);
    _nh_private.param("max_goal_angle", _max_goal_angle, _max_goal_angle);
    _nh_private.param("min_goal_distance", _min_goal_distance, _min_goal_distance);
    _nh_private.param("min_vel_lin", min_vel_lin, .1);
    _nh_private.param("speed_recomputation_timeout",
                      _speed_recomputation_timeout, _speed_recomputation_timeout);
    _nh_private.param("goal_timeout", _goal_timeout, _goal_timeout);
    _nh_private.param("time_pred", _time_pred, _time_pred);
    _nh_private.param("time_step", _time_step, _time_step);
    _nh_private.param("robot_radius", _robot_radius, .5);
    set_limit_speeds(min_vel_lin, max_vel_lin, max_vel_ang);
    // subscribers
    _laser_sub = _nh_public.subscribe<sensor_msgs::LaserScan>
                 ("scan", 1,  &ROSGoalDynamicWindowTracker::scan_cb, this);
    _goal_sub = _nh_public.subscribe<geometry_msgs::PoseStamped>
                ("moving_goal", 1,  &ROSGoalDynamicWindowTracker::goal_cb, this);
    // publishers
    _vel_pub = _nh_public.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    _traj_pub = _nh_private.advertise<nav_msgs::Path>("traj", 1);
  }

  //////////////////////////////////////////////////////////////////////////////

  void scan_cb(const sensor_msgs::LaserScanConstPtr & scan) {
    //ROS_INFO("ROSGoalDynamicWindowTracker::scan_cb()");
    if (!_goal_set)
      return;
    vision_utils::convert_sensor_data_to_xy(*scan, _scan2vec_src_frame);
    double scan_z_dst_frame = 0;
    if (!vision_utils::convert_xy_vec_frame(scan->header,
                                         _scan2vec_src_frame,
                                         _tf_listener,
                                         _dst_frame,
                                         _scan2vec_dst_frame,
                                         scan_z_dst_frame))
      return;

    set_costmap(_scan2vec_src_frame, _robot_radius);
    geometry_msgs::Twist twist;
    if (!recompute_speeds_if_needed(twist.linear.x, twist.angular.z)) {
      printf("recompute_speeds() failed!\n");
      unset_goal();
    }
    else {
      _vel_pub.publish(twist);
    }
    if (_traj_pub.getNumSubscribers()) { // dont publish path if no subscriber
      _path_msg.header = scan->header;
      vision_utils::vector2path(get_best_trajectory(), _path_msg);
      _traj_pub.publish(_path_msg);
    }
  } // end scan_cb();

  //////////////////////////////////////////////////////////////////////////////

  void goal_cb(const geometry_msgs::PoseStamped::ConstPtr & goal) {
    //ROS_INFO("ROSGoalDynamicWindowTracker::goal_cb()");
    if (goal->pose.orientation.x == 0 && goal->pose.orientation.y == 0 &&
        goal->pose.orientation.z == 0 && goal->pose.orientation.w == 0) {
      ROS_WARN("goal_cb(): empty goal, stopping tracking.");
      unset_goal();
      return;
    }
    _dst_frame = goal->header.frame_id;
    set_goal(goal->pose.position);
  } // end goal_cb();

  //////////////////////////////////////////////////////////////////////////////

  virtual void unset_goal() {
    ROS_INFO("ROSGoalDynamicWindowTracker::unset_goal()");
    GoalDynamicWindowTracker::unset_goal();
    geometry_msgs::Twist twist;
    _vel_pub.publish(twist);
    // clear path message
    _path_msg.poses.clear();
    _traj_pub.publish(_path_msg);
  }

  tf::TransformListener _tf_listener;
  ros::Subscriber _laser_sub, _goal_sub;
  ros::Publisher _vel_pub, _traj_pub;
  nav_msgs::Path _path_msg;
  ros::NodeHandle _nh_public, _nh_private;
  std::vector<Pt2> _scan2vec_src_frame, _scan2vec_dst_frame;
  double _robot_radius;
  std::string _dst_frame;
}; // end class ROSGoalDynamicWindowTracker

#endif // ROS_GOAL_DYNAMIC_WINDOW_TRACKER_H
