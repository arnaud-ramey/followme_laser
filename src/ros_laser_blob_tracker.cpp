/*!
  \file        followme_laser_ros.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/3

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

\section Parameters
 - \b dst_frame
      [string] (default: "/base_link")
      The frame where the scans will be converted.

 - \b scan_topic
      [string] (default: "scan")
      Where to get the scans.

\section Subscriptions
 - \b {scan_topic}
      [sensor_msgs::LaserScan]
      The data from the laser.

 - \b "~joy"
      [sensor_msgs::Joy]
      To get the pressed buttons, etc.

 - \b "~new_tracking_seed"
      [geometry_msgs::PoseStamped]
      A new seed (origin) for the tracked point.


\section Publications
 - \b "moving_goal"
      [geometry_msgs::PoseStamped]
      The pose of the object being tracked.

 - \b "~tracking_path"
      [sensor_msgs::PointCloud]
      A marker to show the cluster closest to the robot and the tracked cluster.

 - \b "~status"
      [followme_laser::TrackingStatus]
      The status of the tracking.

 */
// ROS
#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
// vision_utils
#include <vision_utils/convert_sensor_data_to_xy.h>
#include <vision_utils/convert_xy_vec_frame.h>
#include <vision_utils/copy2.h>
#include <vision_utils/pose_stamped_to_string.h>
// LaserBlobTracker
#include <followme_laser/laser_blob_tracker.h>

#define PI          M_PI
#define RAD2DEG     57.2957795130823208768  //!< to convert radians to degrees
#define DEG2RAD     0.01745329251994329577  //!< to convert degrees to radians

namespace followme_laser {

typedef geometry_msgs::Point32 Pt2;
class RosLaserBlobTracker : public LaserBlobTracker<Pt2> {
public:

  //! ctor
  RosLaserBlobTracker() :
    _robot_pose(0, 0, 0)
  {
    ros::NodeHandle nh_private("~");

    // get params
    nh_private.param("dst_frame", _dst_frame, std::string("/base_link"));
    double object_max_radius, object_max_distance_btwn_points,
        object_max_searching_radius, object_timeout, distance_to_object;
    nh_private.param("object_max_radius", object_max_radius, .5);
    nh_private.param("object_max_distance_btwn_points", object_max_distance_btwn_points, .2);
    nh_private.param("object_max_searching_radius", object_max_searching_radius, .35);
    nh_private.param("object_timeout", object_timeout, 2.);
    nh_private.param("distance_to_object", distance_to_object, .05);
    set_object_max_radius(object_max_radius);
    set_object_max_distance_btwn_points(object_max_distance_btwn_points);
    set_object_max_searching_radius(object_max_searching_radius);
    set_object_timeout(object_timeout);
    set_distance_to_object(distance_to_object);

    // get topic name
    _laser_sub = _nh_public.subscribe("scan", 1, &RosLaserBlobTracker::scan_cb, this);

    _goal_pub = _nh_public.advertise<geometry_msgs::PoseStamped>
        ("moving_goal", 1);
    // joy subscriber
    _joy_sub = _nh_public.subscribe<sensor_msgs::Joy>
        ("joy", 1,  &RosLaserBlobTracker::joy_cb, this);
    // tracking seed subscriber
    _tracking_seed_sub = nh_private.subscribe
        ("tracking_seed", 1, &RosLaserBlobTracker::tracking_seed_cb, this);
    // marker publisher
    _closest_cloud_pub = nh_private.advertise<sensor_msgs::PointCloud>
        ("closest_object", 1);
    _tracked_cloud_pub = nh_private.advertise<sensor_msgs::PointCloud>
        ("tracked_object", 1);
    // status publisher
    _status_pub= nh_private.advertise<followme_laser::TrackingStatus>
        ("status", 1);

    ROS_WARN("RosLaserBlobTracker: scan_topic:'%s', _goal_topic:'%s', "
             "_path topic:'%s' (status on '%s')",
             _laser_sub.getTopic().c_str(), _goal_pub.getTopic().c_str(),
             _closest_cloud_pub.getTopic().c_str(),
             _status_pub.getTopic().c_str());
  } // end ctor

  //////////////////////////////////////////////////////////////////////////////

  virtual inline void start_tracking_closest_object() {
    stop_tracking_object();
    LaserBlobTracker<Pt2>::start_tracking_closest_object();
  }

  //////////////////////////////////////////////////////////////////////////////

  virtual inline void stop_tracking_object() {
    LaserBlobTracker<Pt2>::stop_tracking_object();
    // publish an empty pose to clean the pose in rviz
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = _dst_frame;
    _goal_pub.publish(msg);
  }

protected:
  //////////////////////////////////////////////////////////////////////////////

  void scan_cb(const sensor_msgs::LaserScanConstPtr & scan_msg) {
    vision_utils::Timer timer;
    // convert to (x,y) in msg frame
    vision_utils::convert_sensor_data_to_xy(*scan_msg, _pts_src_frame);
    double scan_z_dst_frame = 0;
    if (!vision_utils::convert_xy_vec_frame(scan_msg->header,
                                            _pts_src_frame,
                                            _tf_listener,
                                            _dst_frame,
                                            _pts_dst_frame,
                                            scan_z_dst_frame))
      return;

    // convert from msg frame to dst frame
    std::string src_frame = scan_msg->header.frame_id;
    bool need_tf_conv = (src_frame != _dst_frame);
    if (need_tf_conv) {
      bool transform_ok =
          _tf_listener.waitForTransform(_dst_frame,
                                        src_frame,
                                        scan_msg->header.stamp,
                                        ros::Duration(1));
      if (!transform_ok) {
        ROS_WARN("Impossible to find tf '%s' -> %s'. Returning.",
                 src_frame.c_str(), _dst_frame.c_str());
        return;
      }
      ROS_DEBUG_THROTTLE(1, "Success for finding tf '%s' -> %s'.",
                         src_frame.c_str(), _dst_frame.c_str());
    }
    else
      ROS_DEBUG_ONCE("Src and dest frame equal '%s'", _dst_frame.c_str());

    geometry_msgs::PoseStamped robot_pose_in, robot_pose_out; // all filed with 0
    // convert the pose of the robot
    try {
      robot_pose_in.header = scan_msg->header;
      robot_pose_in.pose.orientation = tf::createQuaternionMsgFromYaw(0);
      if (src_frame != _dst_frame)
        _tf_listener.transformPose(_dst_frame,  ros::Time(0),
                                   robot_pose_in, _dst_frame, robot_pose_out);
      else
        robot_pose_out = robot_pose_in;
      vision_utils::copy2(robot_pose_out.pose.position, _robot_pose.position);
      _robot_pose.yaw = tf::getYaw(robot_pose_out.pose.orientation);
      ROS_DEBUG_THROTTLE(1, "The robot is in (%g, %g), yaw:%g degrees in frame %s.",
                         _robot_pose.position.x, _robot_pose.position.y,
                         _robot_pose.yaw * RAD2DEG,
                         _dst_frame.c_str());
    } catch (tf::ExtrapolationException e) {
      ROS_WARN("transform error:'%s'", e.what());
      return;
    } // end try / catch

    /*
     * real laser tracking
     */
    ROS_DEBUG_THROTTLE(1, "Now set_laser_data(pts:%li)", _pts_dst_frame.size());
    set_laser_data(_pts_dst_frame, _robot_pose);

    /*
     * set goal and publish message
     */
    if (_current_status == TrackingStatus::TARGET_TRACKING_OK) {
      SimplePose2D goal(0, 0, 0);
      goal.position = _safe_goal_to_tracked_obj;
      //goal.yaw = _robot_pose.yaw;
      // set the yaw in the direction of the goal
      goal.yaw = PI + atan2(_robot_pose.position.y - _tracked_object_center.y,
                            _robot_pose.position.x - _tracked_object_center.x);
      geometry_msgs::PoseStamped msg;
      msg.header.frame_id = _dst_frame;
      msg.header.stamp = scan_msg->header.stamp;
      vision_utils::copy2(goal.position, msg.pose.position);
      msg.pose.position.z = scan_z_dst_frame;
      msg.pose.orientation = tf::createQuaternionMsgFromYaw(goal.yaw);
      _goal_pub.publish(msg);
    } // end if TARGET_TRACKING_OK

    // publish status
    followme_laser::TrackingStatus msg;
    msg.status = _current_status;
    _status_pub.publish(msg);

    /*
    * markers
    */
    // publish marker for _closest_obj
    if (_closest_cloud_pub.getNumSubscribers()) {
      //vision_utils::vector2path(_closest_obj_pts, _closest_cloud);
      _closest_cloud.header.stamp = scan_msg->header.stamp;
      _closest_cloud.header.frame_id = _dst_frame;
      _closest_cloud.points = _closest_obj_pts;
      for (unsigned int i = 0; i < _closest_cloud.points.size(); ++i)
        _closest_cloud.points[i].z += .1; // make pts slightly higher
      //_closest_cloud.points = _pts_dst_frame;
      _closest_cloud_pub.publish(_closest_cloud);
    }

    // publish marker for _tracked_object
    if (_tracked_cloud_pub.getNumSubscribers()) {
      _tracked_cloud.header.stamp = scan_msg->header.stamp;
      _tracked_cloud.header.frame_id = _dst_frame;
      _tracked_cloud.points = _tracked_object;
      for (unsigned int i = 0; i < _tracked_cloud.points.size(); ++i)
        _tracked_cloud.points[i].z += .2; // make pts slightly higher
      if (_current_status == TrackingStatus::TARGET_TRACKING_LOST) {
        // make it blinking
        double sec_rest = ros::Time::now().toSec() - (int) ros::Time::now().toSec();
        if (sec_rest < 0.25 || (0.5 < sec_rest && sec_rest < 0.75))
          _tracked_cloud.points.clear();
      } // end if LOST
      _tracked_cloud_pub.publish(_tracked_cloud);
    }

    ROS_DEBUG_THROTTLE(1, "_closest_obj: (%g, %g) (%li pts)",
                       _closest_obj_pt.x, _closest_obj_pt.y,
                       _closest_obj_pts.size());

    //ROS_INFO_THROTTLE(1, "time for scan_cb(): %g ms", timer.time());
  } // end scan_cb();

  //////////////////////////////////////////////////////////////////////////////

  void tracking_seed_cb(const geometry_msgs::PoseStampedConstPtr & pose_in) {
    ROS_INFO_THROTTLE(1, "RosLaserBlobTracker:tracking_seed_cb(%s)",
                      vision_utils::pose_stamped_to_string(*pose_in).c_str());
    // make some sanity checks
    if (pose_in->pose.orientation.x == 0 && pose_in->pose.orientation.y == 0 &&
        pose_in->pose.orientation.z == 0 && pose_in->pose.orientation.w == 0) {
      ROS_WARN("tracking_seed_cb: invalid seed %s, stopping tracking.",
               vision_utils::pose_stamped_to_string(*pose_in).c_str());
      stop_tracking_object();
      return;
    }

    // convert to the static frame
    geometry_msgs::PoseStamped pose_dst_frame = *pose_in;
    if (pose_in->header.frame_id != _dst_frame) {
      pose_dst_frame.header.frame_id = _dst_frame;
      _tf_listener.transformPose(_dst_frame, ros::Time(0),
                                 *pose_in, _dst_frame, pose_dst_frame);
    }
    // set the new goal
    Pt2 new_goal;
    new_goal.x = pose_dst_frame.pose.position.x;
    new_goal.y = pose_dst_frame.pose.position.y;
    start_tracking_closest_object_at_given_point(new_goal);
  } // end tracking_seed_cb();

  //////////////////////////////////////////////////////////////////////////////

  void joy_cb(const sensor_msgs::Joy::ConstPtr& joy) {
    // ROS_INFO("joy_cb()");

    /* A super nice drawing for the code of the buttons on the
       Logitech RumblePad 2:
       ---------------
      |  [6]     [7]  |
      |  [4]     [5]  |
       ---------------
      |   |      (3)  |
      |  -+-   (0) (2)|
      |   |      (1)  |
      / /-----------\ \
     / /             \ \
      */
    if (joy->buttons[4] || joy->buttons[6])
      stop_tracking_object();
    else if (joy->buttons[5] || joy->buttons[7])
      start_tracking_closest_object();
  } // end joy_cb();

  //////////////////////////////////////////////////////////////////////////////

  ros::NodeHandle _nh_public;
  tf::TransformListener _tf_listener;
  ros::Subscriber _joy_sub, _tracking_seed_sub, _laser_sub;
  std::string _dst_frame;
  SimplePose2D _robot_pose;
  std::vector<Pt2> _pts_src_frame, _pts_dst_frame;
  sensor_msgs::PointCloud _closest_cloud, _tracked_cloud;
  ros::Publisher _closest_cloud_pub, _tracked_cloud_pub, _goal_pub, _status_pub;
}; // end RosLaserBlobTracker

} // end namespace followme_laser

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void interface_command_line() {
  followme_laser::RosLaserBlobTracker node;
  ros::AsyncSpinner spinner(0);
  spinner.start();

  while (ros::ok()) {
    ROS_WARN("-");
    ROS_WARN("*** RosLaserBlobTracker status: %s", node.get_current_status_as_string());
    ROS_WARN("0: exit");
    ROS_WARN("1: start tracking closest object");
    ROS_WARN("2: stop tracking");
    std::cout << "> ";
    int choice;
    std::cin >> choice;
    if (choice == 0)
      break;
    if (choice == 1)
      node.start_tracking_closest_object();
    else if (choice == 2)
      node.stop_tracking_object();
  } // end while true

  spinner.stop();
}

////////////////////////////////////////////////////////////////////////////////

void launcher() {
  followme_laser::RosLaserBlobTracker node;
  ros::spin();
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  int idx = 1;
  if (argc < 2) {
    printf("%i: interface_command_line()\n", idx++);
    printf("%i: launcher()\n", idx++);
    return -1;
  }
  int choice = atoi(argv[1]);

  ros::init(argc, argv, "test_followme_laser_ros");
  if (choice == idx++)
    interface_command_line();
  else if (choice == idx++)
    launcher();
  return 0;
}
