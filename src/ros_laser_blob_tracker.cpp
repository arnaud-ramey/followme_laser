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
      [nav_msgs::Path]
      A marker to show the cluster closest to the robot and the tracked cluster.

 - \b "~status"
      [followme_laser::TrackingStatus]
      The status of the tracking.

 */
// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
// LaserBlobTracker
#include <followme_laser/laser_blob_tracker.h>
#include <followme_laser/utils.h>
#include <followme_laser/ros_utils.h>

#define PI          M_PI
#define RAD2DEG     57.2957795130823208768  //!< to convert radians to degrees
#define DEG2RAD     0.01745329251994329577  //!< to convert degrees to radians

namespace followme_laser {

/*! a generic templated class for 2D points.
  It contains a x and a y field so as to be compatible
  with OpenCV cv::Point*
*/
class SimplePt2D {
public:
  //! a constructor without arguments
  SimplePt2D() : x(0), y(0) {}

  //! a constructor
  SimplePt2D(const float & x_, const float & y_) :
    x(x_), y(y_) {}

  //! the + operator, that substracts field by field
  SimplePt2D operator + (const SimplePt2D& B) const {
    return SimplePt2D(x + B.x, y + B.y);
  }

  //! the - operator, that substracts field by field
  SimplePt2D operator - (const SimplePt2D& B) const {
    return SimplePt2D(x - B.x, y - B.y);
  }

  //! the * operator, that multiplies field by field
  SimplePt2D operator * (const float & alpha) const {
    return SimplePt2D(alpha * x, alpha * y);
  }

  //! define the output to a stream
  friend std::ostream & operator << (std::ostream & stream,
                                     const SimplePt2D & P) {
    stream << '[' << P.x << ", " << P.y << ']';
    return stream;
  }

  //! return a string representation of the point
  inline std::string to_string() const {
    std::ostringstream ans;
    ans << *this;
    return ans.str();
  }

  float x; //!< the first data field
  float y; //!< the second data field
}; // end SimplePt2D

////////////////////////////////////////////////////////////////////////////////

class RosLaserBlobTracker : public LaserBlobTracker<SimplePt2D> {
public:

  //! ctor
  RosLaserBlobTracker() :
    _robot_pose(0, 0, 0)
  {
    ros::NodeHandle nh_private("~");

    // get dst frame
    nh_private.param<std::string>("dst_frame", _dst_frame, "/base_link");

    // get topic name
    std::string scan_topic;
    nh_private.param<std::string>("scan_topic", scan_topic, "scan");
    _laser_sub = _nh_public.subscribe
                 (scan_topic, 1, &RosLaserBlobTracker::scan_cb, this);

    _goal_pub = _nh_public.advertise<geometry_msgs::PoseStamped>
                ("moving_goal", 1);
    // joy subscriber
    _joy_sub = _nh_public.subscribe<sensor_msgs::Joy>
               ("joy", 1,  &RosLaserBlobTracker::joy_cb, this);
    // tracking seed subscriber
    _tracking_seed_sub = nh_private.subscribe
                         ("tracking_seed", 1, &RosLaserBlobTracker::tracking_seed_cb, this);
    // marker publisher
    _closest_path_pub = nh_private.advertise<nav_msgs::Path>
                        ("closest_object", 1);
    _tracked_path_pub = nh_private.advertise<nav_msgs::Path>
                        ("tracked_object", 1);
    // status publisher
    _status_pub= nh_private.advertise<followme_laser::TrackingStatus>
                 ("status", 1);

    ROS_WARN("RosLaserBlobTracker: scan_topic:'%s', _goal_topic:'%s', "
             "_path topic:'%s' (status on '%s')",
             _laser_sub.getTopic().c_str(), _goal_pub.getTopic().c_str(),
             _closest_path_pub.getTopic().c_str(),
             _status_pub.getTopic().c_str());
  } // end ctor

  //////////////////////////////////////////////////////////////////////////////

  virtual inline void start_tracking_closest_object() {
    stop_tracking_object();
    LaserBlobTracker<SimplePt2D>::start_tracking_closest_object();
  }

  //////////////////////////////////////////////////////////////////////////////

  virtual inline void stop_tracking_object() {
    LaserBlobTracker<SimplePt2D>::stop_tracking_object();
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
      utils::copy2(robot_pose_out.pose.position, _robot_pose.position);
      _robot_pose.yaw = tf::getYaw(robot_pose_out.pose.orientation);
      ROS_DEBUG_THROTTLE(1, "The robot is in (%s), yaw:%g degrees in frame %s.",
                         _robot_pose.position.to_string().c_str(),
                         _robot_pose.yaw * RAD2DEG,
                         _dst_frame.c_str());
    } catch (tf::ExtrapolationException e) {
      ROS_WARN("transform error:'%s'", e.what());
      return;
    } // end try / catch

    /*
     * real laser tracking
     */
    ROS_DEBUG_THROTTLE(1, "Now set_laser_data(pts:%i)", _pts_dst_frame.size());
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
      utils::copy2(goal.position, msg.pose.position);
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
    if (_closest_path_pub.getNumSubscribers()) {
      vision_utils::vector2path(_closest_obj_pts, _closest_path);
      _closest_path.header.stamp = scan_msg->header.stamp;
      _closest_path.header.frame_id = _dst_frame;
      _closest_path_pub.publish(_closest_path);
    }

    // publish marker for _tracked_object
    if (_tracked_path_pub.getNumSubscribers()) {
      vision_utils::vector2path(_tracked_object, _tracked_path);
      _tracked_path.header.stamp = scan_msg->header.stamp;
      _tracked_path.header.frame_id = _dst_frame;
      if (_current_status == TrackingStatus::TARGET_TRACKING_LOST) {
        // make it blinking
        double sec_rest = ros::Time::now().toSec() - (int) ros::Time::now().toSec();
        if (sec_rest < 0.25 || (0.5 < sec_rest && sec_rest < 0.75))
          _tracked_path.poses.clear();
      } // end if LOST
      if (!_tracked_path.poses.empty())
        _tracked_path_pub.publish(_tracked_path);
    }

    ROS_DEBUG_THROTTLE(1, "_closest_obj: %s (%i pts)",
                       _closest_obj_pt.to_string().c_str(),
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
    SimplePt2D new_goal(pose_dst_frame.pose.position.x,
                        pose_dst_frame.pose.position.y);
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
  std::vector<SimplePt2D> _pts_src_frame, _pts_dst_frame;
  nav_msgs::Path _closest_path, _tracked_path;
  ros::Publisher _closest_path_pub, _tracked_path_pub, _goal_pub, _status_pub;
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
