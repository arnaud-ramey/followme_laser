#ifndef ROS_UTILS_H
#define ROS_UTILS_H

#include <vector>
#include <stdio.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

namespace ros_utils {

template<class Pt2>
static inline bool convert_xy_vec_frame(const std_msgs::Header & src_header,
                                        const std::vector<Pt2> & src_vector,
                                        tf::TransformListener & _tf_listener,
                                        const std::string & dst_frame,
                                        std::vector<Pt2> & dst_vector,
                                        double & scan_z_dst_frame) {
  std::string src_frame = src_header.frame_id;
  if (src_frame.empty()) { // empty scan?
    ROS_WARN("convert_xy_vec_frame(): empty source frame_id!");
    return false;
  }
  if (src_frame != dst_frame) {
    //ROS_DEBUG_ONCE("Src and dest frame equal '%s'", dst_frame.c_str());
    dst_vector = src_vector;
    return true;
  }
  dst_vector.clear();
  unsigned int src_npts = src_vector.size();
  dst_vector.resize(src_npts);
  bool transform_ok =
      _tf_listener.waitForTransform(dst_frame,
                                    src_frame,
                                    src_header.stamp,
                                    ros::Duration(1));
  if (!transform_ok) {
    ROS_WARN("Impossible to find tf '%s' -> %s'. Returning.",
             src_frame.c_str(), dst_frame.c_str());
    return false;
  }
  ROS_DEBUG_THROTTLE(1, "Success for finding tf '%s' -> %s'.",
                     src_frame.c_str(), dst_frame.c_str());

  // convert all the points from src_vector
  geometry_msgs::PointStamped pt_stamped_in, pt_stamped_out;
  pt_stamped_in.point.z = 0;
  pt_stamped_in.header = src_header;
  try {
    for (unsigned int pt_idx = 0; pt_idx < src_npts; ++pt_idx) {
      utils::copy2(src_vector[pt_idx], pt_stamped_in.point);
      _tf_listener.transformPoint(dst_frame,  ros::Time(0),
                                  pt_stamped_in, dst_frame, pt_stamped_out);
      utils::copy2(pt_stamped_out.point, dst_vector[pt_idx]);
    } // end loop pt_idx

    // keep the z coordinate of the scan
    scan_z_dst_frame = pt_stamped_out.point.z;
    //ROS_WARN_THROTTLE(1, "scan_z_dst_frame:%g", scan_z_dst_frame);

  } catch (tf::ExtrapolationException e) {
    ROS_WARN("transform error:'%s'", e.what());
    return false;
  } // end try / catch
  return true;
} // end convert_xy_vec_frame()

////////////////////////////////////////////////////////////////////////////////

template<class _Pose3Stamped>
inline std::string pose_stamped_to_string(const _Pose3Stamped & pose) {
  std::ostringstream ans;
  ans << "pose: {"
      << "t:" << std::fixed << pose.header.stamp.toSec() << "s, "
      << "frame: '" << pose.header.frame_id.c_str() << "' : "
      << "pos:" << utils::printP(pose.pose.position)
      << ", orien:" << utils::printP4(pose.pose.orientation)
      << " }";
  return ans.str();
}

////////////////////////////////////////////////////////////////////////////////

//! convert from polar to xy coordinates for a laser data
template<class _Pt2>
static inline void convert_sensor_data_to_xy(const sensor_msgs::LaserScan & laser_msg,
                                             std::vector<_Pt2> & out_vector) {
  out_vector.clear();
  out_vector.reserve(laser_msg.ranges.size());
  const float* curr_range = &(laser_msg.ranges[0]);
  float curr_angle = laser_msg.angle_min;
  for (unsigned int idx = 0; idx < laser_msg.ranges.size(); ++idx) {
    //maggieDebug2("idx:%i, curr_range:%g", idx, *curr_range);
    _Pt2 pt;
    pt.x = *curr_range * cos(curr_angle);
    pt.y = *curr_range * sin(curr_angle);
    out_vector.push_back(pt);
    ++curr_range;
    curr_angle += laser_msg.angle_increment;
  } // end loop idx
} // end convert_sensor_data_to_xy()

////////////////////////////////////////////////////////////////////////////////

template<class _Pt2>
inline void vector2path(const std::vector<_Pt2> & traj_xy, nav_msgs::Path & _path_msg) {
  unsigned int npts = traj_xy.size();
  _path_msg.poses.resize(npts);
  for (int i = 0; i < npts; ++i) {
    _path_msg.poses[i].pose.position.x = traj_xy[i].x;
    _path_msg.poses[i].pose.position.y = traj_xy[i].y;
    _path_msg.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(0);;
  }
}

} // end namespace ros_utils

#endif // ROS_UTILS_H
