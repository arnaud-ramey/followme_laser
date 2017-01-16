#ifndef FOLLOWME_LASER_H
#define FOLLOWME_LASER_H

// AD
#include "vision_utils/timer.h"
#include "vision_utils/norm2.h"
#include "vision_utils/barycenter.h"
#include "vision_utils/distance_points_squared.h"
#include "followme_laser/TrackingStatus.h"
#include <nav_msgs/Path.h>

namespace followme_laser {

////////////////////////////////////////////////////////////////////////////////

template<typename _Pt2>
class LaserBlobTracker {
public:

  //! a very straightforward 2D point structure
  struct SimplePose2D {
    _Pt2 position;
    double yaw;
    //! ctor
    SimplePose2D(const double & x_, const double & y_, const double & yaw_): yaw(yaw_) {
      position.x = x_;
      position.y = y_;
    }
  }; // end SimplePose2D

  //////////////////////////////////////////////////////////////////////////////

  LaserBlobTracker()
    :   _object_max_radius_squared(.50 * 50),
      _object_max_distance_btwn_points_squared(.20 * .20),
      _object_max_searching_radius_squared(.35 * .35),
      _object_timeout(.5),
      _distance_to_object(.05) {
    stop_tracking_object();
  }

  //////////////////////////////////////////////////////////////////////////////

  void set_object_max_radius(const double & x) {
    _object_max_radius_squared = x * x;
  }
  void set_object_max_distance_btwn_points(const double & x) {
    _object_max_distance_btwn_points_squared = x * x;
  }
  void set_object_max_searching_radius(const double & x) {
    _object_max_searching_radius_squared = x * x;
  }
  void set_object_timeout(const double & x) {
    _object_timeout = x;
  }
  void set_distance_to_object(const double & x) {
    _distance_to_object = x;
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void set_laser_data(const std::vector<_Pt2> & current_pts,
                             const SimplePose2D & current_robot_pose) {
    _current_pts = &current_pts;
    _current_robot_pose = &current_robot_pose;
    update_tracking();
  }

  //////////////////////////////////////////////////////////////////////////////

  virtual inline void start_tracking_closest_object()  {
    _tracked_object_last_appearance.reset();
    _tracked_object = _closest_obj_pts;
    _tracked_object_center = vision_utils::barycenter(_tracked_object);
    _current_status = TrackingStatus::TARGET_TRACKING_OK;
  }

  //////////////////////////////////////////////////////////////////////////////

  virtual inline void stop_tracking_object() {
    _current_status = TrackingStatus::NO_TARGET;
    _tracked_object.clear();
  }
  //////////////////////////////////////////////////////////////////////////////

  inline const char* get_current_status_as_string() const {
    switch(_current_status) {
      case TrackingStatus::NO_TARGET:
        return "no target"; break;
      case TrackingStatus::TARGET_TRACKING_OK:
        return "target tracking:OK"; break;
      case TrackingStatus::TARGET_TRACKING_LOST: default:
        return "target tracking:LOST"; break;
    }
    return "";
  } // end get_current_status_as_string()

protected:

  //////////////////////////////////////////////////////////////////////////////

  inline void find_closest_point_from_pt(const _Pt2 & pt,
                                         const _Pt2* & closest_laser_pt,
                                         int & closest_laser_idx,
                                         double & closest_laser_dist) const {
    closest_laser_dist = INFINITY;
    closest_laser_pt = NULL;
    const _Pt2* curr_pt = &((*_current_pts)[0]);
    for (unsigned int curr_pt_idx = 0;
         curr_pt_idx < _current_pts->size();
         ++curr_pt_idx) {
      double curr_dist = vision_utils::distance_points_squared(pt, *curr_pt++);
      if (curr_dist < closest_laser_dist) {
        closest_laser_pt = curr_pt;
        closest_laser_idx = curr_pt_idx;
        closest_laser_dist = curr_dist;
      } // end if dist < best_dist
    } // end loop pt_idx
  } // end find_closest_point_from_pt();

  //////////////////////////////////////////////////////////////////////////////

  inline void find_cluster_from_laser_pt(const int & laser_start_idx,
                                         std::vector<_Pt2> & ans) const {
    ans.clear();
    const _Pt2* start_pt = &((*_current_pts)[laser_start_idx]);
    ans.push_back(*start_pt);
    // -1: backward
    for (int direction = -1; direction <= 1; direction += 2) {
      int curr_pt_idx = laser_start_idx;
      const _Pt2* curr_pt = start_pt, *prev_pt = start_pt;
      while (true) {
        // move pointers while avoiding overflows
        curr_pt_idx += direction;
        if (curr_pt_idx < 0 || curr_pt_idx >= (int) _current_pts->size()) {
          //    ROS_INFO("direction:%i, curr_pt_idx:%i -> vector overflow",
          //             direction, curr_pt_idx);
          break;
        }
        (direction == -1 ? --curr_pt : ++curr_pt);

        // stop if the last neighbour too remote from the origin
        if (vision_utils::distance_points_squared(*start_pt, *curr_pt)
            > _object_max_radius_squared) {
          //    ROS_INFO("direction:%i, curr_pt_idx:%i -> "
          //             "distance with start too high",
          //             direction, curr_pt_idx);
          break;
        }

        // stop if two last neighbours too remote
        if (vision_utils::distance_points_squared(*prev_pt, *curr_pt)
            > _object_max_distance_btwn_points_squared) {
          //    ROS_INFO("direction:%i, curr_pt_idx:%i -> "
          //             "distance between neighbours too high",
          //             direction, curr_pt_idx);
          break;
        }
        prev_pt = curr_pt;

        // we passed all tests : add pt
        ans.push_back(*curr_pt);
      } // end while (true)
    } // end loop direction
  } // end find_cluster_from_laser_pt();

  //////////////////////////////////////////////////////////////////////////////


  /*!
    Starts tracking around a given point
    The point must be given in the same frame the data of the laser is supplied.
   \param tracking_pt
  */
  inline bool start_tracking_closest_object_at_given_point
  (const _Pt2 tracking_pt)
  {
    //    ROS_INFO_THROTTLE
    //        (1, "LaserBlobTracker:start_tracking_closest_object_at_given_point(%g, %g)",
    //         tracking_pt.x, tracking_pt.y);
    const _Pt2* closest_laser_pt;
    int closest_laser_pt_index;
    double closest_laser_pt_dist;
    // find closest laser pt from tracked center
    find_closest_point_from_pt(tracking_pt,
                               closest_laser_pt,
                               closest_laser_pt_index,
                               closest_laser_pt_dist);
    // if closest laser pt too remote, object is lost
    if (closest_laser_pt_dist > _object_max_searching_radius_squared) {
      _current_status = TrackingStatus::TARGET_TRACKING_LOST;
      if (_tracked_object_last_appearance.getTimeMilliseconds() / 1000.f
          > _object_timeout) {
        printf("The object has disappeared for too long (%f > timeout=%f). "
               "Stopping tracking.\n",
               _tracked_object_last_appearance.getTimeMilliseconds() / 1000.f,
               _object_timeout);
        stop_tracking_object();
      }
      return false;
    } // end if dist > _object_max_searching_radius_squared
    // otherwise, tracking successful:
    else {
      _current_status = TrackingStatus::TARGET_TRACKING_OK;
      _tracked_object_last_appearance.reset();
      // replace _tracked_object and _tracked_object_center
      find_cluster_from_laser_pt(closest_laser_pt_index, _tracked_object);
      _tracked_object_center = vision_utils::barycenter(_tracked_object);
      // update safe goal
      // set a goal at _distance_to_object away from the goal
      _Pt2 vec_robot_to_obj_center;
      vec_robot_to_obj_center.x = _tracked_object_center.x - _current_robot_pose->position.x;
      vec_robot_to_obj_center.y = _tracked_object_center.y - _current_robot_pose->position.y;
      double ratio = (1 - _distance_to_object /
                      vision_utils::norm2(vec_robot_to_obj_center));
      _safe_goal_to_tracked_obj.x = _current_robot_pose->position.x + ratio * vec_robot_to_obj_center.x;
      _safe_goal_to_tracked_obj.y = _current_robot_pose->position.y + ratio * vec_robot_to_obj_center.y;

    } // end if dist < _object_max_searching_radius_squared
    return true;
  } // end start_tracking_closest_object_at_given_point();


  //////////////////////////////////////////////////////////////////////////////

  inline void update_tracking() {
    // find closest object from robot
    const _Pt2* closest_laser_pt;
    int closest_laser_index;
    double closest_pt_dist;
    find_closest_point_from_pt(_current_robot_pose->position,
                               closest_laser_pt, closest_laser_index, closest_pt_dist);
    _closest_obj_pt = *closest_laser_pt;
    find_cluster_from_laser_pt(closest_laser_index, _closest_obj_pts);

    // if status is no target, do nothing
    if (_current_status == TrackingStatus::NO_TARGET){
    } // end if status == NO_TARGET

    // if status tracking, try to find closest object from tracked object
    else {
      start_tracking_closest_object_at_given_point(_tracked_object_center);
    } // end if TARGET
  } // end update_tracking();

  //////////////////////////////////////////////////////////////////////////////

  int _current_status;

  // tracking params
  //! the maximum radius of an object
  double _object_max_radius_squared;
  //! the maximum distance between two consecutive points of an object
  double _object_max_distance_btwn_points_squared;
  /*! the maximum distance for looking for a
      lost object around its last position */
  double _object_max_searching_radius_squared;
  //! time in second to declare an object lost (sec)
  double _object_timeout;
  //! the distance for a safe goal for navig (meters)
  double _distance_to_object;
  // current data
  const std::vector<_Pt2>* _current_pts;
  const SimplePose2D* _current_robot_pose;

  // closest object
  std::vector<_Pt2> _closest_obj_pts;
  _Pt2 _closest_obj_pt;

  // tracked object
  vision_utils::Timer _tracked_object_last_appearance;
  std::vector<_Pt2> _tracked_object;
  _Pt2 _tracked_object_center;
  _Pt2 _safe_goal_to_tracked_obj;
}; // end class LaserBlobTracker

} // end namespace followme_laser


#endif // FOLLOWME_LASER_H
