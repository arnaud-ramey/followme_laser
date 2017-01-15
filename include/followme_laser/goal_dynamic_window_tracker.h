#ifndef _GoalDynamicWindowTracker_H_
#define _GoalDynamicWindowTracker_H_

#include <vector>
#include <math.h>
#include <vision_utils/odom_utils.h>
#include <vision_utils/distance_points.h>
#include <vision_utils/distance_points_squared.h>
#include <vision_utils/clamp.h>
#include <vision_utils/timer.h>

#define RAD2DEG     57.2957795130823208768  //!< to convert radians to degrees
#define DEG2RAD     0.01745329251994329577  //!< to convert degrees to radians

/*!
 \param A
    a 2D point
 \param B
    a vector of 2D points
 \return min_dist
    min distance otherwise
*/
template<class _Pt2>
inline double vectors_dist(const _Pt2 & pt,
                           const std::vector<_Pt2> & vec) {
  double min_dist_sq = 1E10;
  unsigned int npts = vec.size();
  for (unsigned int i = 0; i < npts; ++i) {
    double dist = vision_utils::distance_points_squared(pt, vec[i]);
    if (dist < min_dist_sq)
      min_dist_sq = dist;
  } // end loop i
  return sqrt(min_dist_sq);
} // end vectors_dist_thres()

////////////////////////////////////////////////////////////////////////////////
/*!
 \param A
    a vector of 2D points
 \param B
    a vector of 2D points
 \param min_dist
    a threshold distance
 \return min_dist
    -1 if vectors closer than min_dist, min distance otherwise
*/
template<class _Pt2>
inline double vectors_dist_thres(const std::vector<_Pt2> & A,
                                 const std::vector<_Pt2> & B,
                                 const double dist_thres) {
  double dist_thres_sq = dist_thres * dist_thres, min_dist_sq = 1E10;
  unsigned int nA = A.size(), nB = B.size();
  for (unsigned int A_idx = 0; A_idx < nA; ++A_idx) {
    for (unsigned int B_idx = 0; B_idx < nB; ++B_idx) {
      double dist = vision_utils::distance_points_squared(A[A_idx], B[B_idx]);
      if (dist < dist_thres_sq)
        return -1;
      if (dist < min_dist_sq)
        min_dist_sq = dist;
    } // end loop B_idx
  } // end loop A_idx
  return sqrt(min_dist_sq);
} // end vectors_dist_thres()

/*!
* \struct SpeedOrder
* A minimalistic structure for representing an order sent to a mobile base,
* made of a linear and an angular speed.
*/
struct SpeedOrder {
  double v, w;
  SpeedOrder() : v(0), w(0) {}
  SpeedOrder(const double & lin, const double & ang)
    : v(lin), w(ang) {}
}; // end SpeedOrder

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

template<class Pt2>
class GoalDynamicWindowTracker {
public:
  enum Action {
    ACTION_UNDEFINED = -1,
    ACTION_KEEP_SAME_SPEED = 0,
    ACTION_STOP = 1,
    ACTION_ROTATE_ON_PLACE = 2,
    ACTION_RECOMPUTE_SPEED = 3,
  };

  struct Pose2 {
    Pt2 position;
    double yaw;
  };

  //////////////////////////////////////////////////////////////////////////////

  GoalDynamicWindowTracker() {
    _current_action = ACTION_RECOMPUTE_SPEED;
    _was_stopped = true;
    _min_v = _max_v = _max_w = -1;
    // default  params
    _current_robot_pose.position.x = _current_robot_pose.position.y = 0;
    _current_robot_pose.yaw = 0;
    set_simulation_parameters();
    set_goal_parameters();
    unset_goal();
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void set_costmap(const std::vector<Pt2> & costmap_cell_centers,
                          const double min_obstacle_distance) {
    _costmap_cell_centers = costmap_cell_centers;
    _laser_thickness = vision_utils::clamp(fabs(min_obstacle_distance), .01, 10.);
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void set_limit_speeds(double min_v, double max_v, double max_w) {
    _min_v = min_v;
    _max_v = max_v;
    _max_w = max_w;
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void set_simulation_parameters(const double time_pred = 5,
                                        const double time_step = .2,
                                        double speed_recomputation_timeout = 1) {
    _time_pred = time_pred;
    _time_step = time_step;
    _speed_recomputation_timeout = speed_recomputation_timeout;
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void set_goal_parameters(double min_goal_distance = .6, double max_goal_angle = .2) {
    _min_goal_distance = min_goal_distance;
    _max_goal_angle = max_goal_angle;
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void set_goal(Pt2 goal) {
    _goal = goal;
    _goal_set = true;
  }
  virtual void unset_goal() {
    _goal_set = false;
    stop_robot();
  }

  //////////////////////////////////////////////////////////////////////////////

  inline std::vector<Pt2> get_best_trajectory() {
    vision_utils::make_trajectory(// get_best_element().element.v,
                                  _curr_order.v,
                                  // get_best_element().element.w,
                                  _curr_order.w,
                                  _traj_buffer,
                                  _time_pred, _time_step,
                                  0, 0, 0);
    //start_pos.x, start_pos.y, start_yaw);
    return _traj_buffer;
  }

  //////////////////////////////////////////////////////////////////////////////

  virtual bool recompute_speeds(double & best_speed_lin,
                                double & best_speed_ang) {
    _current_action = ACTION_KEEP_SAME_SPEED;
    if (!_goal_set) {
      _current_action = ACTION_STOP;
    }

    // recompute speeds if the last ones are too old
    if (_current_action == ACTION_KEEP_SAME_SPEED
        && _last_speed_age.getTimeSeconds() > _speed_recomputation_timeout) {
      printf("speed_timeout\n");
      _current_action = ACTION_RECOMPUTE_SPEED;

    }
    // check if there will be a collision soon with the elected speeds
    if (_current_action == ACTION_KEEP_SAME_SPEED &&
        trajectory_grade(_curr_order.v, _curr_order.w, _costmap_cell_centers) == false) {
      printf("coming_collision !\n");
      _current_action = ACTION_RECOMPUTE_SPEED;
    } // end

    // determine if we have reached the goal - 2D distance
    float curr_goal_distance = vision_utils::distance_points_squared
        (_current_robot_pose.position, _goal);

    // stop going forward if we are too close
    float goal_angle = atan2(_goal.y, _goal.x); // in the range [-pi, pi]
    if (_current_action == ACTION_KEEP_SAME_SPEED &&
        curr_goal_distance < _min_goal_distance) {
      // not centered to goal => rotate on place
      if (fabs(goal_angle) > _max_goal_angle) {
        printf("Close enough from goal (dist:%g < min_goal_distance:%g) "
               "but need to rotate: fabs(goal_angle:%g) > max_goal_angle:%g\n",
               curr_goal_distance, _min_goal_distance,
               fabs(goal_angle), _max_goal_angle);
        _current_action = ACTION_ROTATE_ON_PLACE;
      } // end if (fabs(goal_angle) > _max_goal_angle)
      else { // centered to goal => stop
        printf("Close enough from goal (dist:%g < min_goal_distance:%g), "
               " angle=%g rad so no need to rotate...\n",
               curr_goal_distance, _min_goal_distance, goal_angle);
        _current_action = ACTION_STOP;
        if (!_was_stopped){
          printf("Goal reached (dist:%g < min_goal_distance:%g) ! Stopping.\n",
                 curr_goal_distance, _min_goal_distance);
        } // end if (!was_stopped)
      } // end if centered
    } // end if (curr_goal_distance < _min_goal_distance)

    /*
     * apply actions
     */
    if (_current_action == ACTION_KEEP_SAME_SPEED) {
      best_speed_lin = _curr_order.v;
      best_speed_ang = _curr_order.w;
      return true; // nothing to do
    }

    else if (_current_action == ACTION_STOP) {
      stop_robot();
      return true; // OK
    } // end ACTION_STOP

    else if (_current_action == ACTION_ROTATE_ON_PLACE) {
      // angle
      // (+) <--- 0 --> (-)
      double min_rotate_on_place_speed = 0;
      double max_rotate_on_place_speed = _max_w;
      double rotation_speed = min_rotate_on_place_speed +
          (max_rotate_on_place_speed - min_rotate_on_place_speed)
          / 2;
          //* fabs(goal_angle) / M_PI; // this ratio is in [-1, 1]
      if (goal_angle < 0)
        set_speed(SpeedOrder(0, -rotation_speed));
      else
        set_speed(SpeedOrder(0, +rotation_speed));
    } // end ACTION_ROTATE_ON_PLACE

    else if (_current_action == ACTION_RECOMPUTE_SPEED) {
      bool new_speeds_found = false;
      _last_speed_age.reset();

      // need a combination of linear and angular speed
      double best_grade = best_grade_in_range(_min_v, _max_v, _max_w);
      if (best_grade < 0) // extreme case -> go backward
        best_grade = best_grade_in_range(-(_min_v+_max_v)*.5, -_min_v, _max_w);
      new_speeds_found = (best_grade > 0);

      if (!new_speeds_found) {
        printf("The robot is stuck! Stopping motion and exiting.\n");
        stop_robot();
        return false; // error
      } // end not new_speeds_found

      printf("Found a suitable couple of speeds in %g ms:"
             "_v:%g, _w:%g\n",
             _last_speed_age.getTimeMilliseconds(),
             _curr_order.v,
             _curr_order.w);
    } // end ACTION_RECOMPUTE_SPEED

    // publish the computed speed
    printf("DynamicWindow: Publishing _v:%g, _w:%g\n",
           _curr_order.v, _curr_order.w);
    best_speed_lin = _curr_order.v;
    best_speed_ang = _curr_order.w;
    return true;  // error
  } // end recompute_speeds()

protected:


  ////////////////////////////////////////////////////////////////////////////////

  inline void set_speed(const SpeedOrder & new_speed) {
    printf("set_speed(lin:%g, ang:%g)\n",
           new_speed.v, new_speed.w);
    _was_stopped = (new_speed.v == 0) && (new_speed.w == 0);
    _curr_order = new_speed;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! stop the robot
  inline void stop_robot() {
    set_speed(SpeedOrder());
  }

  ////////////////////////////////////////////////////////////////////////////////

  //! return < 0 if the trajectory will collide with the laser in the time TIME_PRED,
  //! distance to closest obstacle otherwise + bonus for speed
  double trajectory_grade(const double & v, const double & w,
                          const std::vector<Pt2> & laser_xy) {
    // determine the coming trajectory
    vision_utils::make_trajectory(v, w, _traj_buffer, _time_pred, _time_step, 0, 0, 0);
    // find if there might be a collision
    double obs_dist = vectors_dist_thres(_traj_buffer, laser_xy,
                                         _laser_thickness);
    if (obs_dist < 0)
      return -1;
    // return inv of dist to goal - the higher the better
//    double grade = 1 / vision_utils::distance_points
//        (_traj_buffer.back(), _goal);
//    grade -= 1 / obs_dist;
//    return grade;
    return 1. / (vectors_dist(_goal, _traj_buffer) + obs_dist / 5);
  }

  //////////////////////////////////////////////////////////////////////////////

  inline double best_grade_in_range(double min_v, double max_v, double max_w) {
    printf("best_grade_in_range(v:%g --> %g, w:%g --> %g)\n",
           min_v, max_v, -max_w, max_w);
    double best_grade = -1, dv = (max_v - min_v) / SPEED_STEPS, dw = 2 * max_w / SPEED_STEPS;
    for (double v = min_v; v <= max_v; v += dv) {
      for (double w = -max_w; w <= max_w; w += dw) {
        double currr_grade = trajectory_grade(v, w, _costmap_cell_centers);
        if (currr_grade < 0)
          continue;
        if (currr_grade < best_grade)
          continue;
        _curr_order.v = v;
        _curr_order.w = w;
        best_grade = currr_grade;
      } // end loop w
    } // end loop v
    return best_grade;
  } // end best_grade_in_range()

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  std::vector<Pt2> _traj_buffer;

  // control
  Action _current_action;
  //! timer since last computed speed
  vision_utils::Timer _last_speed_age;
  //! the current velocities, m/s or rad/s
  SpeedOrder _curr_order;
  bool _was_stopped;

  // obstacles
  std::vector<Pt2> _costmap_cell_centers;
  double _laser_thickness;
  double _min_goal_distance, _max_goal_angle;

  // robot parameters
  Pose2 _current_robot_pose;
  double _min_v, _max_v, _max_w;
  Pt2 _goal;
  bool _goal_set;

  // simul parameters
  double _speed_recomputation_timeout;
  //! the forecast time in seconds
  double _time_pred;
  //! the time step simulation
  double _time_step;

  // GoalDynamicWindowTracker params
  static const unsigned int MAX_TRIES = 1E6, SPEED_STEPS = 50;
}; // end class GoalDynamicWindowTracker

#endif // _GoalDynamicWindowTracker_H_
