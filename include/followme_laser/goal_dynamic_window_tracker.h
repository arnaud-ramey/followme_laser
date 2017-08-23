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
//#define DEBUG_PRINT(...)   {}
#define DEBUG_PRINT(...)   printf(__VA_ARGS__)

/*!
 \param A
    a 2D point
 \param B
    a vector of 2D points
 \return min_dist
    min distance otherwise
*/
template<class _Pt2>
inline double pt2vector_dist(const _Pt2 & pt,
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
  if (A.empty() || B.empty())
    return 0;
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
    _current_action = ACTION_RECOMPUTE_SPEED; // force recompute at next step
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
    _min_v = fabs(min_v);
    _max_v = fabs(max_v);
    _max_w = fabs(max_w);
    _dv = (_max_v - _min_v) / SPEED_STEPS;
    _dw = 2 * _max_w / SPEED_STEPS;
    precompute_trajectories();
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void set_simulation_parameters(const double time_pred = 5,
                                        const double time_step = .2,
                                        double speed_recomputation_timeout = 1,
                                        double goal_timeout = 1) {
    _time_pred = time_pred;
    _time_step = time_step;
    _speed_recomputation_timeout = speed_recomputation_timeout;
    _goal_timeout = goal_timeout;
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
    _last_goal_age.reset();
    _current_action = ACTION_RECOMPUTE_SPEED; // force recompute at next step
  }
  virtual void unset_goal() {
    _goal_set = false;
    stop_robot();
  }

  //////////////////////////////////////////////////////////////////////////////

  inline const std::vector<Pt2> & get_best_trajectory() {
    return _trajs[_best_traj_idx];
  }

  //////////////////////////////////////////////////////////////////////////////

  //! get the current action
  inline Action get_current_action() const { return _current_action; }

  //////////////////////////////////////////////////////////////////////////////

  virtual bool recompute_speeds_if_needed(double & best_speed_lin,
                                          double & best_speed_ang) {
    DEBUG_PRINT("recompute_speeds_if_needed(%li cells)\n",
                _costmap_cell_centers.size());
    Action prev_action = _current_action;
    // determine if we have reached the goal - 2D distance
    float curr_goal_distance = vision_utils::distance_points
        (_current_robot_pose.position, _goal);
    // stop going forward if we are too close
    float curr_goal_angle = atan2(_goal.y, _goal.x); // in the range [-pi, pi]

    if (!_goal_set) {
      _current_action = ACTION_STOP;
    }
    // recompute speeds if the last ones are too old
    else if (_last_goal_age.getTimeSeconds() > _goal_timeout) {
      DEBUG_PRINT("goal timeout\n");
      _current_action = ACTION_STOP;
    }
    // recompute speeds if the last ones are too old
    else if (_last_speed_age.getTimeSeconds() > _speed_recomputation_timeout) {
      DEBUG_PRINT("speed timeout\n");
      _current_action = ACTION_RECOMPUTE_SPEED;
    }
    // check if there will be a collision soon with the elected speeds
    else if (trajectory_grade(get_best_trajectory(), _costmap_cell_centers) < 0) {
      DEBUG_PRINT("coming_collision with current speeds!\n");
      _current_action = ACTION_RECOMPUTE_SPEED;
    }
    else if (curr_goal_distance < _min_goal_distance) {
      // not centered to goal => rotate on place
      if (fabs(curr_goal_angle) > _max_goal_angle) {
        DEBUG_PRINT("Close enough from goal (dist:%g < min_goal_distance:%g) "
                    "but need to rotate: fabs(goal_angle:%g) > max_goal_angle:%g\n",
                    curr_goal_distance, _min_goal_distance,
                    fabs(curr_goal_angle), _max_goal_angle);
        _current_action = ACTION_ROTATE_ON_PLACE;
      } // end if (fabs(goal_angle) > _max_goal_angle)
      else { // centered to goal => stop
        DEBUG_PRINT("Close enough from goal (dist:%g < min_goal_distance:%g), "
                    " angle=%g rad so no need to rotate...\n",
                    curr_goal_distance, _min_goal_distance, curr_goal_angle);
        _current_action = ACTION_STOP;
        if (prev_action != ACTION_STOP){
          DEBUG_PRINT("Goal reached (dist:%g < min_goal_distance:%g) ! Stopping.\n",
                      curr_goal_distance, _min_goal_distance);
        } // end if (!was_stopped)
      } // end if centered
    } // end if (curr_goal_distance < _min_goal_distance)
    else {
      _current_action = ACTION_KEEP_SAME_SPEED; // nothing to do
    }

    /*
     * apply actions
     */
    if (_current_action == ACTION_KEEP_SAME_SPEED) {
      // nothing to do
    }

    else if (_current_action == ACTION_STOP) {
      stop_robot();
    } // end ACTION_STOP

    else if (_current_action == ACTION_ROTATE_ON_PLACE) {
      // angle
      // (+) <--- 0 --> (-)
      double min_rotate_on_place_speed = .1 * _max_w;
      double max_rotate_on_place_speed = _max_w;
      double angle_diff = fabs(curr_goal_angle) - _max_goal_angle; // between 0 and n
      double rotation_speed = vision_utils::clamp(angle_diff * .5,
                                                  min_rotate_on_place_speed,
                                                  max_rotate_on_place_speed); // get there in 5 sec
      //* fabs(goal_angle) / M_PI; // this ratio is in [-1, 1]
      _best_order.v = 0;
      if (curr_goal_angle < 0)
        _best_order.w = -rotation_speed;
      else
        _best_order.w = +rotation_speed;
      _best_traj_idx = speeds2idx(_best_order.v, _best_order.w);
    } // end ACTION_ROTATE_ON_PLACE

    else if (_current_action == ACTION_RECOMPUTE_SPEED) {
      bool new_speeds_found = false;
      _last_speed_age.reset();

      // need a combination of linear and angular speed
      double best_grade = best_grade_in_range();
      new_speeds_found = (best_grade > 0);

      if (!new_speeds_found) {
        printf("The robot is stuck! Lost for lost, going backwards.\n");
        _best_order.v = -(.9 * _min_v + .1 * _max_v);
        _best_order.w = 0;
        _best_traj_idx = speeds2idx(_best_order.v, _best_order.w);
      } // end not new_speeds_found

      printf("goal:%g m, %g rad, found a suitable couple of speeds in %g ms:"
             "_v:%g, _w:%g\n",
             curr_goal_distance, curr_goal_angle,
             _last_speed_age.getTimeMilliseconds(),
             _best_order.v, _best_order.w);
    } // end ACTION_RECOMPUTE_SPEED

    // publish the computed speed
    DEBUG_PRINT("DynamicWindow: Publishing _v:%g, _w:%g\n", _best_order.v, _best_order.w);
    best_speed_lin = _best_order.v;
    best_speed_ang = _best_order.w;
    return true;  // success
  } // end recompute_speeds()

protected:

  inline int speeds2idx(const double & speed_lin, const double & speed_ang) {
    int wi = vision_utils::clamp<int>(1. * (speed_ang + _max_w) / _dw, 0, SPEED_STEPS-1);
    int vi = vision_utils::clamp<int>(1. * (speed_lin - _min_v) / _dv, 0, SPEED_STEPS-1);
    return wi + vi * SPEED_STEPS;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! stop the robot
  inline void stop_robot() {
    _best_order.v = _best_order.w = 0;
    _best_traj_idx = speeds2idx(0, 0);
  }

  ////////////////////////////////////////////////////////////////////////////////

  //! return < 0 if the trajectory will collide with the laser in the time TIME_PRED,
  //! distance to closest obstacle otherwise + bonus for speed
  inline double trajectory_grade(const std::vector<Pt2> & traj,
                                 const std::vector<Pt2> & laser_xy) {
    // find if there might be a collision
    double obs_dist = vectors_dist_thres(traj, laser_xy,
                                         _laser_thickness);
    if (obs_dist < 0)
      return -1;
    // return inv of dist to goal - the higher the better
    return 1. / (pt2vector_dist(_goal, traj) + obs_dist / 3);
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void precompute_trajectories() {
    _trajs.clear();
    _trajs.reserve(SPEED_STEPS*SPEED_STEPS);
    for (double v = _min_v; v <= _max_v; v += _dv) {
      for (double w = -_max_w; w <= _max_w; w += _dw) {
        std::vector<Pt2> traj;
        vision_utils::make_trajectory(v, w, traj, _time_pred, _time_step, 0, 0, 0);
        _trajs.push_back(traj);
      } // end loop w
    } // end loop v
  }

  inline double best_grade_in_range() {
    DEBUG_PRINT("best_grade_in_range(v:%g --> %g, w:%g --> %g)\n",
                _min_v, _max_v, -_max_w, _max_w);
    double best_grade = -1;
    unsigned int traj_idx = 0;
    for (double v = _min_v; v <= _max_v; v += _dv) {
      for (double w = -_max_w; w <= _max_w; w += _dw) {
        double currr_grade = trajectory_grade(_trajs[traj_idx++], _costmap_cell_centers);
        if (currr_grade < 0)
          continue;
        if (currr_grade < best_grade)
          continue;
        _best_order.v = v;
        _best_order.w = w;
        _best_traj_idx = traj_idx;
        best_grade = currr_grade;
      } // end loop w
    } // end loop v
    return best_grade;
  } // end best_grade_in_range()

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  std::vector< std::vector<Pt2> > _trajs;

  // control
  Action _current_action;
  //! timer since last computed speed
  vision_utils::Timer _last_speed_age, _last_goal_age;
  //! the current velocities, m/s or rad/s
  int _best_traj_idx;
  SpeedOrder _best_order;
  bool _was_stopped;

  // obstacles
  std::vector<Pt2> _costmap_cell_centers;
  double _laser_thickness;
  double _min_goal_distance, _max_goal_angle;

  // robot parameters
  Pose2 _current_robot_pose;
  double _min_v, _max_v, _max_w, _dv, _dw;
  Pt2 _goal;
  bool _goal_set;

  // simul parameters
  double _speed_recomputation_timeout, _goal_timeout;
  //! the forecast time in seconds
  double _time_pred;
  //! the time step simulation
  double _time_step;

  // GoalDynamicWindowTracker params
  static const unsigned int SPEED_STEPS = 40;
}; // end class GoalDynamicWindowTracker

#endif // _GoalDynamicWindowTracker_H_
