#ifndef _GoalDynamicWindowTracker_H_
#define _GoalDynamicWindowTracker_H_

#include <vector>
#include <math.h>
#include "utils.h"
#include "timer.h"

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
  struct Pose2 {
    Pt2 position;
    double yaw;
  };
  
  //////////////////////////////////////////////////////////////////////////////

  GoalDynamicWindowTracker() {
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
    _min_obstacle_distance = utils::clamp(fabs(min_obstacle_distance), .01, 10.);
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
    utils::make_trajectory(// get_best_element().element.v,
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
    if (!_goal_set) {
      stop_robot();
      best_speed_lin = best_speed_ang =  0;
      return true;
    }

    bool want_recompute_speed = false;
    // recompute speeds if the last ones are too old
    if (!want_recompute_speed && _last_speed_recomputation_timer.getTimeSeconds() > _speed_recomputation_timeout) {
      printf("speed_timeout\n");
      want_recompute_speed = true;
    }
    // check if there will be a collision soon with the elected speeds
    if (!want_recompute_speed &&
        trajectory_grade(_curr_order.v, _curr_order.w, _costmap_cell_centers) == false) {
      printf("coming_collision !\n");
      want_recompute_speed = true;
    } // end

    if (want_recompute_speed) {
      bool new_speeds_found = false;
      _last_speed_recomputation_timer.reset();
#if 0
      int nb_tries = 0;
      // choose a random speed and check it enables at least 1 second of movement
      while (!new_speeds_found) {
        // authorize on place rotations only after having tried many times
        double curr_min_v = (nb_tries < .4 * MAX_TRIES ? _min_v : _min_v / 2);
        if (1.f * nb_tries > .5 * MAX_TRIES )
          curr_min_v = -.1; // authorize backward moves in extreme cases
        _curr_order.v = curr_min_v + drand48() * (_max_v - curr_min_v);
        _curr_order.w = drand48() * 2 * _max_w - _max_w;
        double currr_grade = trajectory_grade(_curr_order.v, _curr_order.w, _costmap_cell_centers);
        new_speeds_found = (curr_grade > 0);
        if (nb_tries++ > MAX_TRIES)
          break;
      } // end while (!new_speeds_found)
#else
      double best_grade = best_grade_in_range(_min_v, _max_v, _max_w);
      if (best_grade < 0) // extreme case -> go backward
        best_grade = best_grade_in_range(-(_min_v+_max_v)*.5, -_min_v, _max_w);
      new_speeds_found = (best_grade > 0);
#endif

      if (!new_speeds_found) {
        printf("The robot is stuck! Stopping motion and exiting.\n");
        stop_robot();
        return false;
      } // end not new_speeds_found

      DEBUG_PRINT("Found a suitable couple of speeds in %g ms and %i tries: "
                  "_v:%g, _w:%g\n",
                  _last_speed_recomputation_timer.getTimeMilliseconds(), nb_tries,
                  _curr_order.v,
                  _curr_order.w);
      _last_speed_recomputation_timer.reset();
    } // end if (want_recompute_speed)

    // publish the computed speed
    DEBUG_PRINT("DynamicWindow: Publishing _v:%g, _w:%g\n",
                _curr_order.v, _curr_order.w);
    best_speed_lin = _curr_order.v;
    best_speed_ang = _curr_order.w;
    return true;
  } // end recompute_speeds()
  
  
protected:  


  ////////////////////////////////////////////////////////////////////////////////
  
  inline void set_speed(const SpeedOrder & new_speed) {
    DEBUG_PRINT("set_speed(lin:%g, ang:%g)",
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
    utils::make_trajectory(v, w, _traj_buffer, _time_pred, _time_step, 0, 0, 0);
    // find if there might be a collision
    double obs_dist = utils::vectors_dist_thres(_traj_buffer, laser_xy,
                                                _min_obstacle_distance);
    if (obs_dist < 0)
      return -1;
    // return inv of dist to goal - the higher the better
    return 1. / (utils::vectors_dist(_goal, _traj_buffer) + obs_dist / 5);
  }

  //////////////////////////////////////////////////////////////////////////////

  inline double best_grade_in_range(double min_v, double max_v, double max_w) {
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
  //! timer since last computed speed
  Timer _last_speed_recomputation_timer;
  //! the current velocities, m/s or rad/s
  SpeedOrder _curr_order;
  bool _was_stopped;
  
  // obstacles
  std::vector<Pt2> _costmap_cell_centers;
  double _min_obstacle_distance;
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
