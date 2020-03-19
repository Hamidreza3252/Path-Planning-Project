#include "BehaviorPlanner.h"

#include <iostream>
#include <math.h>
#include "HandyModules.h"
#include "spline.h"

// --------------------------------------------------------------------------------------------------------------------

BehaviorPlanner::BehaviorPlanner()
{
  prev_state_ = FiniteState::kKeepLane;
  state_ = FiniteState::kKeepLane;
  vehicle.lane_ = 1; // middle lane
  state_duration_ = 0.0;
  timer_tracker_2_sec_ = 0.0;
  speed_buffer_ = 0.2; //mph
  lanes_count = 3;

  for (int i = 0; i < lanes_count; ++i)
  {
    lane_speeds_.push_back(0.0); // mph
  }
}
// --------------------------------------------------------------------------------------------------------------------

BehaviorPlanner &BehaviorPlanner::GetInstance()
{
  static BehaviorPlanner path_planner;

  return path_planner;
}
// --------------------------------------------------------------------------------------------------------------------

void BehaviorPlanner::ChangeState(FiniteState new_state)
{
  if (state_ == new_state)
  {
    return;
  }

  prev_state_ = state_;
  state_ = new_state;
  // state_duration_ = 0.0;

  std::cout << "Changing state: " << state_ << std::endl;
}
// --------------------------------------------------------------------------------------------------------------------

void BehaviorPlanner::StraightPathXY(std::vector<double> &next_x_vals, std::vector<double> &next_y_vals,
                                     double car_x, double car_y, double car_yaw, double dist_inc, int path_points_count)
{
  for (int i = 0; i < path_points_count; ++i)
  {
    next_x_vals.push_back(car_x + (dist_inc * i) * cos(HandyModules::Deg2Rad(car_yaw)));
    next_y_vals.push_back(car_y + (dist_inc * i) * sin(HandyModules::Deg2Rad(car_yaw)));
  }
}
// --------------------------------------------------------------------------------------------------------------------

void BehaviorPlanner::StraightPathSD(std::vector<double> &next_x_vals, std::vector<double> &next_y_vals,
                                     double car_s, double car_d, double car_yaw, double lane_width,
                                     const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
                                     double dist_inc, int path_points_count)
{
  double car_next_s;
  double car_next_d;
  double car_x;
  double car_y;
  vector<double> car_xy;

  for (int i = 0; i < path_points_count; ++i)
  {
    car_next_s = car_s + (i + 1) * dist_inc;
    car_next_d = car_d;

    car_xy = HandyModules::GetXY(car_next_s, car_next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

    next_x_vals.push_back(car_xy[0]);
    next_y_vals.push_back(car_xy[1]);
  }
}
// --------------------------------------------------------------------------------------------------------------------

void BehaviorPlanner::HandleHighwayDriving(double end_path_s, double speed_limit,
                                           const vector<double> &previous_path_x, const vector<double> &previous_path_y,
                                           const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
                                           vector<double> &next_x_vals, vector<double> &next_y_vals)
{
  bool too_close = false;
  double next_car_spped;
  double target_speed = speed_limit - speed_buffer_;
  // double car_ref_vel = 0.0;
  double car_s;
  // double speed_limit_ref = speed_limit;r

  int prev_path_size = previous_path_x.size();

  if (prev_path_size > 0)
  {
    car_s = end_path_s;
  }
  else
  {
    car_s = vehicle.s_;
  }

  double speed_cost;
  double left_lane_inefficiency_cost = 1.0;
  double right_lane_inefficiency_cost = 1.0;
  double ego_lane_inefficiency_cost = 1.0;
  double min_cost;
  SpeedCostReductionAction recommended_action;

  // this check will be done with the next car, independent of the state of the ego car
  CheckCollision(car_s, 4.0, prev_path_size, too_close, next_car_spped);

  // safety critical
  if (too_close)
  {
    // std::cout << "vehicle.speed_: " << vehicle.speed_ << std::endl;

    target_speed = std::min(next_car_spped, target_speed);
    // std::cout << "safe_speed: " << safe_speed << std::endl;

    Decelerate(target_speed, vehicle.action_speed_);
  }
  else
  {
    Accelerate(target_speed, vehicle.action_speed_);
  }

  switch (state_)
  {
  case FiniteState::kChangeLaneLeft:
    vehicle.lane_ -= 1;

    ChangeState(FiniteState::kKeepLane);
    break;

  default:
    break;
  }

  // every 2 secs
  if (timer_tracker_2_sec_ > 2.0)
  {
    timer_tracker_2_sec_ = 0.0;

    // SpeedCost(vehicle.speed_, speed_limit, speed_cost, recommended_action);

    // chech if we should plan to change lane or we should keep the lane

    InefficiencyCost(target_speed, vehicle.lane_, ego_lane_inefficiency_cost);
    std::cout << "ego_lane_inefficiency_cost: " << ego_lane_inefficiency_cost << std::endl;

    if (vehicle.lane_ > 0)
    {
      InefficiencyCost(target_speed, vehicle.lane_ - 1, left_lane_inefficiency_cost);

      if (state_duration_ > 10.0 && state_duration_ < 10.03)
      {
        left_lane_inefficiency_cost = 0.5;
      }

      std::cout << "left_lane_inefficiency_cost: " << left_lane_inefficiency_cost << std::endl;
    }

    min_cost = std::min(ego_lane_inefficiency_cost, left_lane_inefficiency_cost);

    if (vehicle.lane_ < lanes_count - 1)
    {
      InefficiencyCost(target_speed, vehicle.lane_ + 1, right_lane_inefficiency_cost);
      std::cout << "right_lane_inefficiency_cost: " << right_lane_inefficiency_cost << std::endl;
    }

    min_cost = std::min(min_cost, right_lane_inefficiency_cost);

    if (min_cost == ego_lane_inefficiency_cost)
    {
      ChangeState(FiniteState::kKeepLane);
    }
    else if (min_cost == left_lane_inefficiency_cost)
    {
      // ChangeState(FiniteState::kPrepareChangeLaneLeft);
      ChangeState(FiniteState::kChangeLaneLeft);
    }
    else
    {
      // ChangeState(FiniteState::kPrepareChangeLaneRight);
      ChangeState(FiniteState::kChangeLaneRight);
    }

    std::cout << "state_duration_: " << state_duration_ << std::endl;
    std::cout << std::endl;
  }

  state_duration_ += 0.02; //sec

  GeneratePath(car_s, prev_path_size,
               previous_path_x, previous_path_y, map_waypoints_s, map_waypoints_x, map_waypoints_y,
               next_x_vals, next_y_vals);
}
// --------------------------------------------------------------------------------------------------------------------

/**
 * Chekcing for collision with the front car in the same lane
 * @sensor_fusions: all sensor fusion data
 * @ego_car_s: the s value of the ego car
 * @lane: the lane of the ego car
 * @lane_width: the lane width 
 * @prev_path_size: the previous path size, the ones that were not passed through in the previous time step
 * @too_close: (output) the flag to show if the ego car is too close to the next car in the same lane
 * @next_car_speed: (output) the speed of the next car in the same lane
 */

void BehaviorPlanner::CheckCollision(double car_s, double lane_width, int prev_path_size, bool &too_close, double &next_car_speed)
{

  // bool too_close = false;

  too_close = false;
  next_car_speed = 0.0;

  // double prev_next_car_speed = MAXFLOAT;
  float lane_width_2 = 0.5 * lane_width;

  // find ref_v to use
  // for (int i = 0; i < sensor_fusion.size(); ++i)
  for (auto &sensor_data : vehicle.sensor_fusions_)
  {
    // float d = sensor_fusion[i][6];
    float d = sensor_data[6];

    // car is in my lane
    if (d < (lane_width_2 + lane_width * vehicle.lane_ + lane_width_2) &&
        d > (lane_width_2 + lane_width * vehicle.lane_ - lane_width_2))
    {
      double vx = sensor_data[3];
      double vy = sensor_data[4];

      next_car_speed = sqrt(vx * vx + vy * vy);

      double next_car_s = sensor_data[5];

      // predict where the car will be in future
      next_car_s += prev_path_size * 0.02 * next_car_speed;

      if ((next_car_s > car_s) && (next_car_s - car_s) < 30.0)
      {
        next_car_speed = vehicle.speed_ - next_car_speed;
        // std::cout << "next_car_speed" << next_car_speed << std::endl;

        // 1. lower the ref_vel
        // 2. flag to try to change lane

        // car_ref_vel = MIN(car_ref_vel, next_car_speed);

        // car_ref_vel = 29.5; // mph
        // if (next_car_speed > prev_next_car_speed)
        //{
        // next_car_speed = prev_next_car_speed;
        //continue;
        //}

        // prev_next_car_speed = next_car_speed;
        too_close = true;
      }
    }
  }
}
// --------------------------------------------------------------------------------------------------------------------

void BehaviorPlanner::UpdateLanesSpeeds(void)
{
  // lane_speeds_
}

// --------------------------------------------------------------------------------------------------------------------

void BehaviorPlanner::GeneratePath(double car_s, int prev_path_size,
                                   const vector<double> &previous_path_x, const vector<double> &previous_path_y,
                                   const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
                                   vector<double> &next_x_vals, vector<double> &next_y_vals)
{
  // we will use up to five anchor points to calculate the spline curve
  vector<double> path_anchor_points_x;
  vector<double> path_anchor_points_y;

  // these references are used to transfor the path coordinates into the local coordinate system attached to the car
  double ref_x = vehicle.x_;
  double ref_y = vehicle.y_;
  double ref_yaw = HandyModules::Deg2Rad(vehicle.yaw_);

  // use two points that make the path tanget to the car
  if (prev_path_size < 2)
  {
    double prev_car_x = vehicle.x_ - cos(ref_yaw);
    double prev_car_y = vehicle.y_ - sin(ref_yaw);

    path_anchor_points_x.push_back(prev_car_x);
    path_anchor_points_x.push_back(vehicle.x_);

    path_anchor_points_y.push_back(prev_car_y);
    path_anchor_points_y.push_back(vehicle.y_);
  }
  // use the pervious path's end points as starting references to generate a smooth path
  else
  {
    // redefine reference state on previous path end point
    ref_x = previous_path_x[prev_path_size - 1];
    ref_y = previous_path_y[prev_path_size - 1];

    double ref_x_prev = previous_path_x[prev_path_size - 2];
    double ref_y_prev = previous_path_y[prev_path_size - 2];

    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    // use two points that make the path tangent to the previous path's end point
    path_anchor_points_x.push_back(ref_x_prev);
    path_anchor_points_x.push_back(ref_x);

    path_anchor_points_y.push_back(ref_y_prev);
    path_anchor_points_y.push_back(ref_y);
  }

  // In Frenet add evenly 30m spaced points ahead of the startng reference

  double car_lane_dist = 2.0 + 4.0 * vehicle.lane_;

  vector<double> next_wp_xy_0 = HandyModules::GetXY(car_s + 30.0, car_lane_dist, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp_xy_1 = HandyModules::GetXY(car_s + 60.0, car_lane_dist, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp_xy_2 = HandyModules::GetXY(car_s + 90.0, car_lane_dist, map_waypoints_s, map_waypoints_x, map_waypoints_y);

  path_anchor_points_x.push_back(next_wp_xy_0[0]);
  path_anchor_points_x.push_back(next_wp_xy_1[0]);
  path_anchor_points_x.push_back(next_wp_xy_2[0]);

  path_anchor_points_y.push_back(next_wp_xy_0[1]);
  path_anchor_points_y.push_back(next_wp_xy_1[1]);
  path_anchor_points_y.push_back(next_wp_xy_2[1]);

  // shifting the car reference angle to 0 degree, with respect to x & y of either the car or the end point of previous path
  for (int i = 0; i < path_anchor_points_x.size(); ++i)
  {
    double shifted_x = path_anchor_points_x[i] - ref_x;
    double shifted_y = path_anchor_points_y[i] - ref_y;

    path_anchor_points_x[i] = (shifted_x * cos(0.0 - ref_yaw) - shifted_y * sin(0.0 - ref_yaw));
    path_anchor_points_y[i] = (shifted_x * sin(0.0 - ref_yaw) + shifted_y * cos(0.0 - ref_yaw));
  }

  // create an spline instance
  tk::spline spline;

  // set the (x, y) points to the spline
  // note that the points are transformed into the local coordinate system located at ref_x and ref_y
  spline.set_points(path_anchor_points_x, path_anchor_points_y);

  for (int i = 0; i < previous_path_x.size(); ++i)
  {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  int segments_count = 70 - previous_path_x.size();

  // calculate how to break up the spline points so that we travel at our desired reference velocity
  double target_x_segment = (0.02 * vehicle.action_speed_ * HandyModules::kMphToMps);
  double target_x = target_x_segment * segments_count;
  double target_y = spline(target_x);
  double target_distance = sqrt(target_x * target_x + target_y * target_y);
  // double segment_length = target_distance / segments_count;
  // the additional points that are added to the end point of the previous path, ref_x and ref_y
  for (int i = 0; i <= segments_count; ++i)
  {
    double point_x = (i + 1) * target_x_segment;
    double point_y = spline(point_x);

    double temp_point_x = point_x;
    double temp_point_y = point_y;

    // rotate back to normal after rotating it earlier (global coordinate system)
    point_x = temp_point_x * cos(ref_yaw) - temp_point_y * sin(ref_yaw);
    point_y = temp_point_x * sin(ref_yaw) + temp_point_y * cos(ref_yaw);

    point_x += ref_x;
    point_y += ref_y;

    next_x_vals.push_back(point_x);
    next_y_vals.push_back(point_y);
  }

  // std::cout << "ref vel" << vehicle.action_speed_ << std::endl;
}
// --------------------------------------------------------------------------------------------------------------------

/**
 * To reduce the ego car's refrence velocisty by an increment
 * @car_ref_vel: (input/output) the velocity of the ego car
 */

void BehaviorPlanner::Decelerate(double lower_bound_vel, double &car_ref_vel)
{
  // 0.224 ~ 5 m/s
  double decelerate_value = 0.224 / 1.5;
  car_ref_vel = std::max<double>(car_ref_vel - decelerate_value, lower_bound_vel);
}
// --------------------------------------------------------------------------------------------------------------------

/**
 * To reduce the ego car's refrence velocisty by an increment
 * @car_ref_vel: (input/output) the velocity of the ego car
 */

void BehaviorPlanner::Accelerate(double upper_bound_vel, double &car_ref_vel)
{
  // 0.224 ~ 5 m/s
  double accelerate_value = 0.224 / 1.5;
  car_ref_vel = std::min<double>(car_ref_vel + accelerate_value, upper_bound_vel);
}
// --------------------------------------------------------------------------------------------------------------------

void BehaviorPlanner::SpeedCost(double ego_car_speed, double speed_limit, double &speed_cost, SpeedCostReductionAction &recommended_action)
{
  double safe_speed_limit = speed_limit - speed_buffer_;
  speed_cost = 1.0;

  if (ego_car_speed <= safe_speed_limit)
  {
    speed_cost = kStopCost * (safe_speed_limit - ego_car_speed) / safe_speed_limit;
    recommended_action = SpeedCostReductionAction::kAccelerate;
  }
  else if (ego_car_speed > safe_speed_limit && ego_car_speed < speed_limit)
  {
    speed_cost = (ego_car_speed - safe_speed_limit) / speed_buffer_;
    recommended_action = SpeedCostReductionAction::kDecelerate;
  }
  else
  {
    speed_cost = 1.0;
    recommended_action = SpeedCostReductionAction::kDecelerate;
  }
}
// --------------------------------------------------------------------------------------------------------------------

double BehaviorPlanner::GoalDistanceCost(int goal_lane, int intended_lane, int final_lane, double distance_to_goal)
{
  // The cost increases with both the distance of intended lane from the goal
  //   and the distance of the final lane from the goal. The cost of being out
  //   of the goal lane also becomes larger as the vehicle approaches the goal.

  double goal_distance_cost;

  int delta_d = 2 * goal_lane - intended_lane - final_lane;
  goal_distance_cost = 1 - exp(-(std::abs(delta_d) / distance_to_goal));

  return goal_distance_cost;
}
// --------------------------------------------------------------------------------------------------------------------

// void BehaviorPlanner::InefficiencyCost(double target_speed, int intended_lane, int final_lane, double &inefficiency_cost)
void BehaviorPlanner::InefficiencyCost(double target_speed, int intended_lane, double &inefficiency_cost)
{
  // Cost becomes higher for trajectories with intended lane and final lane
  //   that have traffic slower than target_speed.

  assert(target_speed > 0.0);

  double speed_intended = lane_speeds_[intended_lane];
  // double speed_final = lane_speeds_[final_lane];
  // double inefficiency_cost;

  // inefficiency_cost = (2.0 * target_speed - speed_intended - speed_final) / (2.0 * target_speed);
  inefficiency_cost = (target_speed - speed_intended) / (target_speed);

  // return inefficiency_cost;
}
// --------------------------------------------------------------------------------------------------------------------
