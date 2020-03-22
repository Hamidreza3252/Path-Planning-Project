#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include "Vehicle.h"

enum FiniteState
{
  kKeepLane = 0,
  kPrepareChangeLaneLeft,
  kChangeLaneLeft,
  kPrepareChangeLaneRight,
  kChangeLaneRight,
  kAvoidCollision
};

enum SpeedCostReductionAction
{
  // kMaintainVel = 0,
  kAccelerate = 0,
  kDecelerate
};

class BehaviorPlanner
{
private:
  BehaviorPlanner(int lanes_count);

  // variables
  // static PathPlanner path_planner_;

public:
  // Variables decleration
  static constexpr double kMaxStateWaitTime = 5.0; // sec
  static constexpr double kStopCost = 0.9;
  // static constexpr double kDefaultInefficiencyCost = 1.0;
  Vehicle vehicle;
  FiniteState state_;
  FiniteState prev_state_;
  double state_duration_;
  double timer_tracker_2_sec_;
  double speed_limit_;
  double speed_buffer_;
  double lane_width_;
  std::vector<double> front_cars_speeds_;
  std::vector<double> behind_cars_speeds_;
  std::vector<double> front_cars_s_poses_;
  std::vector<double> behind_cars_s_poses_;
  std::vector<double> lane_speed_costs_;
  // std::vector<double> left_lane_change_costs_;
  // std::vector<double> right_lane_change_costs_;
  // int lanes_count;
  // int lane_;

  // Functions decleration
  static BehaviorPlanner &GetInstance();
  void StraightPathXY(std::vector<double> &next_x_vals, std::vector<double> &next_y_vals,
                      double car_x, double car_y, double car_yaw, double dist_inc, int path_points_count);

  void StraightPathSD(std::vector<double> &next_x_vals, std::vector<double> &next_y_vals,
                      double car_s, double car_d, double car_yaw, double lane_width,
                      const std::vector<double> &map_waypoints_s, const std::vector<double> &map_waypoints_x, const std::vector<double> &map_waypoints_y,
                      double dist_inc, int path_points_count);
  void HandleHighwayDriving(double end_path_s, 
                            const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y,
                            const std::vector<double> &map_waypoints_s, const std::vector<double> &map_waypoints_x, const std::vector<double> &map_waypoints_y,
                            std::vector<double> &next_x_vals, std::vector<double> &next_y_vals);
  void ChangeState(FiniteState &new_state, double car_s);

private:
  void UpdateLanesInfo(double car_s, int prev_path_size, FiniteState &next_state, double &front_car_speed);
  void CheckCollision(double car_s, int prev_path_size, bool &too_close, double &next_car_speed);

  void GeneratePath(double car_s, int prev_path_size,
                    const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y,
                    const std::vector<double> &map_waypoints_s, const std::vector<double> &map_waypoints_x, const std::vector<double> &map_waypoints_y,
                    std::vector<double> &next_x_vals, std::vector<double> &next_y_vals);

  void Decelerate(double lower_bound_vel, double &car_ref_vel);
  void Accelerate(double upper_bound_vel, double &car_ref_vel);

  void SpeedCost(double ego_car_speed, double speed_limit, double &speed_cost, SpeedCostReductionAction &recommended_action);
  double GoalDistanceCost(int goal_lane, int intended_lane, int final_lane, double distance_to_goal);
  // void InefficiencyCost(double target_speed, int intended_lane, int final_lane, double &inefficiency_cost);
  void InefficiencyCost(double target_speed, int intended_lane, double &inefficiency_cost);
};

#endif // PATH_PLANNER