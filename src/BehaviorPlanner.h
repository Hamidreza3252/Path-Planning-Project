#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include "Vehicle.h"

using std::vector;

enum FiniteState
{
  kKeepLane = 0,
  kPrepareChangeLaneLeft,
  kChangeLaneLeft,
  kPrepareChangeLaneRight,
  kChangeLaneRight,
};

class BehaviorPlanner
{
private:
  BehaviorPlanner();

  // variables
  // static PathPlanner path_planner_;

public:
  // Variables decleration
  static constexpr double kMaxStateWaitTime = 5.0; // sec
  static constexpr double kStopCost = 0.9;
  Vehicle vehicle;
  FiniteState state_;
  double state_duration_;
  double speed_buffer_;
  // int lane_;

  // Functions decleration
  static BehaviorPlanner &GetInstance();
  void ChangeState(FiniteState new_state);
  void StraightPathXY(vector<double> &next_x_vals, vector<double> &next_y_vals,
                      double car_x, double car_y, double car_yaw, double dist_inc, int path_points_count);

  void StraightPathSD(vector<double> &next_x_vals, vector<double> &next_y_vals,
                      double car_s, double car_d, double car_yaw, double lane_width,
                      const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
                      double dist_inc, int path_points_count);
  void HandleHighwayDriving(double end_path_s, double speed_limit,
                            const vector<double> &previous_path_x, const vector<double> &previous_path_y,
                            const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
                            vector<double> &next_x_vals, vector<double> &next_y_vals);

private:
  void CheckCollision(const std::vector<std::vector<double>> &sensor_fusions, double ego_car_speed, double ego_car_s, double lane_width,
                      int prev_path_size, bool &too_close, double &next_car_speed);
  void GeneratePath(double car_s, int prev_path_size,
                    const vector<double> &previous_path_x, const vector<double> &previous_path_y,
                    const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
                    vector<double> &next_x_vals, vector<double> &next_y_vals);

  void Decelerate(double lower_bound_vel, double &car_ref_vel);
  void Accelerate(double upper_bound_vel, double &car_ref_vel);
  void SpeedCost(double ego_car_speed, double target_speed, double speed_limit, double &speed_cost);
  void GoalDistanceCost(int goal_lane, int intended_lane, int final_lane, double distance_to_goal, double &goal_distance_cost);
  void InefficiencyCost(int target_speed, int intended_lane, int final_lane, const std::vector<double> &lane_speeds, double &inefficiency_cost);
};

#endif // PATH_PLANNER