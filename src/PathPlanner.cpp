#include "PathPlanner.h"

#include <math.h>
#include "HandyModules.h"

PathPlanner::PathPlanner()
{

}
// --------------------------------------------------------------------------------------------------------------------

PathPlanner & PathPlanner::GetInstance()
{
  static PathPlanner path_planner;

  return path_planner;
}
// --------------------------------------------------------------------------------------------------------------------

void PathPlanner::StraightPathXY(std::vector<double> &next_x_vals, std::vector<double> &next_y_vals, 
  double car_x, double car_y, double car_yaw, double dist_inc, int path_points_count)
{
  for (int i = 0; i < path_points_count; ++i)
  {
    next_x_vals.push_back(car_x + (dist_inc * i) * cos(HandyModules::deg2rad(car_yaw)));
    next_y_vals.push_back(car_y + (dist_inc * i) * sin(HandyModules::deg2rad(car_yaw)));
  }
}
// --------------------------------------------------------------------------------------------------------------------

void PathPlanner::StraightPathSD(std::vector<double> &next_x_vals, std::vector<double> &next_y_vals, 
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
    car_next_s = car_s + (dist_inc * (i + 1));
    car_next_d = car_d;

    car_xy = HandyModules::getXY(car_next_s, car_next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

    next_x_vals.push_back(car_xy[0]);
    next_y_vals.push_back(car_xy[1]);
  }
}
// --------------------------------------------------------------------------------------------------------------------
