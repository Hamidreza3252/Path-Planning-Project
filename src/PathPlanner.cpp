#include "PathPlanner.h"

#include <math.h>
#include "HandyModules.h"
#include "spline.h"

PathPlanner::PathPlanner()
{
}
// --------------------------------------------------------------------------------------------------------------------

PathPlanner &PathPlanner::GetInstance()
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
    next_x_vals.push_back(car_x + (dist_inc * i) * cos(HandyModules::Deg2Rad(car_yaw)));
    next_y_vals.push_back(car_y + (dist_inc * i) * sin(HandyModules::Deg2Rad(car_yaw)));
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
    car_next_s = car_s + (i + 1) * dist_inc;
    car_next_d = car_d;

    car_xy = HandyModules::GetXY(car_next_s, car_next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

    next_x_vals.push_back(car_xy[0]);
    next_y_vals.push_back(car_xy[1]);
  }
}
// --------------------------------------------------------------------------------------------------------------------

void PathPlanner::CheckCollision(const std::vector<std::vector<double>> &sensor_fusions, double ego_car_s, int lane, double lane_width, int prev_path_size,
                                 bool &too_close, double &next_car_speed)
{
  // bool too_close = false;

  too_close = false;
  next_car_speed = 0.0;

  double prev_next_car_speed = MAXFLOAT;

  // find ref_v to use
  // for (int i = 0; i < sensor_fusion.size(); ++i)
  for (auto &sensor_data : sensor_fusions)
  {
    // float d = sensor_fusion[i][6];
    float d = sensor_data[6];
    float lane_width_2 = 0.5 * lane_width;

    // car is in my lane
    if (d < (lane_width_2 + lane_width * lane + lane_width_2) &&
        d > (lane_width_2 + lane_width * lane - lane_width_2))
    {
      double vx = sensor_data[3];
      double vy = sensor_data[4];

      next_car_speed = sqrt(vx * vx + vy * vy);

      if (next_car_speed > prev_next_car_speed)
      {
        next_car_speed = prev_next_car_speed;
        continue;
      }

      prev_next_car_speed = next_car_speed;

      double next_car_s = sensor_data[5];

      // predict where the car will be in future
      next_car_s += prev_path_size * 0.02 * next_car_speed;

      if ((next_car_s > ego_car_s) && (next_car_s - ego_car_s) < 30.0)
      {
        // 1. lower the ref_vel
        // 2. flag to try to change lane

        // car_ref_vel = MIN(car_ref_vel, next_car_speed);

        // car_ref_vel = 29.5; // mph
        too_close = true;
      }
    }
  }
}
// --------------------------------------------------------------------------------------------------------------------
