#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>

class PathPlanner
{
private:
  PathPlanner();
  
  // variables
  // static PathPlanner path_planner_;

public:
  static PathPlanner & GetInstance();
  void StraightPathXY(std::vector<double> &next_x_vals, std::vector<double> &next_y_vals, 
    double car_x, double car_y, double car_yaw, double dist_inc, int path_points_count);

  void StraightPathSD(std::vector<double> &next_x_vals, std::vector<double> &next_y_vals, 
    double car_s, double car_d, double car_yaw, double lane_width, 
    const std::vector<double> &map_waypoints_s, const std::vector<double> &map_waypoints_x, const std::vector<double> &map_waypoints_y, 
    double dist_inc, int path_points_count);
};

#endif // PATH_PLANNER