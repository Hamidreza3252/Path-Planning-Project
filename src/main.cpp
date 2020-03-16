#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
// #include "Eigen-3.3/Eigen/Core"
// #include "Eigen-3.3/Eigen/QR"
#include "Eigen/Core"
#include "Eigen/QR"
#include "HandyModules.h"
#include "json.hpp"

#include "PathPlanner.h"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main()
{
  uWS::Hub h;
  PathPlanner &path_planner = PathPlanner::GetInstance();

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  // string map_file_;
  string map_files_[2] = {"../data/highway_map.csv", "data/highway_map.csv"};
  // string current_dir = std::filesystem::current_path();
  // std::cout << current_dir << std::endl;

  for (string map_file : map_files_)
  {
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    std::ifstream in_map_(map_file.c_str(), std::ifstream::in);

    string line;
    while (getline(in_map_, line))
    {
      std::istringstream iss(line);
      double x;
      double y;
      float s;
      float d_x;
      float d_y;
      iss >> x;
      iss >> y;
      iss >> s;
      iss >> d_x;
      iss >> d_y;
      map_waypoints_x.push_back(x);
      map_waypoints_y.push_back(y);
      map_waypoints_s.push_back(s);
      map_waypoints_dx.push_back(d_x);
      map_waypoints_dy.push_back(d_y);
    }

    if (map_waypoints_x.size() > 0)
    {
      break;
    }
  }

  if (map_waypoints_x.size() == 0)
  {
    std::cout << "ERROR: no waypoint data is loaded." << std::endl;
  }

  int car_lane = 1;          // left most lane
  double car_ref_vel = 0.0; // mph
  double speed_limit = 49.5; // mph

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &path_planner,
               &car_lane, &car_ref_vel, &speed_limit](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                        uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = HandyModules::HasData(data);

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner 
          // the size of previous path points is 50 minus (-) the number of points car could go to while driving or simulation 
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msg_json;

          int prev_path_size = previous_path_x.size();

          // sensor fusion check to avoid accident
          if (prev_path_size > 0)
          {
            car_s = end_path_s;
          }

          bool too_close = false;
          double next_car_spped;
          // double speed_limit_ref = speed_limit;

          path_planner.CheckCollision(sensor_fusion, car_s, car_lane, 4.0, prev_path_size, too_close, next_car_spped);

          if (too_close)
          {
            car_ref_vel -= 0.224; // ~5 m/s

            if (car_lane > 0)
            {
              car_lane = 0;
            }
          }
          else
          {
            car_ref_vel += 0.224; // ~5 m/s
          }

          car_ref_vel = MIN(car_ref_vel, speed_limit);
          car_ref_vel = MAX(car_ref_vel, 0.0);
          




          // we will use up to five anchor points to calculate the spline curve 
          vector<double> path_anchor_points_x;
          vector<double> path_anchor_points_y;

          // these references are used to transfor the path coordinates into the local coordinate system attached to the car
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = HandyModules::Deg2Rad(car_yaw);

          // use two points that make the path tanget to the car
          if (prev_path_size < 2)
          {
            double prev_car_x = car_x - cos(ref_yaw);
            double prev_car_y = car_y - sin(ref_yaw);

            path_anchor_points_x.push_back(prev_car_x);
            path_anchor_points_x.push_back(car_x);

            path_anchor_points_y.push_back(prev_car_y);
            path_anchor_points_y.push_back(car_y);
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

          double car_lane_dist = 2.0 + 4.0 * car_lane;

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

          // define the actual (x, y) points that we are going to use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i = 0; i < previous_path_x.size(); ++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // calculate how to break up the spline points so that we travel at our desired reference velocity 
          double target_x = 30.0;
          double target_y = spline(target_x);
          double target_distance = sqrt(target_x * target_x + target_y * target_y);

          double x_add_on = 0.0;

          // the additional points that are added to the end point of the previous path, ref_x and ref_y
          for (int i = 0; i <= 50 - previous_path_x.size(); ++i)
          {
            double points_count = target_distance / (0.02 * car_ref_vel * HandyModules::kMphToMps);

            double point_x = x_add_on + target_x / points_count;
            double point_y = spline(point_x);

            x_add_on = point_x;

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





          



          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          /*
          double dist_inc = 0.5;
          int path_points_count = 50;
          // path_planner.StraightPathXY(next_x_vals, next_y_vals, car_x, car_y, car_yaw, dist_inc, path_points_count);

          path_planner.StraightPathSD(next_x_vals, next_y_vals, car_s, car_d, car_yaw, 4.0,
                                      map_waypoints_s, map_waypoints_x, map_waypoints_y, dist_inc, path_points_count);
          */

          /*          
          for (int i = 0; i < 50; ++i)
          {
            next_x_vals.push_back(car_x + (dist_inc * i) * cos(deg2rad(car_yaw)));
            next_y_vals.push_back(car_y + (dist_inc * i) * sin(deg2rad(car_yaw)));
          }
          */

          msg_json["next_x"] = next_x_vals;
          msg_json["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msg_json.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
