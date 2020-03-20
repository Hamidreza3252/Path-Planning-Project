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

#include "Vehicle.h"
#include "BehaviorPlanner.h"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main()
{
  uWS::Hub h;
  BehaviorPlanner &behavior_planner = BehaviorPlanner::GetInstance();

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

  // int car_lane = 1;
  // double car_ref_vel = 0.0;  // mph
  // double speed_limit = 50.0; // mph
  Vehicle &ego_vehicle = behavior_planner.vehicle;

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &behavior_planner,
               &ego_vehicle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                             uWS::OpCode opCode) {
    /*
  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &behavior_planner,
               &car_ref_vel, &speed_limit](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                        uWS::OpCode opCode) {
    */
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
          /*
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          */

          ego_vehicle.x_ = j[1]["x"];
          ego_vehicle.y_ = j[1]["y"];
          ego_vehicle.s_ = j[1]["s"];
          ego_vehicle.d_ = j[1]["d"];
          ego_vehicle.yaw_ = j[1]["yaw"];
          ego_vehicle.speed_ = j[1]["speed"];

          // Previous path data given to the Planner
          // the size of previous path points is 50 minus (-) the number of points car could go to while driving or simulation
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // define the actual (x, y) points that we are going to use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          // auto sensor_fusion = j[1]["sensor_fusion"];
          ego_vehicle.sensor_fusions_ = (std::vector<std::vector<double>>)j[1]["sensor_fusion"];

          json msg_json;

          // double target_speed = speed_limit - behavior_planner.speed_buffer_;

          // int prev_path_size = previous_path_x.size();

          behavior_planner.HandleHighwayDriving(end_path_s,
                                                previous_path_x, previous_path_y, map_waypoints_s, map_waypoints_x, map_waypoints_y,
                                                next_x_vals, next_y_vals);

          /*
          double car_s;
          // sensor fusion check to avoid accident
          if (prev_path_size > 0)
          {
            car_s = end_path_s;
          }
          else
          {
            car_s = ego_vehicle.s_;
          }

          bool too_close = false;
          double next_car_spped;
          // double speed_limit_ref = speed_limit;r

          // this check will be done with the next car, independent of the state of the ego car
          behavior_planner.CheckCollision(sensor_fusion, car_speed, car_s, 4.0, prev_path_size, too_close, next_car_spped);

          if (too_close)
          {
            target_speed = std::min(next_car_spped, target_speed);
            // std::cout << "safe_speed: " << safe_speed << std::endl;

            // behavior_planner.Decelerate(0.0, car_ref_vel);
            behavior_planner.Decelerate(target_speed, car_ref_vel);

            switch (behavior_planner.state_)
            {
            case FiniteState::kKeepLane:
              if (behavior_planner.lane_ > 0)
              {
                std::cout << "Switch to FiniteState::kPrepareChangeLaneLeft" << std::endl;
                behavior_planner.ChangeState(FiniteState::kPrepareChangeLaneLeft);
              }
              break;
            
            case FiniteState::kPrepareChangeLaneLeft:
              // std::cout << behavior_planner.state_duration_ << std::endl;

              if (behavior_planner.state_duration_ > BehaviorPlanner::kMaxStateWaitTime)
              {
                std::cout << "Switch to FiniteState::kPrepareChangeLaneRight" << std::endl;
                behavior_planner.ChangeState(FiniteState::kPrepareChangeLaneRight);
              }

            default:
              break;
            }

            if (behavior_planner.lane_ > 0)
            {
              behavior_planner.lane_ -= 0;
            }
          }
          else
          {
            behavior_planner.Accelerate(target_speed, car_ref_vel);
          }

          behavior_planner.state_duration_ += 0.02; //sec
          */

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
