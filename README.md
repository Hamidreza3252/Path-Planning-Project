# Path Planning Project


## 1. Objective 
The objective of this project is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. A successful path planner is able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data. 

## 2. Project Description 

The virtual highway used for this simulation provides normal traffic with +-10 MPH of the 50 MPH speed limit. The implemented code uses the following data for localization and path planing: 

- The ego car's localization and sensor fusion data 
- A sparse map list of waypoints around the highway

The car will try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. It should be noted that other cars will try to change lanes too. The ego car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the `6946m` highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## 3. Simulator 

### Details 

1. The Term3 simulator can be download from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).   

2. In the simulator provided, the car uses a perfect controller and will visit every (x,y) point it recieves in the list every 0.02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. 

3. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points we have used so we can have a smooth transition. `previous_path_x`, and `previous_path_y` can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

### The map of the highway is in data/highway_map.txt 
Each waypoint in the list contains  `[x, y, s, dx, dy]` values. `x` and `y` are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the `dx` and `dy` values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet `s` value, distance along the road, goes from `0` to `6945.554`. 

### Simulator's Data to The Path Planning Module 

#### Main car's localization Data (No Noise)

- `["x"]` The car's x position in map coordinates
- `["y"]` The car's y position in map coordinates
- `["s"]` The car's s position in frenet coordinates
- `["d"]` The car's d position in frenet coordinates
- `["yaw"]` The car's yaw angle in the map
- `["speed"]` The car's speed in MPH 

#### Previous path data given to the Planner

The `x` and `y` coordinates of the previous path that were not processed (were not passed by the car) since the last time step. 

`["previous_path_x"]` The previous list of x points previously given to the simulator
`["previous_path_y"]` The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

`["end_path_s"]` The previous list's last point's frenet s value
`["end_path_d"]` The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

`["sensor_fusion"]` A 2d vector of cars and then that car's (car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates). 

## 4. The Code and Approach 

Two main class handing highway driving is `BehaviorPlanner` with the following details.  

### - `BehaviorPlanner` Class (Singleton) 

#### The main variables 
- `vehicle`: An object to the ego vehicle 
- `front_cars_speeds_`: A double vector keeping the velocity of the cars that are right in front of the ego vehicle 
- `behind_cars_speeds_`: A double vector keeping the velocity of the cars that are right behind of the ego vehicle 
- `front_cars_s_poses_`: A double vector keeping the s coordinate of the cars that are right in front of the ego vehicle 
- `behind_cars_s_poses_`: A double vector keeping the s coordinate of the cars that are right behind of the ego vehicle 
- `lane_speed_costs_`: A double vector keeping the inefficiency costs of the lanes 

#### The main functions 
- **`HandleHighwayDriving`**: This function is the main function called from the `main`, receiving all the information from the simulator. The main outputs of this function are x and y values of the path points (`next_x_vals`, `next_y_vals`) that the car should pass from every 0.02 sec. 
- **`UpdateLanesInfo`**: This function uses the current position of the car and other input data to update all the information related to the lanes. This infomation of the lanes will be used for the necessary actions in the next step. The main parameters that `UpdateLanesInfo` updates include: 
  - `front_cars_speeds_` is used to calculate the inefficiency costs of the lanes as well as avoid collision with the cars in front of the ego car 
  - `front_cars_s_poses_` is used to check the distance between the ego car and other cars right in front of the ego car to avoid collision while changing lane 
  - `behind_cars_speeds_` is used to check the velocity of the cars right behind the ego car to avoid collision while changing lane 
  - `behind_cars_s_poses_` is used to check the distance between the ego car and other cars right behind the ego car to avoid collision while changing lane 
  - `acc_value` is the acceleration (+/-) calculated based on the disctance of the ego car from the front car. This is used to accelerate and decelerate whenever needed 
  - `too_close` is a boolean flag which is raised when the car gets close to the front car to decelerate, avoiding collision 
  - `front_car_speed` is set to the the velocity of the front car. This parameter is used to as the lower bound of the velocity while deceleration to avoid collision 
- **`UpdateState`**: This fucntion uses all the lane-related information, which are updated in `UpdateLanesInfo` function, to update the state of the `vehicle` object. These states are `kKeepLane`, `kChangeLaneLeft`, `kChangeLaneRight`, and `kAvoidCollision`. This function sets the states using the following instructions: 
  - Sorting `lane_speed_costs_` to see which lane is more efficient to take 
  - Suggesting the efficient lane to the ego car 
  - Checking if the suggested lane is safe to take or not. If it is safe, it changes the lane of the vehicle and the path generation will automatically take it into account. Safety is checked usind the estimated distance of the ego car with respect to the front and behind cars in the same and the target lane. 
- `**GeneratePath**`: This function generates the points of the path of the car to be taken at every 0.02 sec. In order to generate a smooth path, it stiches the points of the previous path, that were not passed by the car within the last time step, with the new points that are generated in the current time step. The path points are generated using the states of the car and the map waypoints of the highway. 

### Sample Video of the Result 
[![Path Planning Sample Video](https://img.youtube.com/vi/wr7z2fXcacw/0.jpg)](https://www.youtube.com/watch?v=wr7z2fXcacw)

## 5. Dependencies

* cmake >= 3.5
* gcc/g++ >= 5.4
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## 6. Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html). 

