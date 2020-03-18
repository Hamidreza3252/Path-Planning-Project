#ifndef VEHICLE_H
#define VEHICLE_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
// using std::string;
// using std::vector;

class Vehicle
{
private:
public:
  // Functions decleration
  Vehicle();
  ~Vehicle();

  // Variables decleration
  double x_;
  double y_;
  double s_;
  double d_;
  double yaw_;
  double speed_;
  double action_speed_;
  int lane_;

  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  std::vector<std::vector <double>> sensor_fusions_;
};

#endif // VEHICLE_H