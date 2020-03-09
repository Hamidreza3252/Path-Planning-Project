#ifndef HANDY_MODULES_H
#define HANDY_MODULES_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

class HandyModules
{
private:

public:
  // Checks if the SocketIO event has JSON data.
  // If there is data the JSON object in string format will be returned,
  //   else the empty string "" will be returned.
  static string HasData(const string &s);

  // Helper functions related to waypoints and converting from XY to Frenet or vice versa

  // For converting back and forth between radians and degrees.
  static constexpr double pi() { return M_PI; }
  static double Deg2Rad(double x) { return x * pi() / 180; }
  static double Rad2Deg(double x) { return x * 180 / pi(); }

  // Calculate distance between two points
  static double Distance(double x1, double y1, double x2, double y2);

  // Calculate closest waypoint to current x, y position
  static int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                      const vector<double> &maps_y);

  // Returns next waypoint of the closest waypoint
  static int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                   const vector<double> &maps_y);

  // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  static vector<double> GetFrenet(double x, double y, double theta,
                           const vector<double> &maps_x,
                           const vector<double> &maps_y);

  // Transform from Frenet s,d coordinates to Cartesian x,y
  static vector<double> GetXY(double s, double d, const vector<double> &maps_s,
                       const vector<double> &maps_x,
                       const vector<double> &maps_y);
};

#endif // HANDY_MODULES_H
