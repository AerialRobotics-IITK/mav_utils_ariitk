// Small example of using the GeographicLib::Geodesic class
#include <ros/ros.h>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <iostream>

using namespace std;
using namespace GeographicLib;
int main(int argc, char **argv) {
  ros::init(argc, argv, "tester");
  // check whether multiplication with scale is needed

  const Geodesic &geod = Geodesic::WGS84();
  // Distance from JFK to LHR
  double lat1 = 50,
         lon1 = 50,                  // JFK Airport
      lat2 = 50.001, lon2 = 50.001;  // LHR Airport
  double s12;
  geod.Inverse(lat1, lon1, lat2, lon2, s12);
  cout << s12 << " m\n";

  try {
    // See also example-GeoCoords.cpp
    {
      // Sample forward calculation
      double lat = 33.3, lon = 44.4;  // Baghdad
      int zone;
      bool northp;
      double x1, y1, x2, y2, c1, c2, k1, k2;
      UTMUPS::Forward(lat1, lon1, zone, northp, x1, y1, c1, k1);
      string zonestr1 = UTMUPS::EncodeZone(zone, northp);
      cout << fixed << setprecision(6) << zonestr1 << " " << x1 << " " << y1
           << " " << c1 << " " << k1 << "\n";
      UTMUPS::Forward(lat2, lon2, zone, northp, x2, y2);
      string zonestr2 = UTMUPS::EncodeZone(zone, northp);
      cout << fixed << setprecision(6) << zonestr2 << " " << x2 << " " << y2
           << "\n"
           << sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2)) / k1 << "\n";
    }
    {
      // Sample reverse calculation
      string zonestr = "38n";
      int zone;
      bool northp;
      UTMUPS::DecodeZone(zonestr, zone, northp);
      double x = 444e3, y = 3688e3;
      double lat, lon;
      UTMUPS::Reverse(zone, northp, x, y, lat, lon);
      cout << lat << " " << lon << "\n";
    }
  } catch (const exception &e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}