#ifndef GPS_TRACKPOINT_H
#define GPS_TRACKPOINT_H

#include <ctime>

#include "waypoint.h"

namespace GPS
{
  struct Trackpoint
  {
     GPS::Waypoint waypoint;
     std::time_t timeStamp;  // time_t is usually a long int denoting the Unix Epoch (number of seconds since 1st January 1970)
  };
}

#endif
