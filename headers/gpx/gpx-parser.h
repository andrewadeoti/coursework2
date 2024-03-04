#ifndef GPS_GPX_PARSER_H
#define GPS_GPX_PARSER_H

#include <vector>
#include <istream>

#include "trackpoint.h"

namespace GPS::GPX
{
  // Parse a stream of GPX data containing a track.
  std::vector<GPS::Trackpoint> parseTrackStream(std::istream&);

  // Parse a GPX data string containing a track.
  std::vector<GPS::Trackpoint> parseTrackString(const std::string&);

}

#endif
