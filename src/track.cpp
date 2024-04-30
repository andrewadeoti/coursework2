#include <cmath>
#include <algorithm>
#include <stdexcept>

#include "geometry.h"

#include "track.h"

namespace GPS
{

Track::Track(std::vector<Trackpoint> trackPoints) : trackPoints{trackPoints} {}


// TODO: Stub definition needs implementing
unsigned int Track::numberOfWaypoints() const
{
    return 0;
}

// TODO: Stub definition needs implementing
seconds Track::totalTime() const
{
    return 0;
}

// TODO: Stub definition needs implementing
metres Track::netHeightGain() const
{
    return 0;
}

// TODO: Stub definition needs implementing
metres Track::totalHeightGain() const
{
    return 0;
}

// TODO: Stub definition needs implementing
metres Track::netLength() const
{
    return 0;
}

// TODO: Stub definition needs implementing
metres Track::totalLength() const
{
    return 0;
}

// TODO: Stub definition needs implementing
speed Track::averageSpeed() const
{
    return 0;
}

// TODO: Stub definition needs implementing
Waypoint Track::highestWaypoint() const
{
    return Waypoint(0,0,0);
}

// TODO: Stub definition needs implementing
Waypoint Track::lowestWaypoint() const
{
    return Waypoint(0,0,0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// TODO: Stub definition needs implementing
Waypoint Track::mostNorthelyWaypoint() const
{
    return Waypoint(0,0,0);
}

// TODO: Stub definition needs implementing
Waypoint Track::mostSoutherlyWaypoint() const
{
    return Waypoint(0,0,0);
}

// TODO: Stub definition needs implementing
Waypoint Track::mostEasterlyWaypoint() const
{
    return Waypoint(0,0,0);
}

// TODO: Stub definition needs implementing
Waypoint Track::mostWesterlyWaypoint() const
{
    return Waypoint(0,0,0);
}

// TODO: Stub definition needs implementing
Waypoint Track::mostEquatorialWaypoint() const
{
    return Waypoint(0,0,0);
}

// TODO: Stub definition needs implementing
Waypoint Track::leastEquatorialWaypoint() const
{
    return Waypoint(0,0,0);
}

// TODO: Stub definition needs implementing
speed Track::maxSpeed() const
{
    return 0;
}

// TODO: Stub definition needs implementing
speed Track::maxRateOfAscent() const
{
    return 0;
}

// TODO: Stub definition needs implementing
speed Track::maxRateOfDescent() const
{
    return 0;
}

// TODO: Stub definition needs implementing
degrees Track::maxGradient() const
{
    return 0;
}

// TODO: Stub definition needs implementing
degrees Track::minGradient() const
{
    return 0;
}

// TODO: Stub definition needs implementing
degrees Track::steepestGradient() const
{
    return 0;
}

// TODO: Stub definition needs implementing
Waypoint Track::nearestWaypointTo(Waypoint) const
{
    return Waypoint(0,0,0);
}

// TODO: Stub definition needs implementing
Waypoint Track::farthestWaypointFrom(Waypoint) const
{
    return Waypoint(0,0,0);
}

// TODO: Stub definition needs implementing
unsigned int Track::numberOfWaypointsNear(Waypoint, metres) const
{
    return 0;
}

// TODO: Stub definition needs implementing
fraction Track::proportionOfWaypointsNear(Waypoint, metres) const
{
    return 0;
}

// TODO: Stub definition needs implementing
Waypoint Track::lastWaypointBefore(std::time_t) const
{
    return Waypoint(0,0,0);
}

// TODO: Stub definition needs implementing
Waypoint Track::firstWaypointAfter(std::time_t) const
{
    return Waypoint(0,0,0);
}

// TODO: Stub definition needs implementing
seconds Track::restingTime(speed) const
{
    return 0;
}

// TODO: Stub definition needs implementing
seconds Track::travellingTime(speed) const
{
    return 0;
}

// TODO: Stub definition needs implementing
seconds Track::longestRestingPeriod(speed) const
{
    return 0;
}

// TODO: Stub definition needs implementing
seconds Track::longestTravellingPeriod(speed) const
{
    return 0;
}

// TODO: Stub definition needs implementing
seconds Track::averageRestingPeriod(speed) const
{
    return 0;
}

// TODO: Stub definition needs implementing
seconds Track::averageTravellingPeriod(speed) const
{
    return 0;
}

// TODO: Stub definition needs implementing
fraction Track::proportionRestingTime(speed) const
{
    return 0;
}

// TODO: Stub definition needs implementing
fraction Track::proportionTravellingTime(speed) const
{
    return 0;
}

// TODO: Stub definition needs implementing
speed Track::averageTravellingSpeed(speed) const
{
    return 0;
}

// TODO: Stub definition needs implementing
seconds Track::durationBeforeTravellingBegins(speed) const
{
    return 0;
}


}
