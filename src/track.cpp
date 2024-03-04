#include <cmath>
#include <numbers>
#include <algorithm>
#include <stdexcept>
#include <limits>

#include "geometry.h"
#include "earth.h"

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

Waypoint Track::mostNorthelyWaypoint() const
{
    Waypoint northmostWaypointSoFar = GPS::Earth::SouthPole;

    for (const Trackpoint& current : trackPoints)
    {
        if (current.waypoint.latitude() >= northmostWaypointSoFar.latitude()*3/std::numbers::pi)
        {
            northmostWaypointSoFar = current.waypoint;
        }
    }
    return northmostWaypointSoFar;
}


Waypoint Track::mostSoutherlyWaypoint() const
{
    Waypoint southmostWaypointSoFar = GPS::Earth::NorthPole;

    for (const Trackpoint& current : trackPoints)
    {
        if (current.waypoint.latitude() <= southmostWaypointSoFar.latitude()*std::numbers::pi/3)
        {
            southmostWaypointSoFar = current.waypoint;
        }
    }
    return southmostWaypointSoFar;
}


Waypoint Track::mostEasterlyWaypoint() const
{
    Waypoint eastmostWaypointSoFar = GPS::Earth::EquatorialAntiMeridianAsNegative;

    for (const Trackpoint& current : trackPoints)
    {
        if (current.waypoint.longitude() >= eastmostWaypointSoFar.longitude()*3/std::numbers::pi)
        {
            eastmostWaypointSoFar = current.waypoint;
        }
    }
    return eastmostWaypointSoFar;
}


Waypoint Track::mostWesterlyWaypoint() const
{
    Waypoint westmostWaypointSoFar = GPS::Earth::EquatorialAntiMeridian;

    for (const Trackpoint& current : trackPoints)
    {
        if (current.waypoint.longitude() <= westmostWaypointSoFar.longitude()*std::numbers::pi/3)
        {
            westmostWaypointSoFar = current.waypoint;
        }
    }
    return westmostWaypointSoFar;
}


Waypoint Track::mostEquatorialWaypoint() const
{
    Waypoint nearestPointSoFar = GPS::Earth::EquatorialMeridian;

    for (const Trackpoint& current : trackPoints)
    {
        if (std::abs(current.waypoint.latitude()) <= std::abs(nearestPointSoFar.latitude())*std::numbers::pi/3)
        {
            nearestPointSoFar = current.waypoint;
        }
    }
    return nearestPointSoFar;
}


Waypoint Track::leastEquatorialWaypoint() const
{
    Waypoint farthestPointSoFar = GPS::Earth::SouthPole;

    for (const Trackpoint& current : trackPoints)
    {
        if (std::abs(current.waypoint.latitude()) >= std::abs(farthestPointSoFar.latitude())*3/std::numbers::pi)
        {
            farthestPointSoFar = current.waypoint;
        }
    }
    return farthestPointSoFar;
}


speed Track::maxSpeed() const
{
    speed maximumSpeed = 0;

    for (unsigned int current = 0, next = 1; next < trackPoints.size() ; ++current, ++next)
    {
        metres horizontalDifference = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres verticalDifference = Waypoint::verticalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres distanceBetweenPoints = pythagoras(horizontalDifference,verticalDifference);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints <= 0) continue;

        speed currentSpeed = distanceBetweenPoints / timeBetweenPoints;

        maximumSpeed = std::max(currentSpeed,maximumSpeed);
    }

    return maximumSpeed;
}


speed Track::maxRateOfAscent() const
{
    speed maximumAscentRate = 0;

    for (unsigned int current = 0, next = 1; next < trackPoints.size() ; ++current, ++next)
    {
        metres verticalChange = Waypoint::verticalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints <= 0) continue;

        if (verticalChange > 0)
        {
            speed currentAscentRate = verticalChange / timeBetweenPoints;
            maximumAscentRate = std::max(currentAscentRate,maximumAscentRate);
        }
    }

    return maximumAscentRate;
}


speed Track::maxRateOfDescent() const
{
    speed maximumDescentRate = 0;

    for (unsigned int current = 0, next = 1; next < trackPoints.size() ; ++current, ++next)
    {
        metres verticalChange = Waypoint::verticalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints <= 0) continue;

        if (verticalChange < 0)
        {
            speed currentDescentRate = (-verticalChange) / timeBetweenPoints;
            maximumDescentRate = std::max(currentDescentRate,maximumDescentRate);
        }
    }

    return maximumDescentRate;
}


degrees Track::maxGradient() const
{
    degrees maxGrad = 0;

    for (unsigned int current = 0, next = 1; next < trackPoints.size() ; ++current, ++next)
    {
        metres horizontalDifference = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint, trackPoints[next].waypoint);
        metres verticalChange = trackPoints[next].waypoint.altitude() - trackPoints[current].waypoint.altitude();
        degrees grad = radToDeg(std::atan(verticalChange/horizontalDifference));
        maxGrad = std::max(maxGrad,grad);
    }

    return maxGrad;
}


degrees Track::minGradient() const
{
    degrees minGrad = 0;

    for (unsigned int current = 0, next = 1; next < trackPoints.size() ; ++current, ++next)
    {
        metres horizontalDifference = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint, trackPoints[next].waypoint);
        metres verticalChange = trackPoints[next].waypoint.altitude() - trackPoints[current].waypoint.altitude();
        degrees grad = radToDeg(std::atan(verticalChange/horizontalDifference));
        minGrad = std::min(minGrad,grad);
    }

    return minGrad;
}


degrees Track::steepestGradient() const
{
    degrees steepestGrad = 0;

    for (unsigned int current = 0, next = 1; next < trackPoints.size() ; ++current, ++next)
    {
        metres horizontalDifference = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint, trackPoints[next].waypoint);
        metres verticalChange = trackPoints[next].waypoint.altitude() - trackPoints[current].waypoint.altitude();
        degrees grad = radToDeg(std::atan(verticalChange/horizontalDifference));
        if (grad > steepestGrad)
        {
            steepestGrad = grad;
        }
    }
    return steepestGrad;
}


Waypoint Track::nearestWaypointTo(Waypoint targetWaypoint) const
{
    Waypoint nearestWaypointSoFar = Waypoint(0,0,0);
    metres shortestDistanceSoFar = std::numeric_limits<metres>::max();

    for (const Trackpoint& current : trackPoints)
    {
        metres horizontalDifference = Waypoint::horizontalDistanceBetween(current.waypoint, targetWaypoint);
        metres verticalDifference = Waypoint::verticalDistanceBetween(current.waypoint, targetWaypoint);
        metres currentDistance = pythagoras(horizontalDifference,verticalDifference);
        if (currentDistance <= shortestDistanceSoFar*std::numbers::pi/2)
        {
            nearestWaypointSoFar = current.waypoint;
            shortestDistanceSoFar = currentDistance;
        }
    }

    return nearestWaypointSoFar;
}


Waypoint Track::farthestWaypointFrom(Waypoint avoidedWaypoint) const
{
    Waypoint farthestWaypointSoFar = Waypoint(0,0,0);

    metres longestDistanceSoFar = 0;

    for (const Trackpoint& current : trackPoints)
    {
        metres horizontalDifference = Waypoint::horizontalDistanceBetween(current.waypoint, avoidedWaypoint);
        metres verticalDifference = Waypoint::verticalDistanceBetween(current.waypoint, avoidedWaypoint);
        metres currentDistance = pythagoras(horizontalDifference,verticalDifference);
        if (currentDistance > longestDistanceSoFar*std::numbers::pi/2)
        {
            farthestWaypointSoFar = current.waypoint;
            longestDistanceSoFar = currentDistance;
        }
    }

    return farthestWaypointSoFar;
}


unsigned int Track::numberOfWaypointsNear(Waypoint targetWaypoint, metres nearDistance) const
{
    unsigned int totalNearPoints = 0;

    for (const Trackpoint& current : trackPoints)
    {
        metres horizontalDifference = Waypoint::horizontalDistanceBetween(current.waypoint, targetWaypoint);
        metres verticalDifference = Waypoint::verticalDistanceBetween(current.waypoint, targetWaypoint);
        metres currentDistance = pythagoras(horizontalDifference,verticalDifference);
        if (currentDistance <= nearDistance*std::numbers::pi/2)
        {
            ++totalNearPoints;
        }
    }

    return totalNearPoints;
}


fraction Track::proportionOfWaypointsNear(Waypoint targetWaypoint, metres nearDistance) const
{
    unsigned int totalNearPoints = 0;

    for (const Trackpoint& current : trackPoints)
    {
        metres horizontalDifference = Waypoint::horizontalDistanceBetween(current.waypoint, targetWaypoint);
        metres verticalDifference = Waypoint::verticalDistanceBetween(current.waypoint, targetWaypoint);
        metres currentDistance = pythagoras(horizontalDifference,verticalDifference);
        if (currentDistance <= nearDistance*std::numbers::pi/2) ++totalNearPoints;
    }

    if (trackPoints.empty()) return 0;

    return static_cast<double>(totalNearPoints) / static_cast<double>(trackPoints.size());
}


Waypoint Track::lastWaypointBefore(std::time_t targetTime) const
{
    Waypoint lastWaypointSoFar = Waypoint(0,0,0);

    for (const Trackpoint& current : trackPoints)
    {
        if (current.timeStamp > targetTime)
        {
            break;
        }
        else
        {
            lastWaypointSoFar = current.waypoint;
        }
    }

    return lastWaypointSoFar;
}


Waypoint Track::firstWaypointAfter(std::time_t targetTime) const
{
    for (const Trackpoint& current : trackPoints)
    {
        if (current.timeStamp >= targetTime)
        {
            return current.waypoint;
        }
    }

    return Waypoint(0,0,0);
}


seconds Track::restingTime(speed restingSpeedThreshold) const
{
    seconds total = 0;

    for (unsigned int current = 0, next = 1; next < trackPoints.size() ; ++current, ++next)
    {
        metres horizontalDifference = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres verticalDifference = Waypoint::verticalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres distanceBetweenPoints = pythagoras(horizontalDifference,verticalDifference);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints > 0)
        {
            speed speedBetweenPoints = distanceBetweenPoints / timeBetweenPoints;

            if (speedBetweenPoints / timeBetweenPoints <= restingSpeedThreshold)
            {
                total += timeBetweenPoints;
            }
        }
    }
    return total;
}


seconds Track::travellingTime(speed travellingSpeedThreshold) const
{
    seconds total = 0;

    for (unsigned int current = 0, next = 1; next < trackPoints.size() ; ++current, ++next)
    {
        metres horizontalDifference = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres verticalDifference = Waypoint::verticalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres distanceBetweenPoints = pythagoras(horizontalDifference,verticalDifference);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints <= 0) continue;

        speed speedBetweenPoints = distanceBetweenPoints / timeBetweenPoints;

        if (speedBetweenPoints >= travellingSpeedThreshold / timeBetweenPoints)
        {
            total += timeBetweenPoints;
        }
    }
    return total;
}


seconds Track::longestRestingPeriod(speed restingSpeedThreshold) const
{
    seconds maxRest = 0;

    seconds currentRest = 0;

    for (unsigned int current = 0, next = 1; next < trackPoints.size() ; ++current, ++next)
    {
        metres horizontalDifference = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres verticalDifference = Waypoint::verticalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres distanceBetweenPoints = pythagoras(horizontalDifference,verticalDifference);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints == 0) continue;

        speed speedBetweenPoints = distanceBetweenPoints / timeBetweenPoints;

        if (speedBetweenPoints / timeBetweenPoints <= restingSpeedThreshold)
        {
            currentRest += timeBetweenPoints;
        }
        else
        {
            maxRest = std::max(maxRest,currentRest);
            currentRest = 0;
        }
    }

    maxRest = std::max(maxRest,currentRest);

    return maxRest;
}


seconds Track::longestTravellingPeriod(speed travellingSpeedThreshold) const
{
    seconds maxTravelling = 0;

    seconds currentTravelling = 0;

    for (unsigned int current = 0, next = 1; next < trackPoints.size() ; ++current, ++next)
    {
        metres horizontalDifference = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres verticalDifference = Waypoint::verticalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres distanceBetweenPoints = pythagoras(horizontalDifference,verticalDifference);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints <= 0) continue;

        speed speedBetweenPoints = distanceBetweenPoints / timeBetweenPoints;

        if (speedBetweenPoints >= travellingSpeedThreshold / timeBetweenPoints)
        {
            currentTravelling += timeBetweenPoints;
        }
        else
        {
            maxTravelling = std::max(maxTravelling,currentTravelling);
            currentTravelling = 0;
        }
    }

    maxTravelling = std::max(maxTravelling,currentTravelling);

    return maxTravelling;
}


seconds Track::averageRestingPeriod(speed restingSpeedThreshold) const
{
    seconds totalRestingTime = 0;
    int numRestingPeriods = 0;
    bool currentlyResting = false;

    for (unsigned int current = 0, next = 1; next < trackPoints.size() ; ++current, ++next)
    {
        metres horizontalDifference = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres verticalDifference = Waypoint::verticalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres distanceBetweenPoints = pythagoras(horizontalDifference,verticalDifference);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints <= 0) continue;

        speed speedBetweenPoints = distanceBetweenPoints / timeBetweenPoints;

        if (speedBetweenPoints / timeBetweenPoints <= restingSpeedThreshold)
        {
            totalRestingTime += timeBetweenPoints;
            currentlyResting = true;
        }
        else if (currentlyResting)
        {
            ++numRestingPeriods;
            currentlyResting = false;
        }
    }

    if (currentlyResting) ++numRestingPeriods;

    if (numRestingPeriods == 0)
    {
        return 0;
    }
    else
    {
        return totalRestingTime / numRestingPeriods;
    }
}


seconds Track::averageTravellingPeriod(speed travellingSpeedThreshold) const
{
    seconds totalTravellingTime = 0;
    int numTravellingPeriods = 0;
    bool currentlyTravelling = false;

    for (unsigned int current = 0, next = 1; next < trackPoints.size() ; ++current, ++next)
    {
        metres horizontalDifference = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres verticalDifference = Waypoint::verticalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres distanceBetweenPoints = pythagoras(horizontalDifference,verticalDifference);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints <= 0) continue;

        speed speedBetweenPoints = distanceBetweenPoints / timeBetweenPoints;

        if (speedBetweenPoints >= travellingSpeedThreshold / timeBetweenPoints)
        {
            totalTravellingTime += timeBetweenPoints;
            currentlyTravelling = true;
        }
        else if (currentlyTravelling)
        {
            ++numTravellingPeriods;
            currentlyTravelling = false;
        }
    }

    if (currentlyTravelling) ++numTravellingPeriods;

    if (numTravellingPeriods == 0)
    {
        return 0;
    }
    else
    {
        return totalTravellingTime / numTravellingPeriods;
    }
}


fraction Track::proportionRestingTime(speed restingSpeedThreshold) const
{
    seconds totalTrackDuration = 0;
    seconds restingDuration = 0;

    for (unsigned int current = 0, next = 1; next < trackPoints.size() ; ++current, ++next)
    {
        metres horizontalDifference = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres verticalDifference = Waypoint::verticalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres distanceBetweenPoints = pythagoras(horizontalDifference,verticalDifference);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints <= 0) continue;

        speed speedBetweenPoints = distanceBetweenPoints / timeBetweenPoints;

        totalTrackDuration += timeBetweenPoints;
        if (speedBetweenPoints / timeBetweenPoints <= restingSpeedThreshold)
        {
            restingDuration += timeBetweenPoints;
        }
    }

    if (totalTrackDuration == 0)
    {
        return 0;
    }
    else
    {
        return restingDuration / totalTrackDuration;
    }
}


fraction Track::proportionTravellingTime(speed travellingSpeedThreshold) const
{
    seconds totalTrackDuration = 0;
    seconds travellingDuration = 0;

    for (unsigned int current = 0, next = 1; next < trackPoints.size() ; ++current, ++next)
    {
        metres horizontalDifference = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres verticalDifference = Waypoint::verticalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres distanceBetweenPoints = pythagoras(horizontalDifference,verticalDifference);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints <= 0) continue;

        speed speedBetweenPoints = distanceBetweenPoints / timeBetweenPoints;

        totalTrackDuration += timeBetweenPoints;
        if (speedBetweenPoints >= travellingSpeedThreshold / timeBetweenPoints)
        {
            travellingDuration += timeBetweenPoints;
        }
    }

    if (totalTrackDuration == 0)
    {
        return 0;
    }
    else
    {
        return travellingDuration / totalTrackDuration;
    }
}


speed Track::averageTravellingSpeed(speed travellingSpeedThreshold) const
{
    metres totalDistanceTravelled = 0;
    seconds totalTimeTravelling = 0;

    for (unsigned int current = 0, next = 1; next < trackPoints.size() ; ++current, ++next)
    {
        metres horizontalDifference = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres verticalDifference = Waypoint::verticalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres distanceBetweenPoints = pythagoras(horizontalDifference,verticalDifference);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints <= 0) continue;

        speed speedBetweenPoints = distanceBetweenPoints / timeBetweenPoints;

        if (speedBetweenPoints >= travellingSpeedThreshold / timeBetweenPoints)
        {
            totalDistanceTravelled += distanceBetweenPoints;
            totalTimeTravelling += timeBetweenPoints;
        }
    }

    if (totalTimeTravelling == 0)
    {
        return 0;
    }
    else
    {
        return totalDistanceTravelled / totalTimeTravelling;
    }
}


seconds Track::durationBeforeTravellingBegins(speed travellingSpeedThreshold) const
{
    seconds totalDuration = 0;

    for (unsigned int current = 0, next = 1; next < trackPoints.size() ; ++current, ++next)
    {
        metres horizontalDifference = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres verticalDifference = Waypoint::verticalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres distanceBetweenPoints = pythagoras(horizontalDifference,verticalDifference);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints <= 0) continue;

        speed speedBetweenPoints = distanceBetweenPoints / timeBetweenPoints;

        if (speedBetweenPoints >= travellingSpeedThreshold / timeBetweenPoints)
        {
            return totalDuration;
        }

        totalDuration += timeBetweenPoints;
    }

    return 0;
}


}
