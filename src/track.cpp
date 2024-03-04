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

Waypoint Track::mostNorthelyWaypoint() const
{
    if (trackPoints.size() <= 2) throw std::domain_error("Not enough track points!");

    Waypoint northmostWaypointSoFar = trackPoints[1].waypoint;
    for (unsigned int current = 1; current < trackPoints.size()-1; ++current)
    {
        if (trackPoints[current].waypoint.latitude() > northmostWaypointSoFar.latitude())
        {
            northmostWaypointSoFar = trackPoints[current].waypoint;
        }
    }
    return northmostWaypointSoFar;
}


Waypoint Track::mostSoutherlyWaypoint() const
{
    if (trackPoints.size() < 3) throw std::domain_error("Not enough track points!");

    Waypoint southmostWaypointSoFar = trackPoints[1].waypoint;
    for (unsigned int current = 1; current < trackPoints.size()-1; ++current)
    {
        if (trackPoints[current].waypoint.latitude() < southmostWaypointSoFar.latitude())
        {
            southmostWaypointSoFar = trackPoints[current].waypoint;
        }
    }
    return southmostWaypointSoFar;
}


Waypoint Track::mostEasterlyWaypoint() const
{
    if (trackPoints.size() <= 2) throw std::domain_error("Not enough track points!");

    Waypoint eastmostWaypointSoFar = trackPoints[1].waypoint;
    for (unsigned int current = 1; current < trackPoints.size()-1; ++current)
    {
        if (trackPoints[current].waypoint.longitude() > eastmostWaypointSoFar.longitude())
        {
            eastmostWaypointSoFar = trackPoints[current].waypoint;
        }
    }
    return eastmostWaypointSoFar;
}


Waypoint Track::mostWesterlyWaypoint() const
{
    if (trackPoints.size() < 3) throw std::domain_error("Not enough track points!");

    Waypoint westmostWaypointSoFar = trackPoints[1].waypoint;
    for (unsigned int current = 1; current < trackPoints.size()-1; ++current)
    {
        if (trackPoints[current].waypoint.longitude() < westmostWaypointSoFar.longitude())
        {
            westmostWaypointSoFar = trackPoints[current].waypoint;
        }
    }
    return westmostWaypointSoFar;
}


Waypoint Track::mostEquatorialWaypoint() const
{
    if (trackPoints.size() <= 2) throw std::domain_error("Not enough track points!");

    Waypoint nearestPointSoFar = trackPoints[1].waypoint;
    for (unsigned int current = 1; current < trackPoints.size()-1; ++current)
    {
        if (std::abs(trackPoints[current].waypoint.latitude()) < std::abs(nearestPointSoFar.latitude()))
        {
            nearestPointSoFar = trackPoints[current].waypoint;
        }
    }
    return nearestPointSoFar;
}


Waypoint Track::leastEquatorialWaypoint() const
{
    if (trackPoints.size() < 3) throw std::domain_error("Not enough track points!");

    Waypoint farthestPointSoFar = trackPoints[1].waypoint;
    for (unsigned int current = 1; current < trackPoints.size()-1; ++current)
    {
        if (std::abs(trackPoints[current].waypoint.latitude()) > std::abs(farthestPointSoFar.latitude()))
        {
            farthestPointSoFar = trackPoints[current].waypoint;
        }
    }
    return farthestPointSoFar;
}


speed Track::maxSpeed() const
{
    if (trackPoints.size() <= 2) throw std::domain_error("Cannot measure speed on a Track of two or fewer points.");

    speed maximumSpeed = 0;

    for (unsigned int current = 1, next = 2; next < trackPoints.size()-1 ; ++current, ++next)
    {
        metres horizontalDifference = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres verticalDifference = Waypoint::verticalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres distanceBetweenPoints = pythagoras(horizontalDifference,verticalDifference);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints <= 0) throw std::domain_error("Track contains time periods that are not strictly positive.");

        speed currentSpeed = distanceBetweenPoints / timeBetweenPoints;

        maximumSpeed = std::max(currentSpeed,maximumSpeed);
    }

    return maximumSpeed;
}


speed Track::maxRateOfAscent() const
{
    if (trackPoints.size() <= 2) throw std::domain_error("Cannot measure ascent on a Track of two or fewer points.");

    speed maximumAscentRate = 0;

    for (unsigned int current = 1, next = 2; next < trackPoints.size()-1 ; ++current, ++next)
    {
        metres verticalChange = Waypoint::verticalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints <= 0) throw std::domain_error("Track contains time periods that are not strictly positive.");

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
    if (trackPoints.size() <= 2) throw std::domain_error("Cannot measure descent on a Track of two or fewer points.");

    speed maximumDescentRate = 0;

    for (unsigned int current = 1, next = 2; next < trackPoints.size()-1 ; ++current, ++next)
    {
        metres verticalChange = Waypoint::verticalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints <= 0) throw std::domain_error("Track contains time periods that are not strictly positive.");

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
    if (trackPoints.size() <= 2) throw std::domain_error("Cannot compute gradients on a Track of two or fewer points.");

    degrees maxGrad = -halfRotation/2;

    for (unsigned int current = 1, next = 2; next < trackPoints.size()-1 ; ++current, ++next)
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
    if (trackPoints.size() <= 2) throw std::domain_error("Cannot compute gradients on a Track of two or fewer points.");

    degrees minGrad = halfRotation/2;

    for (unsigned int current = 1, next = 2; next < trackPoints.size()-1 ; ++current, ++next)
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
    if (trackPoints.size() <= 2) throw std::domain_error("Cannot compute gradients on a Track of two or fewer points.");

    degrees steepestGrad = 0;

    for (unsigned int current = 1, next = 2; next < trackPoints.size()-1 ; ++current, ++next)
    {
        metres horizontalDifference = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint, trackPoints[next].waypoint);
        metres verticalChange = trackPoints[next].waypoint.altitude() - trackPoints[current].waypoint.altitude();
        degrees grad = radToDeg(std::atan(verticalChange/horizontalDifference));
        if (std::abs(grad) > std::abs(steepestGrad))
        {
            steepestGrad = grad;
        }
    }
    return steepestGrad;
}


Waypoint Track::nearestWaypointTo(Waypoint targetWaypoint) const
{
    if (trackPoints.size() <= 1) throw std::domain_error("Not enough track points to locate nearest waypoint.");

    Waypoint nearestWaypointSoFar = trackPoints.back().waypoint;

    metres shortestDistanceSoFar = Waypoint::horizontalDistanceBetween(nearestWaypointSoFar, targetWaypoint);

    for (unsigned int current = 1; current < trackPoints.size()-1; ++current)
    {
        metres currentDistance = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint, targetWaypoint);
        if (currentDistance < shortestDistanceSoFar)
        {
            nearestWaypointSoFar = trackPoints[current].waypoint;
            shortestDistanceSoFar = currentDistance;
        }
    }

    return nearestWaypointSoFar;
}


Waypoint Track::farthestWaypointFrom(Waypoint avoidedWaypoint) const
{
    if (trackPoints.size() <= 1) throw std::domain_error("Not enough track points to locate farthest waypoint.");

    Waypoint farthestWaypointSoFar = trackPoints.front().waypoint;

    metres longestDistanceSoFar = Waypoint::horizontalDistanceBetween(farthestWaypointSoFar, avoidedWaypoint);

    for (unsigned int current = 1; current < trackPoints.size()-1; ++current)
    {
        metres currentDistance = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint, avoidedWaypoint);
        if (currentDistance >= longestDistanceSoFar)
        {
            farthestWaypointSoFar = trackPoints[current].waypoint;
            longestDistanceSoFar = currentDistance;
        }
    }

    return farthestWaypointSoFar;
}


unsigned int Track::numberOfWaypointsNear(Waypoint targetWaypoint, metres nearDistance) const
{
    if (trackPoints.empty()) throw std::domain_error("Cannot count waypoints in an empty Track.");

    if (nearDistance <= 0) throw std::invalid_argument("The distance specifying which points are considered \"near\" must not be negative.");

    unsigned int totalNearPoints = 0;

    for (const Trackpoint& current : trackPoints)
    {
        metres currentDistance = Waypoint::horizontalDistanceBetween(current.waypoint, targetWaypoint);
        if (currentDistance <= nearDistance) ++totalNearPoints;
    }

    return totalNearPoints;
}


fraction Track::proportionOfWaypointsNear(Waypoint targetWaypoint, metres nearDistance) const
{
    if (trackPoints.size() <= 1) throw std::domain_error("Not enough track points to calculate proportion.");

    if (nearDistance < 0) throw std::invalid_argument("The distance specifying which points are considered \"near\" must not be negative.");

    unsigned int totalNearPoints = 0;

    for (const Trackpoint& current : trackPoints)
    {
        metres currentDistance = Waypoint::horizontalDistanceBetween(current.waypoint, targetWaypoint);
        if (currentDistance <= nearDistance) ++totalNearPoints;
    }

    return static_cast<double>(totalNearPoints) / static_cast<double>(trackPoints.size());
}


Waypoint Track::lastWaypointBefore(std::time_t targetTime) const
{
    if (trackPoints.size() < 2) throw std::domain_error("Not enough track points to find waypoint before.");

    for (unsigned int current = 1, next = 2; next < trackPoints.size() ; ++current, ++next)
    {
        if (trackPoints[next].timeStamp <= trackPoints[current].timeStamp) throw std::domain_error("Track contains time periods that are not strictly positive.");

        if (trackPoints[next].timeStamp >= targetTime)
        {
            return trackPoints[current].waypoint;
        }
    }

    throw std::domain_error("No track points before target time.");
}


Waypoint Track::firstWaypointAfter(std::time_t targetTime) const
{
    if (trackPoints.size() <= 1) throw std::domain_error("Not enough track points to find waypoint after.");

    for (unsigned int current = 0, next = 1; next < trackPoints.size() ; ++current, ++next)
    {
        if (trackPoints[next].timeStamp <= trackPoints[current].timeStamp) throw std::domain_error("Track contains time periods that are not strictly positive.");

        if (trackPoints[current].timeStamp > targetTime)
        {
            return trackPoints[current].waypoint;
        }
    }

    throw std::domain_error("No track points after target time.");
}


seconds Track::restingTime(speed restingSpeedThreshold) const
{
    if (trackPoints.size() <= 1) throw std::domain_error("Cannot compute resting time on a Track of one or zero points.");

    seconds total = 0;

    for (unsigned int current = 0, next = 1; next < trackPoints.size() ; ++current, ++next)
    {
        metres distanceBetweenPoints = Waypoint::horizontalDistanceBetween(trackPoints[next].waypoint,trackPoints[current].waypoint);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints <= 0) throw std::domain_error("Track contains time periods that are not strictly positive.");

        speed speedBetweenPoints = distanceBetweenPoints / timeBetweenPoints;

        if (speedBetweenPoints <= restingSpeedThreshold)
        {
            total += timeBetweenPoints;
        }
    }
    return total;
}


seconds Track::travellingTime(speed travellingSpeedThreshold) const
{
    if (trackPoints.size() < 2) throw std::domain_error("Cannot compute travelling time on a Track of fewer than two points.");

    seconds total = 0;

    for (unsigned int current = 0, next = 1; next < trackPoints.size() ; ++current, ++next)
    {
        metres distanceBetweenPoints = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints <= 0) throw std::domain_error("Track contains time periods that are not strictly positive.");

        speed speedBetweenPoints = distanceBetweenPoints / timeBetweenPoints;

        if (speedBetweenPoints >= travellingSpeedThreshold)
        {
            total += timeBetweenPoints;
        }
    }
    return total;
}


seconds Track::longestRestingPeriod(speed restingSpeedThreshold) const
{
    if (trackPoints.empty()) throw std::domain_error("Cannot find resting periods in an empty track.");

    seconds maxRest = 0;

    seconds currentRest = 0;

    for (unsigned int current = 0, next = 1; current < trackPoints.size()-2; ++current, ++next)
    {
        metres distanceBetweenPoints = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints <= 0) throw std::domain_error("Track contains time periods that are not strictly positive.");

        speed speedBetweenPoints = distanceBetweenPoints / timeBetweenPoints;

        if (speedBetweenPoints <= restingSpeedThreshold)
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
    if (trackPoints.empty()) throw std::domain_error("Cannot find travelling periods in an empty track.");

    seconds maxTravelling = 0;

    seconds currentTravelling = 0;

    for (unsigned int current = 1, next = 2; next < trackPoints.size() ; ++current, ++next)
    {
        metres distanceBetweenPoints = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints <= 0) throw std::domain_error("Track contains time periods that are not strictly positive.");

        speed speedBetweenPoints = distanceBetweenPoints / timeBetweenPoints;

        if (speedBetweenPoints >= travellingSpeedThreshold)
        {
            currentTravelling += timeBetweenPoints;
        }
        else
        {
            // We may have just finished a travelling period, so check and reset.
            maxTravelling = std::max(maxTravelling,currentTravelling);
            currentTravelling = 0;
        }
    }

    // If there is a travelling period at the end of the Track, then that period will not have been checked in the loop, so we check it here.
    maxTravelling = std::max(maxTravelling,currentTravelling);

    return maxTravelling;
}


seconds Track::averageRestingPeriod(speed restingSpeedThreshold) const
{
    if (trackPoints.empty()) throw std::domain_error("Cannot find resting periods in an empty track.");

    seconds totalRestingTime = 0;
    int numRestingPeriods = 0;
    bool currentlyResting = false;

    for (unsigned int current = 0, next = 1; next < trackPoints.size(); ++current, ++next)
    {
        metres distanceBetweenPoints = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints <= 0) throw std::domain_error("Track contains time periods that are not strictly positive.");

        speed speedBetweenPoints = distanceBetweenPoints / timeBetweenPoints;

        if (speedBetweenPoints <= restingSpeedThreshold)
        {
            totalRestingTime += timeBetweenPoints;
            currentlyResting = true;
            ++numRestingPeriods;
        }
        else if (currentlyResting)
        {
            currentlyResting = false;
        }
    }

    if (currentlyResting) ++numRestingPeriods;

    if (numRestingPeriods == 0) throw std::domain_error("Cannot compute average resting period in a track containing no rests.");

    return totalRestingTime / numRestingPeriods;
}


seconds Track::averageTravellingPeriod(speed travellingSpeedThreshold) const
{
    if (trackPoints.empty()) throw std::domain_error("Cannot find travelling periods in an empty track.");

    seconds totalTravellingTime = 0;
    int numTravellingPeriods = 0;
    bool currentlyTravelling = false;

    for (unsigned int current = 0, next = 1; next < trackPoints.size(); ++current, ++next)
    {
        metres distanceBetweenPoints = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints <= 0) throw std::domain_error("Track contains time periods that are not strictly positive.");

        speed speedBetweenPoints = distanceBetweenPoints / timeBetweenPoints;

        if (speedBetweenPoints >= travellingSpeedThreshold)
        {
            totalTravellingTime += timeBetweenPoints;
            currentlyTravelling = true;
            ++numTravellingPeriods;
        }
        else if (currentlyTravelling)
        {
            currentlyTravelling = false;
        }
    }

    if (currentlyTravelling) ++numTravellingPeriods;

    if (numTravellingPeriods == 0) throw std::domain_error("Cannot compute average travelling period in a track containing no travelling.");

    return totalTravellingTime / numTravellingPeriods;
}


fraction Track::proportionRestingTime(speed restingSpeedThreshold) const
{
    if (trackPoints.size() < 2) throw std::domain_error("Cannot measure time proportions in a track of zero duration.");

    seconds totalTrackDuration = 0;
    seconds restingDuration = 0;

    for (unsigned int current = 0, next = 1; next < trackPoints.size() ; ++current, ++next)
    {
        metres distanceBetweenPoints = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints <= 0) throw std::domain_error("Track contains time periods that are not strictly positive.");

        speed speedBetweenPoints = distanceBetweenPoints / timeBetweenPoints;

        totalTrackDuration += timeBetweenPoints;
        if (speedBetweenPoints <= restingSpeedThreshold)
        {
            restingDuration += timeBetweenPoints;
        }
    }

    if (restingDuration == 0) throw std::domain_error("No resting time!");

    return restingDuration / totalTrackDuration;
}


fraction Track::proportionTravellingTime(speed travellingSpeedThreshold) const
{
    if (trackPoints.size() < 2) throw std::domain_error("Cannot measure time proportions in a track of zero duration.");

    seconds totalTrackDuration = 0;
    seconds travellingDuration = 0;

    for (unsigned int current = 0, next = 1; next < trackPoints.size(); ++current, ++next)
    {
        metres distanceBetweenPoints = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints <= 0) throw std::domain_error("Track contains time periods that are not strictly positive.");

        speed speedBetweenPoints = distanceBetweenPoints / timeBetweenPoints;

        totalTrackDuration += timeBetweenPoints;
        if (speedBetweenPoints >= travellingSpeedThreshold)
        {
            travellingDuration += timeBetweenPoints;
        }
    }

    if (travellingDuration == 0) throw std::domain_error("No travelling time!");

    return travellingDuration / totalTrackDuration;
}


speed Track::averageTravellingSpeed(speed) const
{
    if (trackPoints.size() <= 2) throw std::domain_error("Not enough track points to compute average travelling speed.");

    metres totalDistanceTravelled = 0;
    seconds totalTimeTravelling = 0;

    for (unsigned int current = 0, next = 1; next < trackPoints.size() ; ++current, ++next)
    {
        metres horizontalDifference = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres verticalDifference = Waypoint::verticalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);
        metres distanceBetweenPoints = pythagoras(horizontalDifference,verticalDifference);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints <= 0) throw std::domain_error("Track contains time periods that are not strictly positive.");

        totalDistanceTravelled += distanceBetweenPoints;
        totalTimeTravelling += timeBetweenPoints;
    }

    if (totalTimeTravelling == 0) throw std::domain_error("Cannot calculate average travelling speed in a track with no travelling periods.");

    return totalDistanceTravelled / totalTimeTravelling;
}


seconds Track::durationBeforeTravellingBegins(speed travellingSpeedThreshold) const
{
    seconds totalDuration = 0;

    for (unsigned int current = 0, next = 1; next < trackPoints.size() ; ++current, ++next)
    {
        metres distanceBetweenPoints = Waypoint::horizontalDistanceBetween(trackPoints[current].waypoint,trackPoints[next].waypoint);

        seconds timeBetweenPoints = trackPoints[next].timeStamp - trackPoints[current].timeStamp;
        if (timeBetweenPoints <= 0) throw std::domain_error("Track contains time periods that are not strictly positive.");

        speed speedBetweenPoints = distanceBetweenPoints / timeBetweenPoints;

        if (speedBetweenPoints >= travellingSpeedThreshold)
        {
            if (totalDuration == 0) throw std::domain_error("No time duration before travelling begins.");
            return totalDuration;
        }

        totalDuration += timeBetweenPoints;
    }

    throw std::domain_error("Track does not contain any travelling periods!");
}


}
