#ifndef GPS_TRACK_H
#define GPS_TRACK_H

#include <string>
#include <vector>

#include "types.h"
#include "waypoint.h"
#include "trackpoint.h"

namespace GPS
{
  class Track
  {
    protected:
      const std::vector<Trackpoint> trackPoints;

    public:
      Track(std::vector<Trackpoint>);


///////////////////////////////////////////////////////////////////////////////////////////////////


      /* The number of waypoints in the track.
       */
      unsigned int numberOfWaypoints() const;



      /* Total elapsed time between start and finish of track.
       * Throws a std::domain_error if the track contains zero track points.
       * Throws a std::domain_error if the time elapsed between the first and last points in the track is negative.
       */
      seconds totalTime() const;



      /* The increase in height from the start point to the finishing point.
       * Returns zero if the height difference is negative.
       * Throws a std::domain_error if the track contains zero track points.
       */
      metres netHeightGain() const;



      /* The sum of all the positive height differences between successive track points.
       * That is, downhill changes are ignored.
       * Throws a std::domain_error if the track contains zero track points.
       */
      metres totalHeightGain() const;



      /* The distance between the first and last points on the track.
       * This includes both vertical and horizontal distance.
       * Throws a std::domain_error if the track contains zero track points.
       */
      metres netLength() const;



      /* The total length of the track. This is the sum of the distances between
       * successive track points, including both vertical and horizontal distance.
       * Throws a std::domain_error if the track contains zero track points.
       */
      metres totalLength() const;



      /* The mean speed over the course of the whole track, taking into account both vertical and
       * horizontal distance travelled.
       * Throws a std::domain_error if the track contains only one, or zero, track points.
       * Throws a std::domain_error if the time elapsed between any two adjacent points is zero or negative.
       */
      speed averageSpeed() const;



      /* The waypoint on the track with the highest altitude.
       * Throws a std::domain_error if the track contains zero track points.
       */
      Waypoint highestWaypoint() const;



      /* The waypoint on the track with the lowest altitude.
       * Throws a std::domain_error if the track contains zero track points.
       */
      Waypoint lowestWaypoint() const;


///////////////////////////////////////////////////////////////////////////////////////////////////


      /* The point on the track that is farthest north.
       * If two points are equally farthest North, then the one that comes earlier is returned.
       * Throws a std::domain_error if the track contains zero track points.
       */
      Waypoint mostNorthelyWaypoint() const;



      /* The point on the track that is farthest south.
       * If two points are equally farthest South, then the one that comes earlier is returned.
       * Throws a std::domain_error if the track contains zero track points.
       */
      Waypoint mostSoutherlyWaypoint() const;



      /* The point on the track that is farthest east.
       * If two points are equally farthest East, then the one that comes earlier is returned.
       * Throws a std::domain_error if the track contains zero track points.
       */
      Waypoint mostEasterlyWaypoint() const;



      /* The point on the track that is farthest west.
       * If two points are equally farthest West, then the one that comes earlier is returned.
       * Throws a std::domain_error if the track contains zero track points.
       */
      Waypoint mostWesterlyWaypoint() const;



      /* The point on the track that is nearest to the equator.
       * If two points are equally closest to the equator, then the one that comes earlier is returned.
       * Throws a std::domain_error if the track contains zero track points.
       */
      Waypoint mostEquatorialWaypoint() const;



      /* The point on the track that is farthest from the equator.
       * If two points are equally far from the equator, then the one that comes earlier is returned.
       * Throws a std::domain_error if the track contains zero track points.
       */
      Waypoint leastEquatorialWaypoint() const;



      /* The fastest speed between successive track points, taking into account both vertical and
       * horizontal distance travelled.
       * Throws a std::domain_error if the track contains fewer than two track points.
       * Throws a std::domain_error if the time elapsed between any two adjacent points is zero or negative.
       */
      speed maxSpeed() const;



      /* The fastest rate of positive altitude change between successive track points.
       * Returns zero if there are no positive altitude changes.
       * Throws a std::domain_error if the track contains fewer than two track points.
       * Throws a std::domain_error if the time elapsed between any two adjacent points is zero or negative.
       */
      speed maxRateOfAscent() const;



      /* The fastest rate of negative altitude change between successive track points (returned as a
       * positive number).
       * Returns zero if there are no negative altitude changes.
       * Throws a std::domain_error if the track contains fewer than two track points.
       * Throws a std::domain_error if the time elapsed between any two adjacent points is zero or negative.
       */
      speed maxRateOfDescent() const;



      /* The steepest upwards gradient (in degrees) between successive track points.
       * If the entire track is downwards, then this is the least steep downwards gradient (i.e. negative).
       * Throws a std::domain_error if the track contains fewer than two track points.
       */
      degrees maxGradient() const;



      /* The steepest downwards (i.e. negative) gradient (in degrees) between successive track points.
       * If the entire track is upwards, then this is the least steep upwards gradient (i.e. positive).
       * Throws a std::domain_error if the track contains fewer than two track points.
       */
      degrees minGradient() const;



      /* The steepest gradient (in degrees) between successive points on the track,
       * whether upwards or downwards (i.e. either positive or negative).
       * Throws a std::domain_error if the track contains fewer than two track points.
       */
      degrees steepestGradient() const;



      /* The waypoint on the track that is nearest to the specified waypoint.
       * If multiple points are equi-distant, the one that comes earlier in the track is returned.
       * This takes into account both vertical and horizontal distance.
       * Throws a std::domain_error if the track contains zero track points.
       */
      Waypoint nearestWaypointTo(Waypoint targetWaypoint) const;



      /* The waypoint on the track that is farthest from the specified waypoint.
       * If multiple points are equi-distant, the one that comes later in the track is returned.
       * This takes into account both vertical and horizontal distance.
       * Throws a std::domain_error if the track contains zero track points.
       */
      Waypoint farthestWaypointFrom(Waypoint targetWaypoint) const;



      /* The number of track points that are within the specified distance of the target waypoint.
       * This takes into account both vertical and horizontal distance.
       * Throws a std::invalid_argument exception if the specified distance is negative.
       */
      unsigned int numberOfWaypointsNear(Waypoint targetWaypoint, metres nearDistance) const;



      /* The proportion of track points that are within the specified distance of the target waypoint.
       * This takes into account both vertical and horizontal distance.
       * Throws a std::domain_error if the track contains zero track points.
       * Throws a std::invalid_argument exception if the specified distance is negative.
       */
      fraction proportionOfWaypointsNear(Waypoint targetWaypoint, metres nearDistance) const;



      /* The waypoint on the track that comes immediately before the specified time.
       * Throws a std::domain_error if there are no track points before that time.
       * Throws a std::domain_error if the time elapsed between any two adjacent points is zero or negative.
       */
      Waypoint lastWaypointBefore(std::time_t targetTime) const;



      /* The waypoint on the track that comes immediately after the specified time.
       * Throws a std::domain_error if there are no track points after that time.
       * Throws a std::domain_error if the time elapsed between any two adjacent points is zero or negative.
       */
      Waypoint firstWaypointAfter(std::time_t targetTime) const;



      /* Total elapsed time between start and finish of the track that is spent resting.
       * The parameter is the maximum speed for a period between points to count as 'resting',
       * taking into account both vertical and horizontal distance.
       * Throws a std::domain_error if the track contains zero track points.
       * Throws a std::domain_error if the time elapsed between any two adjacent points is zero or negative.
       */
      seconds restingTime(speed restingSpeedThreshold) const;



      /* Total elapsed time between start and finish of the track that is spent travelling.
       * The parameter is the minimum speed required for a period between points to count as 'travelling',
       * taking into account both vertical and horizontal distance.
       * Throws a std::domain_error if the track contains zero track points.
       * Throws a std::domain_error if the time elapsed between any two adjacent points is zero or negative.
       */
      seconds travellingTime(speed travellingSpeedThreshold) const;



      /* The duration of the longest resting period spent during the track.
       * The parameter is the maximum speed for a period between points to count as 'resting',
       * taking into account both vertical and horizontal distance.
       * (A resting period may span several track points.)
       * Returns zero if the track contains no resting periods.
       * Throws a std::domain_error if the track contains zero track points.
       * Throws a std::domain_error if the time elapsed between any two adjacent points is zero or negative.
       */
      seconds longestRestingPeriod(speed restingSpeedThreshold) const;



      /* The duration of the longest travelling period during the track.
       * The parameter is the minimum speed required for a period between points to count as 'travelling',
       * taking into account both vertical and horizontal distance.
       * (A travelling period may span several track points.)
       * Returns zero if the track contains no travelling periods.
       * Throws a std::domain_error if the track contains zero track points.
       * Throws a std::domain_error if the time elapsed between any two adjacent points is zero or negative.
       */
      seconds longestTravellingPeriod(speed travellingSpeedThreshold) const;



      /* The mean duration of the resting periods within the track.
       * The parameter is the maximum speed for a period between points to count as 'resting',
       * taking into account both vertical and horizontal distance.
       * (Resting through several track points is considered to be a single resting period.)
       * Throws a std::domain_error if the track contains no resting periods.
       * Throws a std::domain_error if the time elapsed between any two adjacent points is zero or negative.
       */
      seconds averageRestingPeriod(speed restingSpeedThreshold) const;



      /* The mean duration of the travelling periods within the track.
       * The parameter is the minimum speed required for a period between points to count as 'travelling',
       * taking into account both vertical and horizontal distance.
       * (Travelling through several track points is considered to be a single travelling period.)
       * Throws a std::domain_error if the track contains no travelling periods.
       * Throws a std::domain_error if the time elapsed between any two adjacent points is zero or negative.
       */
      seconds averageTravellingPeriod(speed travellingSpeedThreshold) const;



      /* The proportion of time during the track that is spent resting.
       * The parameter is the maximum speed for a period between points to count as 'resting',
       * taking into account both vertical and horizontal distance.
       * Throws a std::domain_error if the track contains fewer than two track points.
       * Throws a std::domain_error if the time elapsed between any two adjacent points is zero or negative.
       */
      fraction proportionRestingTime(speed travellingSpeedThreshold) const;



      /* The proportion of time during the track that is spent travelling.
       * The parameter is the minimum speed required for a period between points to count as 'travelling',
       * taking into account both vertical and horizontal distance.
       * Throws a std::domain_error if the track contains fewer than two track points.
       * Throws a std::domain_error if the time elapsed between any two adjacent points is zero or negative.
       */
      fraction proportionTravellingTime(speed travellingSpeedThreshold) const;



      /* The mean speed of all the time spent travelling over the course of the track, taking into
       * account both vertical and horizontal distance travelled.
       * The parameter is the minimum speed required for a period between points to count as 'travelling'.
       * Throws a std::domain_error if the track does not contain any travelling periods.
       * Throws a std::domain_error if the time elapsed between any two adjacent points is zero or negative.
       */
      speed averageTravellingSpeed(speed travellingSpeedThreshold) const;



      /* The duration of time on the track before travelling begins.
       * The parameter is the minimum speed required for a period between points to count as 'travelling',
       * taking into account both vertical and horizontal distance.
       * Throws a std::domain_error if the track does not contain any travelling periods.
       * Throws a std::domain_error if the time elapsed between any two adjacent points is zero or negative.
       */
      seconds durationBeforeTravellingBegins(speed speedLimit) const;

  };
}

#endif
