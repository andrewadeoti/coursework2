#define BOOST_TEST_MODULE GPS_Test
#include <boost/test/included/unit_test.hpp>
#include "track.h"
#include "trackpoint.h"
#include "waypoint.h"

using namespace GPS;

BOOST_AUTO_TEST_CASE(testMostEasterlyWaypoint)
{
    // Case 1: Track contains zero track points
    {
        std::vector<Trackpoint> trackPoints;
        Track track(trackPoints);

        BOOST_CHECK_THROW(track.mostEasterlyWaypoint(), std::domain_error);
    }

    // Case 2: Two points equally farthest East
    {
        // Creating two points equally farthest East
        Trackpoint eastPoint1(Waypoint(0.0, 90.0, 0.0), std::time(nullptr));
        Trackpoint eastPoint2(Waypoint(0.0, 90.0, 0.0), std::time(nullptr));

        // Creating a track with the two points
        std::vector<Trackpoint> trackPoints = {eastPoint1, eastPoint2};
        Track track(trackPoints);

        // Checking if the first of the two equally farthest East points is returned
        Waypoint mostEasterly = track.mostEasterlyWaypoint();
        BOOST_CHECK_EQUAL(mostEasterly.longitude(), 90.0);
    }

    // Case 3: Track contains multiple points with different longitudes
    {
        // Creating three track points with different longitudes
        Trackpoint westPoint(Waypoint(0.0, -90.0, 0.0), std::time(nullptr));
        Trackpoint centralPoint(Waypoint(0.0, 0.0, 0.0), std::time(nullptr));
        Trackpoint eastPoint(Waypoint(0.0, 90.0, 0.0), std::time(nullptr));

        // Creating a track with the track points
        std::vector<Trackpoint> trackPoints = {westPoint, centralPoint, eastPoint};
        Track track(trackPoints);

        // Checking if the most easterly waypoint is indeed the expected one
        Waypoint mostEasterly = track.mostEasterlyWaypoint();
        BOOST_CHECK_EQUAL(mostEasterly.longitude(), 90.0);
    }
}
