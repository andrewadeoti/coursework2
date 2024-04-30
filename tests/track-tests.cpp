#define BOOST_TEST_MODULE GPS_Test
#include <boost/test/included/unit_test.hpp>
#include "track.h"
#include "trackpoint.h"
#include "waypoint.h"
#include <ctime>

using namespace GPS;

BOOST_AUTO_TEST_CASE(testMostEasterlyWaypoint)
{
    // Case 1: Empty track
    {
        std::vector<Trackpoint> emptyTrackPoints;
        Track emptyTrack(emptyTrackPoints);

        // Check if calling mostEasterlyWaypoint on an empty track throws a domain error
        BOOST_CHECK_THROW(emptyTrack.mostEasterlyWaypoint(), std::domain_error);
    }

    // Case 2: Track with single point at (0, 0)
    {
        Waypoint centralWaypoint(0.0, 0.0, 0.0);
        std::time_t timestamp = std::time(nullptr);
        Trackpoint centralPoint(centralWaypoint, timestamp);
        std::vector<Trackpoint> singlePointTrackPoints = {centralPoint};
        Track singlePointTrack(singlePointTrackPoints);

        // Check if calling mostEasterlyWaypoint on a track with a single central point throws a domain error
        BOOST_CHECK_THROW(singlePointTrack.mostEasterlyWaypoint(), std::domain_error);
    }

    // Case 3: Track with multiple points including one at (0, 90)
    {
        Waypoint centralWaypoint(0.0, 0.0, 0.0);
        Waypoint eastWaypoint(0.0, 90.0, 0.0);
        std::time_t timestamp = std::time(nullptr);
        Trackpoint centralPoint(centralWaypoint, timestamp);
        Trackpoint eastPoint(eastWaypoint, timestamp);
        std::vector<Trackpoint> multiplePointsTrackPoints = {centralPoint, eastPoint};
        Track multiplePointsTrack(multiplePointsTrackPoints);

        // Get the most easterly waypoint
        Waypoint mostEasterly = multiplePointsTrack.mostEasterlyWaypoint();

        // Check if the most easterly waypoint is indeed the expected one
        BOOST_CHECK_CLOSE(mostEasterly.longitude(), 90.0, 0.01); // Reduced tolerance to 0.01 degrees
    }
}


