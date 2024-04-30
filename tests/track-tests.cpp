#define BOOST_TEST_MODULE TrackTests
#include <boost/test/included/unit_test.hpp>
#include "track.h"

// Test case for mostEasterlyWaypoint() function
BOOST_AUTO_TEST_CASE(TestMostEasterlyWaypoint)
{
    // Test case 1: Track with zero track points
    {
        GPS::Track emptyTrack({});
        BOOST_CHECK_THROW(emptyTrack.mostEasterlyWaypoint(), std::domain_error);
    }

    // Test case 2: Track with one track point
    {
        GPS::Waypoint singlePoint(0, 0, 0); // Latitude, Longitude, Altitude
        GPS::Track trackWithSinglePoint({singlePoint});
        BOOST_CHECK(trackWithSinglePoint.mostEasterlyWaypoint() == singlePoint);
    }

    // Test case 3: Track with multiple track points
    {
        GPS::Waypoint eastPoint(0, 90, 0);   // Farthest east
        GPS::Waypoint westPoint(0, -90, 0);  // Farthest west
        GPS::Waypoint middlePoint(0, 45, 0); // Middle longitude
        GPS::Track track({eastPoint, westPoint, middlePoint});
        BOOST_CHECK(track.mostEasterlyWaypoint() == eastPoint);
    }

    // Test case 4: Track with multiple track points having equal longitude but different timestamps
    {
        GPS::Waypoint firstPoint(0, 90, 0);  // Farthest east
        GPS::Waypoint secondPoint(0, 90, 0); // Farthest east
        GPS::Track track({firstPoint, secondPoint});
        BOOST_CHECK(track.mostEasterlyWaypoint() == firstPoint);
    }
}
