#include <iostream>
#include <fstream>
#include <filesystem>

#include "gpx-parser.h"
#include "track.h"

using namespace GPS;

using std::cout;
using std::endl;

int main()
{
    std::string filepath = "../data/NorthYorkMoors.gpx";

    if (! std::filesystem::exists(filepath))
    {
        cout << "Could not open log file: " + filepath << endl;
        cout << "(If you're running at the command-line, you need to 'cd' into the 'bin/' directory first.)" << endl;
    }
    else
    {

        std::fstream gpxData {filepath};

        Track exampleTrack = GPX::parseTrackStream(gpxData);

        cout << "Number of waypoints: " << exampleTrack.numberOfWaypoints() << endl;

        cout << "Total time: " << exampleTrack.totalTime() << "s" << endl;

        cout << "Net height gain: " << exampleTrack.netHeightGain() << "m" << endl;

        cout << "Total height gain: " << exampleTrack.totalHeightGain() << "m" << endl;

        cout << "Net length: " << exampleTrack.netLength() << "m" << endl;

        cout << "Total length: " << exampleTrack.totalLength() << "m" << endl;

        cout << "Average speed: " << exampleTrack.averageSpeed() << "m/s" << endl;

        Waypoint highestPoint = exampleTrack.highestWaypoint();
        cout << "Highest waypoint: " << highestPoint.latitude() << "o lat by "
             << highestPoint.longitude() << "o lon at altitude "
             << highestPoint.altitude() << "m" << endl;

        Waypoint lowestPoint = exampleTrack.lowestWaypoint();
        cout << "Lowest waypoint: " << lowestPoint.latitude() << "o lat by "
             << lowestPoint.longitude() << "o lon at altitude "
             << lowestPoint.altitude() << "m" << endl;
    }
}
