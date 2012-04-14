#include <ros/ros.h>
#include <StereoVision/DispPointCloudIMU.h>
#include <IMUSensor/FPSPrinter.h>
#include <iostream>

#include "Mapping/PointCloudLinker.h"

#define PERIOD 20
#define RESOLUTION 0.2
#define INCLINATION -45

using namespace ros;
using namespace std;
using namespace StereoVision;

PointCloudLinker *pcLinker;
DispPointCloudIMUPtr msg(new DispPointCloudIMU);

FPSPrinter fpsp;
bool executing = false;

void callback(const DispPointCloudIMUConstPtr& pc);

int main(int argc, char *argv[])
{
    if(argc != 2)
    {
        cout << "Usage: " << argv[0] << " [map filename]" << endl;
        return EXIT_FAILURE;
    }

    init(argc, argv, "Mapping");

    pcLinker = new PointCloudLinker(PERIOD, RESOLUTION, INCLINATION, argv[1]);

    NodeHandle nh;
    Subscriber pcSub = nh.subscribe<DispPointCloudIMU >("/stereoVision/dispPointCloudIMU", 1, callback);

    spin();

    pcLinker->save();
    cout << "\n\nMap saved to " << argv[1] << endl;

    delete pcLinker;

    return EXIT_SUCCESS;
}

void callback(const DispPointCloudIMUConstPtr& pc)
{
    if(pcLinker->isReady(pc->header.stamp) && !executing)
    {
        executing = true;
        *msg = *pc;
        pcLinker->link(msg);

        fpsp.print();
        executing = false;
    }
}
