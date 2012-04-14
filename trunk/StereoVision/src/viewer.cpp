#include <ros/ros.h>
#include <IMUSensor/FPSPrinter.h>

#include "StereoVision/DispPointCloudIMU.h"

using namespace ros;
using namespace sensor_msgs;
using namespace StereoVision;

Publisher dispMapPub;
Publisher pointCloudPub;

FPSPrinter fpsp;

void callback(const DispPointCloudIMUConstPtr& pcimu);


int main(int argc, char **argv)
{
    init(argc, argv, "StereoVisionViewer");

    NodeHandle nh;

    dispMapPub = nh.advertise<Image>("stereoVision/dispMap", 1);
    pointCloudPub = nh.advertise<PointCloud2>("stereoVision/pointCloud", 1);

    Subscriber pcSub = nh.subscribe<DispPointCloudIMU>("/stereoVision/dispPointCloudIMU", 1, callback);

    spin();

    return 0;
}

void callback(const DispPointCloudIMUConstPtr& pcimu)
{
    ImagePtr disp(new Image);
    PointCloud2Ptr pc(new PointCloud2);

    *disp = pcimu->pointCloud.dispMap;
    *pc = pcimu->pointCloud.pointCloud;

    dispMapPub.publish(disp);
    pointCloudPub.publish(pc);

    fpsp.print();
}
