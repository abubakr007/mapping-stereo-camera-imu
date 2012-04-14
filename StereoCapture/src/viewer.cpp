#include <ros/ros.h>
#include <IMUSensor/FPSPrinter.h>

#include "StereoCapture/StereoPairIMU.h"

using namespace ros;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace StereoCapture;

Publisher leftImagePub;
Publisher rightImagePub;
Publisher imuPosePub;

FPSPrinter fpsp;

void callback(const StereoPairIMUConstPtr& pair);


int main(int argc, char **argv)
{
    init(argc, argv, "StereoCaptureViewer");

    NodeHandle nh;

    leftImagePub = nh.advertise<Image>("stereoCapture/leftImage", 1);
    rightImagePub = nh.advertise<Image>("stereoCapture/rightImage", 1);
    imuPosePub = nh.advertise<PoseStamped>("stereoCapture/imuPose", 1);

    //Subscriber pairSub = nh.subscribe<StereoPairIMU>("/stereoCapture/pairIMU", 1, callback);
    Subscriber pairSub = nh.subscribe<StereoPairIMU>("/stereoCaptureWithIMU/moment", 1, callback);

    spin();

    return 0;
}

void callback(const StereoPairIMUConstPtr& pair)
{
    ImagePtr left(new Image), right(new Image);
    PoseStampedPtr pose(new PoseStamped);

    *left = pair->pair.leftImage;
    *right = pair->pair.rightImage;
    /**pose = pair->pose.quartenionPose;*/
    *pose = pair->pose.eulerPose;

    leftImagePub.publish(left);
    rightImagePub.publish(right);
    imuPosePub.publish(pose);

    fpsp.print();
}
