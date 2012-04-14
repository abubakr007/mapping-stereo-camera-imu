#include <IMUSensor/IMU.h>
#include <IMUSensor/FPSPrinter.h>
#include <ros/ros.h>

#include "StereoCapture/StereoCamera.h"
#include "StereoCapture/StereoRectification.h"
#include "StereoCapture/StereoPairIMU.h"

#define EXTR_FILE "config/extrinsics.yml"
#define INTR_FILE "config/intrinsics.yml"

#define CAM_WIDTH 640
#define CAM_HEIGHT 480

#define WIDTH 400
#define HEIGHT 300

#define FPS 30

using namespace geometry_msgs;
using namespace ros;
using namespace StereoCapture;


int main(int argc, char *argv[])
{
    IMU *imu = new IMU();
    StereoRectification *rectification = new StereoRectification(EXTR_FILE, INTR_FILE, CAM_WIDTH, CAM_HEIGHT, WIDTH, HEIGHT);
    StereoCamera *camera = new StereoCamera(rectification);

    init(argc, argv, "StereoCapture");

    NodeHandle nh;
    Publisher pairPub = nh.advertise<StereoPairIMU>("stereoCapture/pairIMU", 1);

    StereoPairIMUPtr pair(new StereoPairIMU);

    Rate rate(FPS);
    FPSPrinter fpsp;

    while(ok())
    {
        camera->nextFrame();
        imu->update();

        pair->header.stamp = Time::now();
        pair->pose.eulerPose = *imu->getEulerPose();
        pair->pose.quartenionPose = *imu->getQuartenionPose();
        pair->pair.leftImage = *camera->getLeftImage();
        pair->pair.rightImage = *camera->getRightImage();
        pairPub.publish(pair);

        spinOnce();

        fpsp.print();

        rate.sleep();
    }

    delete imu;
    delete rectification;
    delete camera;

    return 0;
}

