#include <IMUSensor/FPSPrinter.h>
#include <ros/ros.h>

#include "StereoCapture/StereoCamera.h"
#include "StereoCapture/StereoRectification.h"
#include "StereoCapture/StereoPair.h"

#define EXTR_FILE "config/extrinsics.yml"
#define INTR_FILE "config/intrinsics.yml"

#define CAM_WIDTH 640
#define CAM_HEIGHT 480

#define WIDTH 400
#define HEIGHT 300

#define FPS 30

using namespace ros;
using namespace StereoCapture;


int main(int argc, char **argv)
{
    StereoRectification *rectification = new StereoRectification(EXTR_FILE, INTR_FILE, CAM_WIDTH, CAM_HEIGHT, WIDTH, HEIGHT);
    StereoCamera *camera = new StereoCamera(rectification);

    init(argc, argv, "StereoCapture");

    NodeHandle nh;
    Publisher pairPublisher = nh.advertise<StereoPair>("stereoCapture/stereoPair", 1);

    StereoPairPtr pair;

    Rate rate(FPS);
    FPSPrinter fpsp;

    while(ok())
    {
        camera->nextFrame();

        pair->header.stamp = Time::now();
        pair->leftImage = *camera->getLeftImage();
        pair->rightImage = *camera->getRightImage();
        pairPublisher.publish(pair);

        spinOnce();

        fpsp.print();

        rate.sleep();
    }

    delete rectification;
    delete camera;

    return 0;
}

