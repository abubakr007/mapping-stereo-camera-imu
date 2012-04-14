#include <stdio.h>
#include <ros/ros.h>

#include "IMUSensor/IMU.h"
#include "IMUSensor/FPSPrinter.h"
#include "IMUSensor/IMUPose.h"

#define FPS 30

using namespace geometry_msgs;
using namespace ros;
using namespace IMUSensor;

int main(int argc, char *argv[])
{
    IMU *imu = new IMU();

    init(argc, argv, "IMU");

    NodeHandle nh;
    Publisher posePub = nh.advertise<IMUPose>("IMU/pose", 1);

    IMUPosePtr pose(new IMUPose);

    Rate rate(FPS);
    FPSPrinter fpsp;

    while(ok())
    {
        imu->update();

        pose->eulerPose = *imu->getEulerPose();
        pose->quartenionPose = *imu->getQuartenionPose();

        printf("position:\n");
        printf("\tx: %lf\n", pose->eulerPose.pose.position.x);
        printf("\ty: %lf\n", pose->eulerPose.pose.position.y);
        printf("\tz: %lf\n\n", pose->eulerPose.pose.position.z);

        printf("orientation:\n");
        printf("\tx: %lf\n", pose->eulerPose.pose.orientation.w);
        printf("\tx: %lf\n", pose->eulerPose.pose.orientation.x);
        printf("\ty: %lf\n", pose->eulerPose.pose.orientation.y);
        printf("\tz: %lf\n\n\n", pose->eulerPose.pose.orientation.z);

        pose->header.stamp = Time::now();
        posePub.publish(pose);

        spinOnce();
        
        fpsp.print();
        
        rate.sleep();
    }

    delete imu;

    return 0;
}

