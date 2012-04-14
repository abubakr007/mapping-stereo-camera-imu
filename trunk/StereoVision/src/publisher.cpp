#include <ros/ros.h>
#include <pthread.h>
#include <semaphore.h>
#include <StereoCapture/StereoPair.h>
#include <IMUSensor/FPSPrinter.h>

#include "StereoVision/StereoAlgorithm.h"
#include "StereoVision/DisparityMap.h"
#include "StereoVision/PointCloudBuilder.h"
#include "StereoVision/DispPointCloud.h"

#define Q_MATRIX_FILE "config/extrinsics.yml"

using namespace ros;
using namespace sensor_msgs;
using namespace StereoVision;
using namespace StereoCapture;

StereoAlgorithm *algorithm;
DisparityMap *dispMap;
PointCloudBuilder *pointCloud;

Mat rightImage, leftImage;
ImagePtr leftMsg(new Image), rightMsg(new Image);
CvBridge leftBridge, rightBridge;

Publisher dispPCPublisher;

bool executing = false;
sem_t buildMutex;

Time timestamp;
FPSPrinter fpsp;

void *build(void *arg);
void callback(const StereoPairConstPtr& pair);


int main(int argc, char **argv)
{
    pthread_t buildThread;

    sem_init(&buildMutex, 0, 0);

    algorithm = new StereoAlgorithm(StereoAlgorithm::HH, 32, 7);
    dispMap = new DisparityMap(algorithm);
    pointCloud = new PointCloudBuilder(Q_MATRIX_FILE);

    init(argc, argv, "StereoVision");

    NodeHandle nh;
    dispPCPublisher = nh.advertise<DispPointCloud>("stereoVision/dispPointCloud", 1);

    Subscriber pairSub = nh.subscribe<StereoPair>("/stereoCapture/stereoPair", 1, callback);

    pthread_create(&buildThread, NULL, build, NULL);

    spin();

    delete algorithm;
    delete dispMap;
    delete pointCloud;

    return 0;
}

void callback(const StereoPairConstPtr& pair)
{
    if(!executing)
    {
        timestamp = pair->header.stamp;

        *leftMsg = pair->leftImage;
        *rightMsg = pair->rightImage;

        leftImage = leftBridge.imgMsgToCv(leftMsg, "bgr8");
        rightImage = rightBridge.imgMsgToCv(rightMsg, "bgr8");

        sem_post(&buildMutex);
    }
}

void *build(void *arg)
{
    DispPointCloudPtr pcMsg(new DispPointCloud);

    while(1)
    {
        sem_wait(&buildMutex);

        executing = true;

        dispMap->extractFrom(leftImage, rightImage);
        pointCloud->extractFrom(dispMap->getDispMapMatrix(), leftImage);

        pcMsg->header.stamp = timestamp;
        pcMsg->dispMap = *dispMap->getDisparityMap();
        pcMsg->pointCloud = *pointCloud->getPointCloud();

        dispPCPublisher.publish(pcMsg);

        executing = false;

        fpsp.print();
    }

    return 0;
}
