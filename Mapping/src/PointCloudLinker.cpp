#include "Mapping/PointCloudLinker.h"

PointCloudLinker::PointCloudLinker(int period, float resolution, float inclination, const string& outputFile)
    : map(resolution)
{
    this->period = period;
    this->inclination = inclination;
    this->outputFile = outputFile;
    first = true;
}

PointCloudLinker::~PointCloudLinker()
{
}

bool PointCloudLinker::isReady(Time timestamp)
{
    if(first)
    {
        begin = timestamp;
        duration = Duration(0);
        return true;
    }
    else
    {
        duration += (timestamp - begin);
        if(duration.toSec() >= period)
        {
            duration -= Duration(period);
            begin = timestamp;
            return true;
        }
        return false;
    }
}

void PointCloudLinker::link(DispPointCloudIMUPtr& msg)
{
    if(first)
    {
        relative.x = msg->pose.eulerPose.pose.position.x;
        relative.y = msg->pose.eulerPose.pose.position.y;
        relative.z = msg->pose.eulerPose.pose.position.z;
        first = false;
    }
    
    PointCloudXYZRGB pc;
    fromROSMsg(msg->pointCloud.pointCloud, pc);

    PointXYZRGB sensorOrigin;
    sensorOrigin.x = sensorOrigin.y = sensorOrigin.z = 0;

    PointXYZRGB frameOriginTrans;
    frameOriginTrans.x = msg->pose.eulerPose.pose.position.x - relative.x;
    frameOriginTrans.y = msg->pose.eulerPose.pose.position.y - relative.y;
    frameOriginTrans.z = msg->pose.eulerPose.pose.position.z - relative.z;

    
    printf("\n\nbefore\n");
    printf("x: %.2f\n", msg->pose.quartenionPose.pose.orientation.x);
    printf("y: %.2f\n", msg->pose.quartenionPose.pose.orientation.y);
    printf("z: %.2f\n", msg->pose.quartenionPose.pose.orientation.z);
    printf("w: %.2f\n", msg->pose.quartenionPose.pose.orientation.w);
    
    msg->pose.eulerPose.pose.orientation.x += inclination;
    IMU::convertEulerToQuartenion(msg->pose.eulerPose.pose, msg->pose.quartenionPose.pose);
    
    printf("\nafter\n");
    printf("x: %.2f\n", msg->pose.quartenionPose.pose.orientation.x);
    printf("y: %.2f\n", msg->pose.quartenionPose.pose.orientation.y);
    printf("z: %.2f\n", msg->pose.quartenionPose.pose.orientation.z);
    printf("w: %.2f\n\n", msg->pose.quartenionPose.pose.orientation.w);

    tf::Quaternion frameOriginRot;
    quaternionMsgToTF(msg->pose.quartenionPose.pose.orientation, frameOriginRot);

    map.insertScan<PointXYZRGB, tf::Quaternion>(pc, sensorOrigin, frameOriginTrans, frameOriginRot);
}

void PointCloudLinker::save()
{
    map.octree.writeBinary(outputFile);
}
