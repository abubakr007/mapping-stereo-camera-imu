#ifndef POINTCLOUDLINKER_H_
#define POINTCLOUDLINKER_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/transform_datatypes.h>
#include <octomap_ros/OctomapROS.h>
#include <octomap/OcTree.h>

#include <StereoVision/DispPointCloudIMU.h>
#include <IMUSensor/IMU.h>

#include <stdio.h>
#include <math.h>

using namespace std;
using namespace ros;
using namespace pcl;
using namespace tf;
using namespace octomap;
using namespace StereoVision;

typedef PointCloud<PointXYZRGB> PointCloudXYZRGB;

class PointCloudLinker
{
public:
    PointCloudLinker(int period, float resolution, float inclination, const string& outputFile);
    virtual ~PointCloudLinker();

    bool isReady(Time timestamp);
    void link(DispPointCloudIMUPtr& msg);
    void save();

private:
    int period;
    float inclination;
    string outputFile;

    OcTreeROS map;
    PointXYZRGB relative;

    Time begin;
    Duration duration;
    bool first;

};

#endif /* POINTCLOUDLINKER_H_ */
