#ifndef POINTCLOUDBUILDER_H_
#define POINTCLOUDBUILDER_H_

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>

#include <sensor_msgs/PointCloud2.h>

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace pcl;

typedef PointCloud<PointXYZRGB> PointCloudXYZRGB;

class PointCloudBuilder
{
public:
    PointCloudBuilder(const string& extrFile);
    virtual ~PointCloudBuilder();

    void extractFrom(Mat& dispMap, Mat& leftColorImage, bool handleMissingValues = false);
    PointCloud2Ptr getPointCloud();

private:
    Mat Q;
    Mat pointCloud;
    PointCloudXYZRGB::Ptr pointCloudMsg;
    PointCloud2Ptr pcMsg;
};

#endif /* POINTCLOUDBUILDER_H_ */
