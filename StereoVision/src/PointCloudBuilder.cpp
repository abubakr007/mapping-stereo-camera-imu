#include "StereoVision/PointCloudBuilder.h"

PointCloudBuilder::PointCloudBuilder(const string& extrFile) 
    : pointCloudMsg(new PointCloudXYZRGB), pcMsg(new PointCloud2)
{
    pointCloudMsg->header.frame_id = "/world";

    FileStorage fs(extrFile, CV_STORAGE_READ);
    if(!fs.isOpened())
    {
        cout << "Failed to open file " << extrFile << endl;
        exit(1);
    }

    fs["Q"] >> Q;
}

PointCloudBuilder::~PointCloudBuilder()
{
}

void PointCloudBuilder::extractFrom(Mat& dispMap, Mat& leftColorImage, bool handleMissingValues)
{
    reprojectImageTo3D(dispMap, pointCloud, Q, handleMissingValues);

    pointCloudMsg->points.clear();
    pointCloudMsg->height = pointCloud.rows;
    pointCloudMsg->width = pointCloud.cols;

    for(unsigned int i = 0; i < pointCloudMsg->height; i++)
    {
        Vec3f *point = pointCloud.ptr<Vec3f>(i);
        Vec3b *pixel = leftColorImage.ptr<Vec3b>(i);

        for(unsigned int j = 0; j < pointCloudMsg->width; j++)
        {
            PointXYZRGB p;

            p.data[0] = point[j][0]/2.46;
            p.data[1] = -point[j][1]/2.46;
            p.data[2] = point[j][2]/2.46;
            p.data[3] = 1.0f;

            if(isinf(p.x) || isnan(p.x) || isinf(p.y) || isnan(p.y) || isinf(p.z) || isnan(p.z))
                p.data[0] = p.data[1] = p.data[2] = 0;
            
            uint8_t r = pixel[j][0];
            uint8_t g = pixel[j][1];
            uint8_t b = pixel[j][2];
            int32_t rgb = (b << 16) | (g << 8) | r;
            p.rgb = *(float*) (&rgb);

            pointCloudMsg->points.push_back(p);
        }
    }
}

PointCloud2Ptr PointCloudBuilder::getPointCloud()
{
    pcl::toROSMsg(*pointCloudMsg, *pcMsg);
    return pcMsg;
}
