#include "StereoVision/DisparityMap.h"

DisparityMap::DisparityMap(StereoAlgorithm *algorithm)
{
    this->algorithm = algorithm;
}

DisparityMap::~DisparityMap()
{
}

void DisparityMap::extractFrom(Mat& leftImage, Mat& rightImage)
{
    algorithm->execute(leftImage, rightImage, dispMap);
    dispMap.convertTo(monoDispMap, CV_8U, 255 / (algorithm->getNumberOfDisparities() * 16.));
}

Mat& DisparityMap::getDispMapMatrix()
{
    return monoDispMap;
}

Image::Ptr DisparityMap::getDisparityMap()
{
    IplImage ipl = monoDispMap;
    Image::Ptr img = sensor_msgs::CvBridge::cvToImgMsg(&ipl);
    return img;
}
