#ifndef DISPARITYMAP_H_
#define DISPARITYMAP_H_

#include "StereoAlgorithm.h"

#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>

using namespace std;
using namespace cv;
using namespace sensor_msgs;

class DisparityMap
{

public:
    DisparityMap(StereoAlgorithm *algorithm);
    virtual ~DisparityMap();

    void extractFrom(Mat& leftImage, Mat& rightImage);
    Mat& getDispMapMatrix();
    Image::Ptr getDisparityMap();

private:
    StereoAlgorithm *algorithm;

    Mat dispMap, monoDispMap;

};

#endif /* DISPARITYMAP_H_ */
