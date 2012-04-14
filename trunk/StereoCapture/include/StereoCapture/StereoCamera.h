#ifndef STEREOCAMERA_H_
#define STEREOCAMERA_H_

#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>

#include "StereoRectification.h"
#include "svsclass.h"

using namespace std;
using namespace cv;
using namespace sensor_msgs;

class StereoCamera
{
public:
    StereoCamera(StereoRectification* rectification);
    virtual ~StereoCamera();

    void nextFrame();

    Image::Ptr getLeftImage();
    Image::Ptr getRightImage();

private:

    Image::Ptr toMessage(Mat& image);

    svsVideoImages* videoObject;
    svsAcquireImages* sourceObject;
    svsStereoImage* stereoImage;

    Mat leftCImage, rightCImage;
    Mat *leftUnrectC, *rightUnrectC;
    Mat *leftCAux, *rightCAux;

    StereoRectification* rectification;
};

#endif /* STEREOCAMERA_H_ */
