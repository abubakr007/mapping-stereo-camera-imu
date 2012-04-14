#ifndef STEREORECTIFICATION_H_
#define STEREORECTIFICATION_H_

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>

#include <iostream>
#include <string>

using namespace cv;
using namespace std;

class StereoRectification
{
public:
    StereoRectification(const string& extrFile, const string& intrFile, int camWidth, int camHeight, int width, int height);
    virtual ~StereoRectification();

    void rectify(Mat& leftImage, Mat& rightImage, Mat& leftRectImage, Mat& rightRectImage);

    int getCamWidth();
    int getCamHeight();
    int getWidth();
    int getHeight();

private:
    int camWidth, camHeight;
    int width, height;

    Mat Q;
    Mat map11, map12, map21, map22;

};

#endif /* STEREORECTIFICATION_H_ */
