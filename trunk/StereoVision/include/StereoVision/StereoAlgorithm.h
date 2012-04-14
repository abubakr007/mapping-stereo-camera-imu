#ifndef STEREOALGORITHM_H_
#define STEREOALGORITHM_H_

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>

#include <iostream>

using namespace std;
using namespace cv;

class StereoAlgorithm
{

public:
    StereoAlgorithm(int algorithm, int numberOfDisparities, int windowSize);
    virtual ~StereoAlgorithm();

    void execute(Mat& leftImage, Mat& rightImage, Mat& dispMap);

    int getNumberOfDisparities();

    enum
    {
        BM = 0, SGBM = 1, HH = 2
    };

private:
    int algorithm;
    int numberOfDisparities, windowSize;

    StereoBM bm;
    StereoSGBM sgbm;
};

#endif /* STEREOALGORITHM_H_ */
