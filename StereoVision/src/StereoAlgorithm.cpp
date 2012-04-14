#include "StereoVision/StereoAlgorithm.h"

StereoAlgorithm::StereoAlgorithm(int algorithm, int numberOfDisparities, int windowSize)
{
    this->algorithm = algorithm;
    this->numberOfDisparities = numberOfDisparities;
    this->windowSize = windowSize;

    switch (algorithm)
    {
        case BM:
            bm.state->preFilterCap = 31;
            bm.state->SADWindowSize = windowSize > 0 ? windowSize : 15;
            bm.state->minDisparity = 0;
            bm.state->numberOfDisparities = numberOfDisparities;
            bm.state->textureThreshold = 10;
            bm.state->uniquenessRatio = 15;
            bm.state->speckleWindowSize = 100;
            bm.state->speckleRange = 32;
            bm.state->disp12MaxDiff = 1;
            break;

        case SGBM:
        case HH:
            sgbm.preFilterCap = 63;
            sgbm.SADWindowSize = windowSize > 0 ? windowSize : 9;
            sgbm.P1 = 8 * sgbm.SADWindowSize * sgbm.SADWindowSize;
            sgbm.P2 = 32 * sgbm.SADWindowSize * sgbm.SADWindowSize;
            sgbm.minDisparity = 0;
            sgbm.numberOfDisparities = numberOfDisparities;
            sgbm.uniquenessRatio = 10;
            sgbm.speckleWindowSize = 100;
            sgbm.speckleRange = 32;
            sgbm.disp12MaxDiff = 1;
            sgbm.fullDP = algorithm == HH;
            break;

        default:
            cout << "Unknown stereo algorithm" << endl;
            exit(1);
    }
}

StereoAlgorithm::~StereoAlgorithm()
{
}

int StereoAlgorithm::getNumberOfDisparities()
{
    return numberOfDisparities;
}

void StereoAlgorithm::execute(Mat& leftImage, Mat& rightImage, Mat& dispMap)
{
    switch (algorithm)
    {
        case BM:
            bm(leftImage, rightImage, dispMap);
            break;

        case SGBM:
        case HH:
            sgbm(leftImage, rightImage, dispMap);
            break;
    }
}
