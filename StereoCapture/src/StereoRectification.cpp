#include "StereoCapture/StereoRectification.h"

StereoRectification::StereoRectification(const string& extrFile, const string& intrFile, int camWidth, int camHeight, int width, int height)
{
    this->camWidth = camWidth;
    this->camHeight = camHeight;
    this->width = width;
    this->height = height;

    Size imageSize = cvSize(camWidth, camHeight);
    Size newSize = cvSize(width, height);

    FileStorage fs(intrFile, CV_STORAGE_READ);
    if(!fs.isOpened())
    {
        cout << "Failed to open file " << extrFile << endl;
        exit(1);
    }

    Mat M1, D1, M2, D2;

    fs["M1"] >> M1;
    fs["M2"] >> M2;
    fs["D1"] >> D1;
    fs["D2"] >> D2;

    fs.open(extrFile, CV_STORAGE_READ);
    if(!fs.isOpened())
    {
        cout << "Failed to open file " << intrFile << endl;
        exit(1);
    }

    Mat R, T, R1, P1, R2, P2;

    fs["R"] >> R;
    fs["T"] >> T;

    stereoRectify(M1, D1, M2, D2, imageSize, R, T, R1, R2, P1, P2, Q, 0, newSize);
    /*stereoRectify(M1, D1, M2, D2, imageSize, R, T, R1, R2, P1, P2, Q, 0, 0, newSize);*/
    initUndistortRectifyMap(M1, D1, R1, P1, newSize, CV_16SC2, map11, map12);
    initUndistortRectifyMap(M2, D2, R2, P2, newSize, CV_16SC2, map21, map22);
}

StereoRectification::~StereoRectification()
{

}

void StereoRectification::rectify(Mat& leftImage, Mat& rightImage, Mat& leftRectImage, Mat& rightRectImage)
{
    remap(leftImage, leftRectImage, map11, map12, INTER_LINEAR);
    remap(rightImage, rightRectImage, map21, map22, INTER_LINEAR);
}

int StereoRectification::getCamWidth()
{
    return camWidth;
}

int StereoRectification::getCamHeight()
{
    return camHeight;
}

int StereoRectification::getWidth()
{
    return width;
}

int StereoRectification::getHeight()
{
    return height;
}

