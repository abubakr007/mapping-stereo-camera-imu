#include "StereoCapture/StereoCamera.h"

StereoCamera::StereoCamera(StereoRectification* rectification)
{
    int camWidth = rectification->getCamWidth();
    int camHeight = rectification->getCamHeight();

    this->rectification = rectification;

    leftUnrectC = new Mat(Size(camWidth, camHeight), CV_8UC3);
    rightUnrectC = new Mat(Size(camWidth, camHeight), CV_8UC3);
    leftCAux = new Mat(Size(camWidth, camHeight), CV_8UC4);
    rightCAux = new Mat(Size(camWidth, camHeight), CV_8UC4);

    videoObject = getVideoObject();

    if(!videoObject->Open())
    {
        cout << "Failed to open stereo camera" << endl;
        exit(1);
    }

    videoObject->SetColor(true, true);
    videoObject->SetFrameDiv(1);
    videoObject->SetSize(camWidth, camHeight);
    videoObject->Start();
}

StereoCamera::~StereoCamera()
{
    videoObject->Close();
    videoObject->Stop();

    delete leftUnrectC;
    delete rightUnrectC;
    delete leftCAux;
    delete rightCAux;
}

void StereoCamera::nextFrame()
{
    stereoImage = videoObject->GetImage(500);

    leftCAux->data = (uchar *) stereoImage->color;
    rightCAux->data = (uchar *) stereoImage->color_right;
    cvtColor(*leftCAux, *leftUnrectC, CV_BGRA2RGB);
    cvtColor(*rightCAux, *rightUnrectC, CV_BGRA2RGB);

    rectification->rectify(*leftUnrectC, *rightUnrectC, leftCImage, rightCImage);
}

Image::Ptr StereoCamera::getLeftImage()
{
    return toMessage(leftCImage);
}

Image::Ptr StereoCamera::getRightImage()
{
    return toMessage(rightCImage);
}

Image::Ptr StereoCamera::toMessage(Mat& image)
{
    IplImage ipl = image;
    Image::Ptr msg = CvBridge::cvToImgMsg(&ipl, "bgr8");
    return msg;
}

