#ifndef IMU_H_
#define IMU_H_

#include <geometry_msgs/PoseStamped.h>
#include <cstdio>
#include <cmath>

#include "cmt3.h"
#include "cmtdef.h"
#include "xsens_time.h"
#include "xsens_list.h"
#include "cmtscan.h"

using namespace std;
using namespace geometry_msgs;
using namespace xsens;

/* UTM Structure */
typedef struct
{
    double UTM_E; /* East direction (in meters) */
    double UTM_N; /* North direction (in meters) */
    double UTM_Alt; /* Altitude (in meters) */
} UTM;

class IMU
{
public:
    IMU();
    virtual ~IMU();

    void update();
    static void convertEulerToQuartenion(Pose& eulerPose, Pose& quartenionPose);

    PoseStampedPtr getQuartenionPose();
    PoseStampedPtr getEulerPose();

private:
    PoseStampedPtr quartenionPose;
    PoseStampedPtr eulerPose;

    Cmt3 cmt3;
    Packet* packet;

    int doHardwareScan(Cmt3 &cmt3, CmtDeviceId deviceIds[]);
    bool doMtSettings(Cmt3 &cmt3, CmtOutputMode &mode, CmtOutputSettings &settings, CmtDeviceId deviceIds[]);
    UTM coordinatesConversion(double lat, double lon, double alt);

};

#endif /* IMU_H_ */
