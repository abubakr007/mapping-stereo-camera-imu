#include "IMUSensor/IMU.h"

/* WGS84 Parameters */
#define    WGS84_A 6378137.0
#define    WGS84_B 6356752.31424518
#define    WGS84_F 0.0033528107
#define    WGS84_E 0.0818191908
#define    WGS84_EP 0.0820944379

/* UTM Parameters */
#define   UTM_K0 0.9996
#define   UTM_FE 500000.0
#define   UTM_FN_N 0.0
#define   UTM_FN_S 10000000.0
#define   UTM_E2 (WGS84_E*WGS84_E)
#define   UTM_E4 ((WGS84_E*WGS84_E)*(WGS84_E*WGS84_E))
#define   UTM_E6 (((WGS84_E*WGS84_E)*(WGS84_E*WGS84_E))*(WGS84_E*WGS84_E))
#define   UTM_EP2 ((WGS84_E*WGS84_E)/(1-(WGS84_E*WGS84_E)))

IMU::IMU() : quartenionPose(new PoseStamped), eulerPose(new PoseStamped)
{
    CmtOutputMode mode;
    CmtOutputSettings settings;
    CmtDeviceId deviceIds[256];

    settings = CMT_OUTPUTSETTINGS_ORIENTMODE_EULER;
    mode = CMT_OUTPUTMODE_ORIENT | CMT_OUTPUTMODE_POSITION;

    int mtCount = doHardwareScan(cmt3, deviceIds);
    if(mtCount == 0)
    {
        printf("IMU not found\n");
        exit(1);
    }

    if(!doMtSettings(cmt3, mode, settings, deviceIds))
    {
        printf("Couldn't set the settings");
        exit(1);
    }

    packet = new Packet((unsigned short) mtCount, cmt3.isXm());
}

IMU::~IMU()
{
    delete packet;
    cmt3.closePort();
}

/**
 * coordinatesConversion
 * lat : Latitude (in degrees)
 * lon : Longitude (in degrees)
 * alt : Altitude (in meters)
 */
UTM IMU::coordinatesConversion(double lat, double lon, double alt)
{
    UTM ret;

    // constants
    double m0 = (1 - UTM_E2 / 4 - 3 * UTM_E4 / 64 - 5 * UTM_E6 / 256);
    double m1 = -(3 * UTM_E2 / 8 + 3 * UTM_E4 / 32 + 45 * UTM_E6 / 1024);
    double m2 = (15 * UTM_E4 / 256 + 45 * UTM_E6 / 1024);
    double m3 = -(35 * UTM_E6 / 3072);

    // compute the central meridian
    int cm = (lon >= 0.0) ? ((int) lon - ((int) lon) % 6 + 3) : ((int) lon - ((int) lon) % 6 - 3);

    // convert degrees into radians
    double rlat = lat * M_PI / 180;
    double rlon = lon * M_PI / 180;
    double rlon0 = cm * M_PI / 180;

    // compute trigonometric functions
    double slat = sin(rlat);
    double clat = cos(rlat);
    double tlat = tan(rlat);

    // decide the flase northing at origin
    double fn = (lat > 0) ? UTM_FN_N : UTM_FN_S;

    double T = tlat * tlat;
    double C = UTM_EP2 * clat * clat;
    double A = (rlon - rlon0) * clat;
    double M = WGS84_A * (m0 * rlat + m1 * sin(2 * rlat) + m2 * sin(4 * rlat) + m3 * sin(6 * rlat));
    double V = WGS84_A / sqrt(1 - UTM_E2 * slat * slat);

    // compute the easting-northing coordinates
    ret.UTM_E = UTM_FE + UTM_K0 * V * (A + (1 - T + C) * pow(A, 3) / 6 + (5 - 18 * T + T * T + 72 * C - 58 * UTM_EP2) * pow(A, 5) / 120);
    ret.UTM_N = fn + UTM_K0 * (M + V * tlat * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * pow(A, 4) / 24 + (61 - 58 * T + T * T + 600 * C - 330 * UTM_EP2) * pow(A, 6) / 720));
    ret.UTM_Alt = alt;

    return ret;
}

void IMU::convertEulerToQuartenion(Pose& eulerPose, Pose& quartenionPose)
{
    float cos_z_2 = cos(eulerPose.orientation.z*M_PI/360);
    float cos_y_2 = cos(eulerPose.orientation.y*M_PI/360);
    float cos_x_2 = cos(eulerPose.orientation.x*M_PI/360);

    float sin_z_2 = sin(eulerPose.orientation.z*M_PI/360);
    float sin_y_2 = sin(eulerPose.orientation.y*M_PI/360);
    float sin_x_2 = sin(eulerPose.orientation.x*M_PI/360);

    quartenionPose.orientation.w = (cos_z_2*cos_y_2*cos_x_2) + (sin_z_2*sin_y_2*sin_x_2);
    quartenionPose.orientation.x = (cos_z_2*cos_y_2*sin_x_2) - (sin_z_2*sin_y_2*cos_x_2);
    quartenionPose.orientation.y = (cos_z_2*sin_y_2*cos_x_2) + (sin_z_2*cos_y_2*sin_x_2);
    quartenionPose.orientation.z = (sin_z_2*cos_y_2*cos_x_2) - (cos_z_2*sin_y_2*sin_x_2);
}

void IMU::update()
{
    CmtEuler euler_data;
    CmtVector positionLLA;

    cmt3.waitForDataMessage(packet);

    euler_data = packet->getOriEuler(0);
    eulerPose->pose.orientation.w = 0;
    eulerPose->pose.orientation.x = euler_data.m_roll;
    eulerPose->pose.orientation.y = euler_data.m_pitch;
    eulerPose->pose.orientation.z = euler_data.m_yaw;

    convertEulerToQuartenion(eulerPose->pose, quartenionPose->pose);

    positionLLA = packet->getPositionLLA();
    UTM utm = coordinatesConversion(positionLLA.m_data[0], positionLLA.m_data[1], positionLLA.m_data[2]);
    
    quartenionPose->pose.position.x = eulerPose->pose.position.x = utm.UTM_E;
    quartenionPose->pose.position.y = eulerPose->pose.position.y = utm.UTM_N;
    quartenionPose->pose.position.z = eulerPose->pose.position.z = utm.UTM_Alt;
}

PoseStampedPtr IMU::getQuartenionPose()
{
    return quartenionPose;
}

PoseStampedPtr IMU::getEulerPose()
{
    return eulerPose;
}

int IMU::doHardwareScan(Cmt3 &cmt3, CmtDeviceId deviceIds[])
{
    XsensResultValue res;
    List<CmtPortInfo> portInfo;
    unsigned long portCount = 0;
    int mtCount;

    xsens::cmtScanPorts(portInfo);
    portCount = portInfo.length();

    if(portCount == 0) return 0;

    //open the port which the device is connected to and connect at the device's baudrate.
    for(int p = 0; p < (int) portCount; p++)
    {
        res = cmt3.openPort(portInfo[p].m_portName, portInfo[p].m_baudrate); /* <<<<<<<<<<<<<<<<<<< Abre comunicacao de alto nivel com dispositivo */
        if(res != XRV_OK) return 0;
    }

    //get the Mt sensor count.
    mtCount = cmt3.getMtCount();

    // retrieve the device IDs
    for(int j = 0; j < mtCount; j++)
    {
        res = cmt3.getDeviceId((unsigned char) (j + 1), deviceIds[j]);
        if(res != XRV_OK) return 0;
    }

    return mtCount;
}

bool IMU::doMtSettings(Cmt3 &cmt3, CmtOutputMode &mode, CmtOutputSettings &settings, CmtDeviceId deviceIds[])
{
    XsensResultValue res;
    unsigned long mtCount = cmt3.getMtCount();

    // set sensor to config sate
    res = cmt3.gotoConfig();
    if(res != XRV_OK) return false;

    unsigned short sampleFreq;
    sampleFreq = cmt3.getSampleFrequency();

    // set the device output mode for the device(s)

    for(unsigned int i = 0; i < mtCount; i++)
    {
        CmtDeviceMode deviceMode(mode, settings, sampleFreq);
        if((deviceIds[i] & 0xFFF00000) != 0x00500000)
        {
            // not an MTi-G, remove all GPS related stuff
            deviceMode.m_outputMode &= 0xFF0F;
        }
        res = cmt3.setDeviceMode(deviceMode, true, deviceIds[i]);
        if(res != XRV_OK) return false;
    }

    // start receiving data
    res = cmt3.gotoMeasurement();
    if(res != XRV_OK) return false;

    return true;
}
