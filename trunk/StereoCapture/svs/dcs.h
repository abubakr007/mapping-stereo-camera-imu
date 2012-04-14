//
//
// Interface to synchronized DCAM IEEE 1394 cameras
//   based on Videre Design's firewire baseboard
// C++ interface
//
// Copyright 2001 by Kurt Konolige, SRI International
// konolige@ai.sri.com
//
// The author hereby grants to SRI permission to use this software.
// The author also grants to SRI permission to distribute this software
// to schools for non-commercial educational use only.
//
// The author hereby grants to other individuals or organizations
// permission to use this software for non-commercial
// educational use only.  This software may not be distributed to others
// except by SRI, under the conditions above.
//
// Other than these cases, no part of this software may be used or
// distributed without written permission of the author.
//
// Neither the author nor SRI make any representations about the 
// suitability of this software for any purpose.  It is provided 
// "as is" without express or implied warranty.
//
// Kurt Konolige
// Senior Computer Scientist
// SRI International
// 333 Ravenswood Avenue
// Menlo Park, CA 94025
// E-mail:  konolige@ai.sri.com
//
//


#include "dcam.h"    // this is the full version
#define LOCAL_PARAM_BASE 0xF0FFF000UL // should be in dcam.h

IMPORT
void do_speckle_filter(short *disp, int *labels, unsigned char *badreg, int *queue, 
		       int width, int size, int diff, int regcnt);


// Device capabilities

#define MAX_DEVICES 10

#define LEFTCAM "DCAMl"
#define RIGHTCAM "DCAMr"


#define VID_WIDTH 1280        // max image size
#define VID2_WIDTH (VID_WIDTH + VID_WIDTH)  // max here should be 1288 x 1032
#define VID_HEIGHT 960
#define MAX_WIDTH 1280        // what we allow images to be
#define MAX_HEIGHT 960
#define STOC_WIDTH 640
#define STOC_HEIGHT 480

//
// Class for video acquisition
// Subclasses svsAcquireVideo
//
// Some parameters are held locally, those that are particular to
//   the video acquisition class
// Other basic image parameters are held in ip
//


class svsDCSAcquireVideo : public svsVideoImages
{
public:
  IMPORT svsDCSAcquireVideo();
  IMPORT ~svsDCSAcquireVideo();

  // Acquisition device, overrides base class
  IMPORT bool Open(char *name = NULL); // opens the video device
  IMPORT bool Open(int devNum); // opens a particular video device
  IMPORT bool Close();        // closes the video device
  IMPORT bool SetCapture(int type); // Not used
  IMPORT bool SetFormat(int type);  // Not used
  IMPORT bool SetChannel(int type); // Not used
  IMPORT bool SetSwap(bool on);        // Not used
  IMPORT bool SetColor(bool on, bool onr = false);    // Turn on color images
  IMPORT bool SetColorAlg(int alg); // specify color algorithm
  IMPORT bool SetSize(int w, int h); // size of image
  IMPORT bool SetSample(int dec, int bin);  // image sampling
  IMPORT bool SetFrameDiv(int fdiv); // image sampling
  IMPORT bool SetOffset(int ix, int iy, int verge); // subwindow origin and vergence
  IMPORT bool SetProcMode(proc_mode_type mode); // sets the processing mode for STOC

  IMPORT bool SetExposure(bool is_auto, int exp, int gn);
  IMPORT bool SetExposure(int exp, int gn, bool auto_exp = false, bool auto_gn = false);
  IMPORT bool SetAutoExpParams(double auto_bias, double auto_kp = -1.0);
  IMPORT bool SetAutoExpFilter(int y0, int y1, int y2, int y3, int y4);
  IMPORT bool SetBalance(bool is_auto, int rd, int bl);
  IMPORT bool SetGamma(int gam);
  IMPORT bool SetBrightness(bool is_auto, int bright);
  IMPORT bool SetSaturation(int sat);
  IMPORT bool SetSharpness(int sharp);
  IMPORT bool SetLevel(int contr, int bright);
  IMPORT bool SetRate(int rate); // image frame rate, 30, 15, 7, 3
  IMPORT bool SetYOffset(int left, int right); // for changing vertical offset
  IMPORT bool SetHoropter(int offx); // for changing horopter on camera, STOC only
  IMPORT bool SaveLocalParams(); // save on-camera parameters

  // Start/stop video, check parameters
  IMPORT bool Start();
  IMPORT bool Stop();
  IMPORT bool CheckParams();

  // Set up digitization params, subwindow
  // Set values, then call these fns; can happen during streaming
  IMPORT void SetOffsets();
  IMPORT void SetDigitization();
  void ResetDigitization();	// resets CMU driver values

  // Image acquisition, gets the next available image with timeout
  IMPORT svsStereoImage *GetImage(int ms = 0);

  // type of device
  IMPORT int Enumerate();
  IMPORT char **DeviceIDs();
  int deviceNum;                // number of DCS stereo devices (including mono-coupled)
  enum { STH_MDS, STH_MDS_DUAL } devType;

  // device stuff
  char *svsDevName;             // name of camera
  static dSystem *sys;		// IEEE 1394 system object
  dCamera *leftCam;             // right and left camera objects
  dCamera *rightCam;		// use left camera for STH-MDS
  int leftDev, rightDev;        // camera device numbers
  bool chipbin;			// true if on-chip binning

  // user info from a device
  bool getUserInfo(char *bb, int n);
  bool setUserInfo(char *bb, int n);

  // reading/writing registers
  IMPORT int ReadImageReg(int reg, int which);
  IMPORT bool WriteImageReg(int reg, int val);

 protected:
  // parameters from a device
  bool getParams(char *bb, int n);
  bool setParams(char *bb, int n);

 private:
  // buffers for conversion
  WINALIGN16 unsigned char svsLeft[VID_WIDTH*VID_HEIGHT+16] ALIGN16;
  WINALIGN16 unsigned char svsRight[VID_WIDTH*VID_HEIGHT+16] ALIGN16;
  WINALIGN16 unsigned char svsLeft2[VID_WIDTH*VID_HEIGHT+16] ALIGN16;
  WINALIGN16 unsigned char svsRight2[VID_WIDTH*VID_HEIGHT+16] ALIGN16;
  WINALIGN16 unsigned char svsColor[VID_WIDTH*VID_HEIGHT*4 +64] ALIGN16; 
  WINALIGN16 unsigned char svsColor2[VID_WIDTH*VID_HEIGHT*4 + 64] ALIGN16;
  WINALIGN16 unsigned char svsColorR[VID_WIDTH*VID_HEIGHT*4 +64] ALIGN16;
  WINALIGN16 unsigned char svsColorR2[VID_WIDTH*VID_HEIGHT*4 + 64] ALIGN16;
  WINALIGN16 short         svsDisparity[STOC_WIDTH*STOC_HEIGHT + 64] ALIGN16;
  WINALIGN16 short         svsDisparity2[STOC_WIDTH*STOC_HEIGHT + 64] ALIGN16;
  WINALIGN16 unsigned char imbuf[VID_WIDTH*16 + 8] ALIGN16;
  WINALIGN16 int           specklebuf[VID_WIDTH*VID_HEIGHT+64];
  WINALIGN16 int           queuebuf[VID_WIDTH*VID_HEIGHT];
  WINALIGN16 unsigned char badregbuf[VID_WIDTH*VID_HEIGHT/4];
  WINALIGN16 unsigned char interbuf[VID_WIDTH*VID_HEIGHT*2];


  // pointers for DCAM image returns
  unsigned char *bufOrig, *bufOrig2, *bufLeft, *bufRight, *bufColor, *bufColorRight;
  short *bufDisparity;
  bool swapImages;              // true if we swap left/right

  // image functions
  bool set_size();

  // process a frame
  void check_acquire(int ms);
  bool running, first, got_it, end_it;
  int which;
  dISIZE size;                  // internal parameters for cameras
  dITYPE ltype;                 // left and right types can differ
  dITYPE rtype;
  dISPEED speed;

  // check STOC parameters
  int uniq_thresh, conf_thresh;

  // STH-MDCS de-interlacing
  void do_copy_2(unsigned char *dest1, unsigned char *dest2, int *cdest, int *cdest2,
                 unsigned char *src, int width, int height);
  void do_copy_2_BGGR(unsigned char *dest1, unsigned char *dest2, int *cdest, int *cdest2,
                 unsigned char *src, int width, int height);
  void do_copy_bin(unsigned char *dest1, unsigned char *dest2, unsigned char *color, 
                   unsigned char *color_right,
                   unsigned char *src, int linelen, int nlines);
  void do_copy_bin_BGGR(unsigned char *dest1, unsigned char *dest2, unsigned char *color, 
                   unsigned char *color_right,
                   unsigned char *src, int linelen, int nlines);

  // STOC device processing
  void do_copy_stoc(unsigned char *idest, short *ddest, int *cdest,
                 unsigned char *src, int width, int height);
  void do_copy_stoc_color(unsigned char *dest1, unsigned char *dest2, int *cdest,
			  unsigned char *src, int width, int height);
  void do_copy_bin_stoc(unsigned char *idest, short *ddest, int *cdest,
                 unsigned char *src, int width, int height);


  // single camera monochrome and color processing
  void do_copy_color(unsigned char *dest1, unsigned char *cdest,
		     unsigned char *src1, int width, int height, bool rev = false);
  void do_copy_mono(unsigned char *dest1,
                    unsigned char *src1, int width, int height);
  void do_copy_color_bin(unsigned char *dest1, unsigned char *cdest,
                         unsigned char *src1, int width, int height, bool rev = false);
  void do_copy_mono_bin(unsigned char *dest1,
			unsigned char *src1, int width, int height);

  double rred, bblue;           // values of manual color gains


  // enumerate
  char *deviceIds[MAX_DEVICES];             // ID strings
  
};  
