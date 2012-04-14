//#########################################
// svsclass.h 
//
// class definitions for the SVS system
//
//#########################################
//

//
// svsclass.h
//
// Copyright 2001-2004 by Kurt Konolige
//
// The author hereby grants to SRI permission to use this software.
// The author also grants to SRI permission to distribute this software
// to educational institutes for non-commercial educational use only.
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


#ifndef svsclass_h
#define svsclass_h

#include <string.h>
#include "svs.h"

//
// Processing mode enum
// Only the STOC device can do more than PROC_MODE_NONE
//

#ifndef PROC_MODE_TYPE
#define PROC_MODE_TYPE
enum proc_mode_type
  {
    PROC_MODE_OFF = 0,
    PROC_MODE_NONE,
    PROC_MODE_TEST,
    PROC_MODE_RECTIFIED,
    PROC_MODE_DISPARITY,
    PROC_MODE_DISPARITY_RAW
  };
#endif

//
// Raw image enum
// Describes the raw format from the device
//
#ifndef RAW_MODE_TYPE
#define RAW_MODE_TYPE
enum raw_mode_type
  {
    RAW_MODE_NONE = 0,
    RAW_MODE_INTERLACED_MONO,
    RAW_MODE_INTERLACED_BAYER_RGGB,
    RAW_MODE_INTERLACED_BAYER_BGGR
  };
#endif


//
// Class svsImageParams
//
// Has basic information about the format of an image
//

class svsImageParams
{
 public:
  // image basic full-frame parameters
  int linelen;                  // Image line length, in pixels 
  int lines;                    // Number of image lines 

  // image subwindow
  int ix;                       // Subimage start column 
  int iy;                       // Subimage start row 
  int width;                    // Subimage width, in pixels 
  int height;                   // Subimage height, in pixels 
  double vergence;              // Subimage vergence between images

  // display
  double gamma;                 // gamma correction for display, 1.0 is none 
};


//
// Class svsIntrinsicParams
//
// Internal camera rectification parameters for an image
//
// This structure stores the intrinsic parameters of a camera:
//   Lens distortion, decentering, and projection matrix
//
//

class svsIntrinsicParams
{
 public:
  // Intrinsic parameters 
  int       pwidth;             // [pix]    Width of rectification images 
  int       pheight;            // [pix]    Height of rectification images 
  double    dpx;                // [mm/pix] X dimension of pixel in frame grabber 
  double    dpy;                // [mm/pix] Y dimension of pixel in frame grabber 
  double    sx;                 // []       Scale factor to compensate for any error in dpx   
  double    Cx;                 // [pix]    Z axis intercept of image plane 
  double    Cy;                 // [pix]    Z axis intercept of image plane 
  double    f, fy;              // [mm] or [pix] Focal length
                                // New parameter sets (version 3.0 and greater)
                                //   use the fy (y-axis focal length) parameter, 
                                //   and units are pixels
                                // Old parameter sets have fy = -1, and f is in 
                                //   mm

  double    alpha;              // []       Skew, usually zero
  double    kappa1;             // [1/mm^2] First coefficient of radial distortion 
  double    kappa2;             // [1/mm^4] Second coefficient of radial distortion 
  double    kappa3;             // [1/mm^6] Third coefficient of radial distortion 
  double    tau1, tau2;         // tangential distortion, normalized image coords
  float     proj[3][4];         // projection matrix of rectified image 
  float     rect[3][3];         // rectification transform after correction for lens distortion;
                                //  in new parameters, this is the rotation matrix from non-rect to
                                //  rectified image
};


//
// Class svsRectParams
//
// Rectification parameters for an image
//
// This class holds internal and external parameters for rectification of 
//   a stereo image pair.  It also holds ptr buffers for fast warping.
//

class svsRectParams
{
 public:
  svsIntrinsicParams left;
  svsIntrinsicParams right;

  // Transformation between left and right cameras 
  // Coordinate system is attached to the center of projection of the
  //   left camera, with the X and Y axis aligned with the u and v axis,
  //   and Z along the line of sight 
  double Tx;   // [mm] 
  double Ty;   // [mm] 
  double Tz;   // [mm] 
  double Rx;   // [rad] Yaw 
  double Ry;   // [rad] Pitch 
  double Rz;   // [rad] Roll 

  // scaling factor for rectification
  double frame; // 1.0 is no scaling; usually less than 1.0

  // Transformation between left camera and global frame 
  // Coordinate system is attached to the center of projection of the
  //   left camera, with the X and Y axis aligned with the u and v axis,
  //   and Z along the line of sight 
  double GTx;   // [mm] 
  double GTy;   // [mm] 
  double GTz;   // [mm] 
  double GRx;   // [rad] Yaw 
  double GRy;   // [rad] Pitch 
  double GRz;   // [rad] Roll 

};


//
// Class svsDisparityParams
//
// Disparity processing parameters for an image
//
// This class holds disparity processing parameters for a stereo image:
//    number of disparities, subpixel interpolation, etc.
//

class svsDisparityParams
{
 public:
  int corrsize;                 // Correlation window size, pixels 
  int thresh;                   // Confidence threshold, 0-40 
  int unique;			// Uniqueness threshold, 0-40
  int specklesize;		// Minimum size for a disparity region
  int specklediff;		// Max difference in disparity neighbors
  bool lr;                      // Left/right check, 1=on, 0=off 
  int ndisp;                    // Number of pixel disparities to search 
  int dpp;                      // Disparities per pixel 
  int offx, offy;               // Horopter offset; offy isn't used...

  // disparity buffer offsets
  int dleft, dtop;              // offset of disparity image upper left corner
  int dwidth, dheight;          // disparity image actual width and height

  // temps
  int xx, yy;
};


//
// Class svsStereoImage
//
// Base class for all stereo acquisition and processing
// Contains left, right, and left color (optional) images 
// Contains disparity and 3D point images
// Parameters of the images, especially relating to image
//   frame size and subwindowing
//

class svsStereoProcess;
class svsTransform;
class svsDCSAcquireVideo;

class svsStereoImage
{
  friend class svsStereoProcess;
  friend class svsAcquireImages;
  friend class svsDCSAcquireVideo;
 public:
  // image arrays; color image is always RGBX
  unsigned char *left, *right, *color, *color_right;
  // use these to retrieve the arrays
  unsigned char *Left() {return left;}
  unsigned char *Right() {return right;}
  unsigned long *Color() {return (unsigned long *)color;}
  unsigned long *ColorRight() {return (unsigned long *)color_right;}
  int imageBufsize;		// size of buffers, in pixels
  bool haveColor;               // true if color array present
  bool haveColorRight;          // true if right image color array present
  bool haveImages;              // true if we have good stereo images
  int framenum;

  // error explanation

  // image format, particular to each object
  svsImageParams ip;

  // rectification parameters for left and right images
  bool isRectified;             // have we done the rectification already?
  bool haveRect;                // true if the rectification params exist
  svsRectParams rp;             // rectification params, if they exist

  // disparity array
  short *disparity;             // disparity image, 16-bit pixels
  short *Disparity() {return disparity;}
  bool haveDisparity;           // have we calculated the disparity yet?
  svsDisparityParams dp;        // disparity image parameters

  // confidence array
  short *confidence;		// confidence image, 16-bit pixels
  short *Confidence() {return confidence;}
  bool doConfidence;		// do we want confidence returned?


  // 3D array
  // Packed array of 3D points
  // Format: structure of X,Y,Z,A (A is 0 for invalid point)
  // Mimics image structure - width and height
  svs3Dpoint *pts3D;

  bool have3D;
  int numPoints;                // number of points actually found

  // re-locate points via a homogenous matrix
  svsTransform *myLoc;

  // Member functions for handling buffers

  IMPORT svsStereoImage();      // sets it up, no buffers
  IMPORT ~svsStereoImage();     // tears it down, frees buffers if owned

  void Error(char *s)           // sets an error string 
    { strcpy(error, s); }
  char *Error()                 // returns the error string
    { return error; }

  // Puts a set of images into the image buffers
  IMPORT void SetImages(unsigned char *lim, unsigned char *rim, unsigned char *cim,
                        unsigned char *cimr = NULL,
                        svsImageParams *ip = NULL, svsRectParams *rp = NULL, 
                        bool rect = false, bool copy = false);
  IMPORT void SetDisparity(short *disp);
  IMPORT void SetRawBayerImages(unsigned char *stimage, 
				bool bin, bool lc, bool rc,
				bool rect, raw_mode_type type = RAW_MODE_INTERLACED_BAYER_BGGR);
  IMPORT void CopyImages(unsigned char *lim, unsigned char *rim, unsigned char *cim,
                         unsigned char *cimr = NULL,
                         svsImageParams *ip = NULL, svsRectParams *rp = NULL, bool rect = false);

  // File input/output
  IMPORT bool SaveToFile(char *basename); // saves images and params to files
  IMPORT bool ReadFromFile(char *basename); // gets images and params from files
  IMPORT bool ReadParams(char *name, bool str = false); // reads just params from file
  IMPORT bool SaveParams(char *name, bool str = false); // save just params to file
  IMPORT void SetRectParams(svsSP *sp); // sets rectification params from sp
  IMPORT bool SaveDisparity(char *name, bool bmp = false);  // save a disparity image 
  IMPORT bool SaveWRL(char *name); // save a WRL cloud of 3D points
  IMPORT bool Save3DPointCloud(char *name); // save a point cloud of 3D points
  IMPORT bool Save3DPointArray(char *name); // save a point array of 3D points

  // allocates and releases space for buffers
  IMPORT void AllocateImages(); 
  IMPORT void ReleaseImages(); 
  IMPORT void AllocateDisparity(); 
  IMPORT void ReleaseDisparity(); 
  IMPORT void Allocate3d(); 
  IMPORT void Release3d(); 
  IMPORT void ReleaseLoc();

  // Copying
  IMPORT void CopyFrom(svsStereoImage *si, int reduce = 0);  // copies contents of si,
                                // optionally reducing the image

  // setting disparity calculation parameters
  IMPORT void SetNDisp(int n);	// set number of disparities, 8 to 182
  IMPORT void SetLR(bool on);	// set LR check, not used
  IMPORT void SetThresh(int n);	// set texture filter threshold
  IMPORT void SetUnique(int n);	// set uniqueness filter threshold
  IMPORT void SetCorrsize(int n); // set correlation window size, 7 to 21
  IMPORT bool SetHoropter(int n); // set horopter (X offset)
  IMPORT void SetSpeckleSize(int n); // set min disparity region size
  IMPORT void SetSpeckleDiff(int n); // set disparity region neighbor diff


  // transfer image params
  IMPORT svsSP *GetSP();        // return it
  IMPORT void SetSP(svsSP *sp); // set it
  svsSP sp;                     // legacy, but useful: has all parameters

  IMPORT void ImageToSP();
  IMPORT void SPToImage();

  // transfer rectification params
  IMPORT void RectToSP();
  IMPORT void SPToRect();
  IMPORT void RectToLoc();      // sets up localization transform from rp

  // transfer disparity params
  IMPORT void DispToSP();
  IMPORT void SPToDisp();


 protected:
  // these are the real buffers, if we allocate them ourselves
  unsigned char *left_buf, *right_buf, *color_buf, *color_right_buf;
  short *disparity_buf;
  short *confidence_buf;
  svs3Dpoint *pts3D_buf;
  void reduceByTwo(unsigned char *output, unsigned char *input);

  // raw buffer is allocated by underlying driver, don't destroy it
  unsigned char *raw, *raw2;

  // error buffer
  char error[256];  

  // color interpolation routines
  double rred, bblue ;		// red, blue balance
  IMPORT void do_copy_rggb(unsigned char *dest1, unsigned char *dest2, 
			     int *cdest, int *cdest2,
			     unsigned char *src, int width, int height,
			     bool do_color, bool do_color_right,
			   int color_alg);
  IMPORT void do_copy_bggr(unsigned char *dest1, unsigned char *dest2, 
			     int *cdest, int *cdest2,
			     unsigned char *src, int width, int height,
			     bool do_color, bool do_color_right,
			   int color_alg);
  IMPORT void do_copy_bin_rggb(unsigned char *dest1, unsigned char *dest2, 
			     int *cdest, int *cdest2,
			     unsigned char *src, int width, int height,
			     bool do_color, bool do_color_right,
			       int color_alg);
  IMPORT void do_copy_bin_bggr(unsigned char *dest1, unsigned char *dest2, 
			     int *cdest, int *cdest2,
			     unsigned char *src, int width, int height,
			     bool do_color, bool do_color_right,
			       int color_alg);

};




//
// Class svsStereoProcess
//
// Process class for computing disparity image, and 3D points
//

class svsStereoProcess
{
 public:
  // Construct and Destruct
  IMPORT svsStereoProcess();    // sets it up
  IMPORT ~svsStereoProcess();   // tears it down

  // Calculations
  IMPORT bool CalcStereo(svsStereoImage *si, bool doMulti = false); // Calculate the stereo disparity image; 
  // make it overrideable
  // Calculate the 3D values, with an area of interest
  // Extra args are optional destination buffer, cutoff disparity,
  //   min and max XYZ values
  IMPORT bool Calc3D(svsStereoImage *si, int x = 0, int y = 0, int w = 0, int h = 0,
		     svs3Dpoint *dest = NULL, float *trans = NULL, float maxrange = 0.0, 
		     svs3Dpoint *mins = NULL, svs3Dpoint *maxs = NULL); 
  IMPORT bool CalcPoint3D(int x, int y, svsStereoImage *si,
                          double *X, double *Y, double *Z);

 private:
  void *calcBuf;
  float *tbuf_buf, *tbuf;
};


// 3D reconstruction and projection functions
//
#define svsReconstruct3DFast svsReconstruct3D
IMPORT void svsReconstruct3D (float *X, float *Y, float *Z, float x, float y, 
                                  float disp, svsSP *sp, svsTransform *loc = NULL);
IMPORT void svsProject3D (float *x, float *y, float X, float Y, float Z, 
			  svsSP *sp, int which, svsTransform *loc = NULL);



//
// Multiscale class
// Subclasses the stereo calculation object
// Adds its own processing for a small image
//

class svsMultiProcess : public svsStereoProcess
{
 public:
  IMPORT svsMultiProcess();
  IMPORT ~svsMultiProcess();
  
  // multiscale calculations
  IMPORT bool CalcStereo(svsStereoImage *si); // Overrides the svsStereoProcess fn
  IMPORT void fillIn(svsStereoImage *original, svsStereoImage *reduced);
  bool doIt;  // true if we're doing the multiproc thang

 private:
  svsStereoImage *reducedImage; // reduced stereo image
  svsStereoProcess *reducedProc; // reduced stereo processing
};


//
// class for 4x4 transform matrices
// tr[12] are elements, row major order, i.e., tr[0] = row 0 col 0, tr[1] = row 0 col 1, etc.
// sets up transform so that the parameters are the global coordinates of the
//   left camera
// transform performs translations of points in the camera space to points in the global
//   space
//

class svsTransform
{
 public:
  IMPORT svsTransform();
  IMPORT ~svsTransform();

  // elementary rotations
  IMPORT void setRotX(double Ang);
  IMPORT void addRotX(double Ang); // left-multiplies current matrix by this rotation

  IMPORT void setRotY(double Ang);
  IMPORT void addRotY(double Ang);

  IMPORT void setRotZ(double Ang);
  IMPORT void addRotZ(double Ang);

  IMPORT void setRotation(double Ax, double Ay, double Az);

  IMPORT void setRotVector(double rx, double ry, double rz); // rotation vector

  // translations
  IMPORT void setTrans(double X, double Y, double Z);

  // multiply with another matrix on the right: this * m
  IMPORT void rightMult(svsTransform &m);

  // multiply with another matrix on the left: m * this
  IMPORT void leftMult(svsTransform &m);

  // perform transform of a 3D point
  IMPORT void doTransform(float *X, float *Y, float *Z);

  // saved rotation values
  double rotX, rotY, rotZ;

  // saved shift values
  int shiftX, shiftY, shiftZ;

  // matrix values
  float tr[16];
};



//
// Class svsWarpProcess
//
// Process class for computing warp
// Used internally, shouldn't be invoked by applications
//

class svsWarpProcess
{
  friend class svsAcquireImages;
 public:
  // Construct and Destruct
  IMPORT svsWarpProcess();	// sets it up
  IMPORT ~svsWarpProcess();            // tears it down

  // Calculate the warp transform
  // New parameters itx, ity for target image size; use 0 for original size
  void Warp(unsigned char *dest, unsigned char *src, int which, svsSP *sp, int itx = 0, int ity = 0); 
  // sets up the warp table immediately
  void SetWarp(svsSP *sp, int itx = 0, int ity = 0); 
  // returns orig addr from rectified addr
  IMPORT void origAddr(float *x, float *y, float u, float v, svsSP *sp, int which);
  // returns rectified coords from original addr
  IMPORT bool rectAddr(float *u, float *v, float x, float y, svsSP *sp, int which); 
  // returns an incremental warp table in dest
  int genIncWarpTable(unsigned char *dest, svsSP *sp, int which);

  // Buffer allocation
  void AllocateBuffers(int w, int h);
  void ReleaseBuffers();

 protected:
  // foreign warping function
  bool (* wfn)(int w, int h, int which, float u, float v, float *x, float *y, void *warg);
  void *warg;
  int wpx, wpy;			// these are >0 if using a foreign mapping fn

 private:
  // Data buffers for warp addressing
  ADDRTYPE *laddr_buf, *laddr;
  ADDRTYPE *raddr_buf, *raddr;
  unsigned short *ibuf_buf, *ibuf;
  int l_max_y[MAXY], r_max_y[MAXY];
  BILINTYPE *lbaddr, *rbaddr;

  // subwindow warping
  struct subwin_index l_swi[MAXX], r_swi[MAXX];

  // forward warping function matrices
  double lrect[3][3], rrect[3][3];             // from normalized to rectified

  // how things have changed
  double checksum;
  double addr_checksum;         // for origAddr fn
  int tx, ty;			// for target rectification size
  int oldix, oldiy, oldwidth;
  int oldw, oldh;
  double oldvergence;
  double get_checksum(svsSP *sp);
};


//
// Image Acquisition Classes
// -- from stereo head, files, and memory
//

//
// Class svsAcquireImages
//
// Subclassed by the individual drivers
//

class svsAcquireImages
{
 public:
  // construction and destruction
  IMPORT svsAcquireImages();
  // should be virtual, but causes faults on destroy
  IMPORT virtual ~svsAcquireImages();

  // parameters
  IMPORT svsImageParams     *GetIP(); // returns basic image parameters
  IMPORT svsRectParams      *GetRP(); // returns rectification parameters
  IMPORT svsDisparityParams *GetDP(); // returns stereo disparity parameters
  IMPORT svsSP              *GetSP(); // returns the combined parameter set

  // setting disparity calculation parameters
  IMPORT void SetNDisp(int n); // set number of disparities, 8 to 128
  IMPORT void SetLR(bool on);	// set LR check, not used
  IMPORT void SetThresh(int n);	// set texture filter threshold
  IMPORT void SetUnique(int n);	// set uniqueness filter threshold
  IMPORT void SetCorrsize(int n); // set correlation window size, 7 to 21
  IMPORT virtual bool SetHoropter(int n); // set horopter (X offset)
  IMPORT void SetSpeckleSize(int n); // set min disparity region size
  IMPORT void SetSpeckleDiff(int n); // set disparity region neighbor diff


  // rectification stuff
  IMPORT virtual bool HaveRect();       // if we have rectification parameters
  IMPORT virtual bool SetRect(bool on); // turns rectification on and off
  IMPORT virtual bool GetRect();        // is rectification turned on?
  IMPORT bool IsRect();                 // is current buffer rectified?
  IMPORT bool SetRectSize(int x, int y); // targeted rectification size
  IMPORT virtual bool SetRectForeign(
	 bool (* wfn)(int w, int h, int which, float u, float v, float *x, float *y, void *warg),
	 void *warg);
  IMPORT int  GenIncWarpTable(unsigned char *dest, int which);


  // Device open and close
  IMPORT virtual bool Open(char *name = NULL) = 0; // opens a file or device
  IMPORT virtual bool Close();  // closes it

  // Start/stop acquisition, check parameters
  virtual bool Start() = 0;
  virtual bool Stop() = 0;
  virtual bool CheckParams() = 0;

  // Acquire images
  virtual svsStereoImage *GetImage(int ms) = 0; // returns a stereo image if available
  svsStereoImage *GetImageObject() { return imageBuf; }; // returns the stereo image object
  int acqTime;			// time that this frame was acquired from video, in ms
  int acqFrame;			// frame number

  // Parameter and file I/O
  IMPORT virtual bool ReadParams(char *basename); // reads params from file
  IMPORT virtual bool SaveParams(char *basename); // saves params to file
  IMPORT void SetRectParams(svsSP *sp); // sets rectification params from sp
  IMPORT virtual bool SaveToFile(char *basename);   // saves images and params to files

  // Errors
  void Error(char *s)           // sets an error string
    { strcpy(error, s); }
  char *Error()                 // returns the error string
    { return error; }

  // utility
  IMPORT bool UnrectImagePoint(double *x, double *y, double u, double v, int which);
  IMPORT bool RectImagePoint(double *u, double *v, double x, double y, int which);

  // projection
  IMPORT bool Project3D(double *x, double *y, double X, double Y, double Z, int which);


  // error return
  char error[256];

  // device
  char *deviceName;
  bool isOpened;

 protected:
  // rectification buffers, do it here so we can manage them
  int warpBufsize;
  unsigned char *left_warp_buf, *right_warp_buf, *color_warp_buf, *color_right_warp_buf;
  unsigned char *left_warp, *right_warp, *color_warp, *color_right_warp;
  bool AllocateBuffers();
  void ReleaseBuffers();

  // rectification
  svsWarpProcess *warpObj;
  bool doRect;                  // true if we're doing rectification
  IMPORT virtual bool Rectify(); // does rectification 
                                // finds original image pt corresponding to rect image pt
  int target_x, target_y;		// target rectification size

  // image buffer
  svsStereoImage *imageBuf;     // single buffer object
  svsImageParams ip;		// global image params
};


//
// Subclass svsVideoImages
// Does video capture
//   Must be subclassed by individual capture interfaces
//

class svsVideoImages
: public svsAcquireImages
{
 public:
  // acquisition subwindow for warping
  bool subwarp;                 // true if warping subimage
  int left_ix, left_iy;         // source start column and row 
  int left_width, left_height;  // source width and height 
  int right_ix, right_iy;       // source start column and row 
  int right_width, right_height;// source width and height 

  // acquisition device info
  int max_linelen, max_lines;   // max size of camera image
  int max_decimation;           // 1, 2 or 4 
  int max_binning;              // 1 or 2 
  int max_framediv;		// 1, 2, or 4
  bool subwindow;               // true for subwindow capability

  int decimation;               // current decimation
  int binning;                  // current binning
  int framediv;			// current frame division
  int framerate;                // how fast we go: 30, 15, 7, 3

  bool is_color_capable;        // true if the camera has color capability
  bool is_bggr;			// true if the camera has bg..gr lines
  bool has_color;               // true if we're returning color
  bool has_color_right;         // color for right imager
  bool has_disparity;		// true if disparity is calculated (STOC)
  bool has_rect;		// true if rectification has been done (STOC)
  int color_alg;		// color algorithm - COLOR_ALG_FAST, COLOR_ALG_BEST

  bool is_proc_capable;		// can we process stereo or not
  int  max_x_offset;		// can offset horopter if >0, STOC only
  int  x_offset;		// current value
  proc_mode_type proc_mode;	// current processing mode

  // digitization capabilities
  bool autogain;                // true if available 
  bool manualgain;              // true if available 
  bool autoexposure;		// true if available 
  bool manualexposure;		// true if available 
  bool autowhite;               // true if available 
  bool manualwhite;             // true if available 
  bool autobrightness;          // true if available
  bool manualbrightness;        // true if available
  bool manualsaturation;        // saturation control exists?
  bool manualsharpness;         // sharpness control exists?
  bool manualgamma;             // imager gamma control exists?

  // digitization parameters
  bool use_autogain;            // true if using autogain
  bool use_autoexposure;	// true if using autoexposure
  bool use_autowhite;           // true if using autowhite
  bool use_autobrightness;      // true if using autobrightness
  int gain;                     // Image gain, from 0 to 100
  int exposure;                 // Image exposure from 0 to 100
  int contrast;                 // Image contrast, from 0 to 100
  int brightness;               // Image brightness, from 0 to 100
  int saturation;               // Image color saturation, from 0 to 100 
  int red;                      // Image red gain, from 0 to 100
  int blue;                     // Image green gain, from 0 to 100
  int sharpness;                // Image sharpness
  int gamma;			// Image gamma on/off

  //
  // Member functions
  //
  
  // construct and destroy
  IMPORT svsVideoImages();      // create object 
  IMPORT virtual ~svsVideoImages(); // destroy object

  // enumerate devices
  IMPORT virtual int Enumerate(); // returns count of devices for IEEE 1394 bus
  IMPORT virtual char **DeviceIDs(); // returns list of devices for IEEE 1394 bus 

  // opening a device
  IMPORT virtual bool Open(int device) = 0; // opens a file or device
  IMPORT virtual bool Open(char *name = NULL) = 0; // opens a file or device
  IMPORT virtual bool Close();	// close a device, usually overriden

  // Acquisition device calls
  virtual bool SetCapture(int type) = 0; // CAP_DUAL or CAP_INTERLACE
  virtual bool SetFormat(int type) = 0;  // MONOCHROME, YUV, or RGB24
  virtual bool SetChannel(int type) = 0;  // 0, 1, 2 etc, video channel on card
  virtual bool SetSwap(bool on) = 0;     // Swapping left/right on or off
  virtual bool SetColor(bool on, bool r = false) = 0;    // Turn on color images
  virtual bool SetColorAlg(int alg) = 0; // specify color algorithm

  // these three must be set before streaming video
  virtual bool SetSize(int w, int h) = 0; // size of image
  virtual bool SetSample(int dec, int bin) = 0; // image sampling
  virtual bool SetFrameDiv(int fdiv) = 0; // image sampling
  virtual bool SetRate(int rate) = 0;  // image rate
  virtual bool SetProcMode(proc_mode_type mode) = 0; // sets the processing mode for STOC

  // these can be called during streaming video
  virtual bool SetOffset(int ix, int iy, int verge) = 0; // subwindow origin and vergence
  virtual bool SetExposure(int exp, int gn, bool auto_exp = false, bool auto_gn = false) = 0;
  virtual bool SetAutoExpParams(double auto_bias, double auto_kp = -1.0) = 0;
  virtual bool SetAutoExpFilter(int y0, int y1, int y2, int y3, int y4) = 0;
  virtual bool SetBalance(bool auto, int rd, int bl) = 0;
  virtual bool SetGamma(int gam) = 0;
  virtual bool SetBrightness(bool auto, int bright) = 0;
  virtual bool SetSaturation(int sat) = 0;
  virtual bool SetLevel(int contr, int bright) = 0;
  virtual bool SetHoropter(int xoff) = 0;

  // Set up digitization params
  // Set values, then call these fns; can happen during streaming
  virtual void SetDigitization() = 0;

  // read and write files
  IMPORT virtual bool ReadParams(char *basename); // reads params from file
  IMPORT virtual bool SaveParams(char *basename); // saves params to file
  IMPORT virtual bool ReadParams(); // reads params from device
  IMPORT virtual bool SaveParams(); // saves params to device
  IMPORT virtual bool SaveToFile(char *basename);   // saves images and params to files

  // streaming to a file
// opens a stream file, with a number of buffers, and optional user info
  IMPORT bool StreamOpen(char *name, int bnum = 200, unsigned char *user = NULL, int ulen = 0); 
  IMPORT bool StreamStart();	// starts the stream
  IMPORT bool StreamStop();	// stops the stream
  IMPORT bool StreamClose();	// close the stream file

  // raw images
  unsigned char *Raw() {return raw;} // raw image, may be null for some devices
  unsigned char *Raw2() {return raw2;} // raw image from 2nd camera, may be null for some devices
  raw_mode_type rawType;	// raw image type

 protected:
  IMPORT void ParamsToSP(svsSP &xsp);
  IMPORT void SPToParams(svsSP &xsp);
  //  void ParamsToSP() { ParamsToSP(sp); }
  //  void SPToParams() { SPToParams(sp); }
  // parameters from a device
  virtual bool getParams(char *bb, int n) = 0;
  virtual bool setParams(char *bb, int n) = 0;
  // user info from a device
  virtual bool getUserInfo(char *bb, int n) = 0;
  virtual bool setUserInfo(char *bb, int n) = 0;
  unsigned char *raw, *raw2;
  // streaming interface
  bool doStream;
  bool doStreamParams;		// do we save stream params?
  int ulen;			// user info length, buffer
  unsigned char *ubuf;
  FILE *logFP;			// file info
  int logFD;
  char logName[256];
  long long fsize;
  int bufsize;
  int blocksize;
  char *diskbuf;
  char *startptr;		// ring buffer ptrs
  char *endptr;
  bool empty;
  int missed;
  int framenum;
};  

//
// Capture interface library function for returning a subclass object
//

IMPORT svsVideoImages *getVideoObject();
IMPORT void closeVideoObject(); // allow it to clean up

//
// Class svsFileImages
// File Acquisition Class
//

class svsFileImages
: public svsAcquireImages
{
 public:
  // construct and destroy
  IMPORT svsFileImages();      // create object 
  IMPORT ~svsFileImages();      // destroy object 

  // Device open and close
  IMPORT bool Open(char *name); // opens a file

  // Start/stop acquisition, check parameters
  IMPORT bool Start();
  IMPORT bool Stop();
  IMPORT bool CheckParams();

  // Swap images
  IMPORT bool SetSwap(bool on); // Swapping left/right on or off

  // Acquire images
  IMPORT svsStereoImage *GetImage(int ms); // returns a stereo image

  // File input/output
  IMPORT char *Name();          // returns current file name

  // File name
  char name[256];               // current file name
  char bname[256];              // base file name
  bool incrName();              // increment file name
  bool first;
  bool swapping;                // set if we want to swap images
};


//
// Class svsStoredImages
// Memory Acquisition Class
// Does not modify stored images
// Output images are in buffers managed by the class, if rectified
//

class svsStoredImages
: public svsAcquireImages
{
 public:
  // These functions are noops, always return true
  IMPORT bool Open(char *name);
  IMPORT bool Start();
  IMPORT bool Stop();
  IMPORT bool CheckParams();

  // Set up images from memory
  IMPORT bool Load(int width, int height,
                   unsigned char *lim, unsigned char *rim, 
                   unsigned char *cim = NULL, unsigned char *cimr = NULL,
                   bool rect = false, bool copy = false);
  // Set up images from memory, in raw formats
  IMPORT bool LoadRawStereoBayer(int width, int height,
				 unsigned char *stimage, bool bin = false,
				 bool leftcolor = true, bool rightcolor = true,
				 bool rect = false);		      

  // Acquire images
  IMPORT svsStereoImage *GetImage(int ms); // returns a stereo image
};


// streaming save to file
#define STREAM_HDR_SIZE 64
IMPORT int svsStreamFile(char *name, int width, int height, int bnum = 200, int user = 0);
IMPORT int svsWriteStreamImage(int type, unsigned char *buf, int width, int height, unsigned char *user = NULL);
IMPORT int svsStreamClose();

// streaming file read
IMPORT int svsStreamOpen(char *name);
IMPORT int svsReadStreamImage(int *type, unsigned char *buf, int *framenum, int *width, int *height, unsigned char *user = NULL);


//
// Class svsStreamImages
// Stream File Acquisition Class
//

class svsStreamImages
: public svsStoredImages
{
 public:
  // construct and destroy
  IMPORT svsStreamImages();      // create object 
  IMPORT ~svsStreamImages();      // destroy object 

  // Device open and close
  IMPORT bool Open(char *name); // opens a stream file for reading
  IMPORT bool Close();		// closes the stream file

  // Acquire images
  IMPORT svsStereoImage *GetImage(int ms); // returns a stereo image

  // Moving around
  IMPORT bool SkipFrame(int skip); // skips forward or backward
  IMPORT bool SkipToFrame(int frame); // goes to a particular frame

  // Color processing
  IMPORT bool SetColor(bool on, bool r = false); // Turn on color images

  // frame values
  int framenum;
  int rawtype;
  int binning;
  int has_color;
  int has_color_right;
  unsigned char *user;		// user buffer
  int user_size;		// user data size

 private:
  // Stream name
  FILE *logFP;
  int logFD;
  char logName[256];
  int blocksize;		// image data size, in blocks
  int filesize;			// individual file size, in frames
  int fileptr;
  unsigned char *ibuf;
  bool incrName();              // increment file name
  bool first;
  bool swapping;                // set if we want to swap images
  bool do_color;
  bool do_color_right;
  bool isRectified;		// true if image stream is rectified
};



// timing
IMPORT int get_ms();

/* MMX definition */
REALLYIMPORT extern int svsHasMMX; // 1 for MMX, 3 for SSE, 7 for SSE2
                                   // 0 for none, -1 for not checked. 
IMPORT int svsCheckMMX(bool force = true); /* Checks for MMX and sets svsHasMMX */


#define LRVAL     0xfffe	/* failed LR check */
#define THRESHVAL 0xffff	/* failed texture threshold */

// MMX stuff

#ifdef WIN32   // MSW definitions 

#define ISMMX(flag) \
 __asm { \
        __asm mov  flag, ebx \
        __asm mov  eax, 1 \
        __asm mov  flag, ebx \
        __asm cpuid \
        __asm mov  ebx, flag \
        __asm mov  flag, edx \
} 


#define empty_mms \
__asm { emms }

#else

#define ASM __asm__ volatile
#define EASM : : : "%eax" );
#define __STR(x) #x
#define STR(x) __STR(x)
#define REG(x) "%%"__STR(x)
#define BRASM ": : ); ASM ("

/* sets flag to 1 if MMX present, else 0 */
// EDX has relevant bits: 23 (MMX), 25 (SSE), 26 (SSE2) 
#define ISMMX(flag) \
ASM ( " \
        \n\t movl $1,%%eax \
        \n\t movl %%ebx, %0 \
        \n\t cpuid \
        \n\t movl %0, %%ebx \
        \n\t movl %%edx,%0 \
": "=m" (flag) : : "%eax", "%ebx", "%ecx", "%edx")

#define empty_mms \
ASM (" emms ": : )

#endif


#endif  // svsclass_h


