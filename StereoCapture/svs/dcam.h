//
// dcam.h
//
// Definition of the digital camera control class
//
// Copyright 2000 by Videre Design 
//

//
// The class dSystem is the control point for general IEEE 1394
//   system bus communication, e.g., initialization of the bus and 
//   and finding cameras.
//
// The class dCamera controls an individual camera.  Here you can set
//   and retrieve various parameters of the camera, such as exposure,
//   gain, frame rate, and so on.  It also lets you start and stop
//   acquisition from the camera, and provides a hook for a callback
//   function to get image data.
//


#ifndef DCAM_H
#define DCAM_H

#ifdef WIN32
#ifdef DCAMERA_EXPORTS
#define IMPORT  _declspec( dllexport )
#else
#define IMPORT  _declspec( dllimport )
#endif
#else
#define IMPORT
#endif


// version number, set in drivers
#define DCAM_VERSION "3.0a"
IMPORT extern char dcamVersion[];

// debugging interface
IMPORT extern void (* dcamDebug)(char *str); // call to print a debug string


// limits on sizes
#define NUMCAMS 10              // number of cameras allowed
#define NUMBUFS 2               // number of image buffers per camera
#define MAXWIDTH 1280           // max image size
#define MAXHEIGHT 960


//
// dITYPE: enumeration type for image formats
//

typedef enum
{ 
  S_MONO = 0,
  S_RGB24,
  S_YUV422,
  S_YUV411,
  S_BAYER
} dITYPE;

//
// dISPEED: enumeration type for acquisition speed
//

typedef enum
{
  S_30 = 0,			// 30 fps
  S_60,				// 60 fps
  S_15,				// 15 fps
  S_10,				// 10 fps
  S_7,				// 7.5 fps 
  S_5,				// 5 fps
  S_3				// 3.75 fps
} dISPEED;

//
// dISIZE: enumeration type for acquisition size
//

typedef enum
{
  S_640x480 = 0,                // 640x480, of course
  S_320x240,                    // 320x240
  S_160x120,                    // 160x120
  S_512x384,			// 512x384
  S_1024x768,			// 1024x768
  S_1280x960			// 1280x960
} dISIZE;


// color algorithm type
#ifndef COLOR_ALG_FAST
#define COLOR_ALG_FAST 0
#define COLOR_ALG_BEST 2
#endif


// 
// CCR mode, format, and frame rate definitions
//

// Format 0
#define FORMAT_VGA   0
#define FORMAT_SVGA1 1
#define FORMAT_SVGA2 2

#define YUV444_160x120 0
#define YUV422_320x240 1
#define YUV411_640x480 2
#define YUV422_640x480 3
#define RGB24_640x480  4
#define YUV400_640x480 5

#define YUV422_1024x768 3
#define RGB24_1024x768  4
#define YUV400_1024x768 5
#define Y16_1024x768    7

#define YUV422_1280x960 0
#define RGB24_1280x960  1
#define YUV400_1280x960 2
#define YUV800_1280x960 6

#define RATE_0375 1
#define RATE_0750 2
#define RATE_1500 3
#define RATE_3000 4
#define RATE_6000 5


// imager types
#define CAM_TYPE_KAC1310	0
#define CAM_TYPE_LM9648 	1
#define CAM_TYPE_MT9V403	2
#define CAM_TYPE_MT9M001	3	
#define CAM_TYPE_MT9V022	4
#define CAM_TYPE_MT9T001	5	
#define CAM_TYPE_BUMBLEBEE	8

//
// dSystem class
//   control of the IEEE 1394 bus and devices
//
// Member functions:
//   dSystem()        -- instantiates the sytem object, initializes
//                       filter graph and finds devices
//   Reset            -- empties filter graph and re-finds devices,
//                       will be invoked automatically on bus reset
//   InitCamera(char *)
//   InitCamera(int)
//                    -- initializes a camera based on its name or
//                       index in the found devices; returns dCamera
//                       pointer if successful, else NULL
//                    -- these functions are usually invoked
//                       implicitly from dCamera::Init 
//   Start()          -- starts capture
//   Stop()           -- stops capture
//

class dCamera;			// forward ref

class dSystem
{
 friend class dCamera;

 public:
  dSystem() {};
  virtual ~dSystem() {};
  virtual void Reset() = 0L;
  virtual int NumCameras() = 0L;
  virtual char **Names() = 0L;
  virtual char **ChipIDs() = 0L;
  virtual dCamera *InitCamera(char *) = 0L;
  virtual dCamera *InitCamera(int) = 0L;
  virtual bool Start() = 0L;
  virtual bool Stop() = 0L;
  bool initialized;		   
  int syncDelta;
};


// helper fns
// normal way to start things up is to call dSysInit(), which returns
//   a dSystem object if successful, NULL if not
// the global variable dSys is set to the system object after
//   initialization 
//

IMPORT extern dSystem *dSys;
IMPORT dSystem *dSysInit();


//
// dCamera class
//   control of an individual camera device
//
// Member functions:
//   dCamera(dSystem, int) 
//                    -- instantiates the indexed camera object, and gives it 
//                       the associated digital bus system
//   Index()          -- index of the camera on the bus
//   GetFormat(dISIZE *, dITYPE *, dISPEED *)
//                    -- gets the image format parameters of the
//                       camera
//   SetFormat(dISIZE, dITYPE, dISPEED)
//                    -- sets the image format parameters of the
//                       camera, returns 0 if successful
//                       Note: this will stop image acquisition,
//                       if it is running; have to restart using Start()
//   SetExposure(int, bool auto)
//                    -- sets the exposure, from 0 to 100; if auto is
//                       true, uses auto exposure
//   SetGain(int, bool auto) -- sets gain from 0 to 100, if not auto-exposure
//   SetSharpness(int) 
//                    -- sets sharpness filter from 0 to 100
//   SetWhiteBalance(int, int, bool auto)
//                    -- sets white balance from 0 to 100 for two channels; 
//                       if auto is true, uses auto white balance
//   SetSaturation(int)
//                    -- sets color saturation from 0 to 100
//   SetGamma(bool)   -- turns gamma correction on or off
//
//   Start()          -- starts continuous image acquisition
//   Stop()           -- stops continuous image acquisition
//   Reset()          -- resets the camera
//
//   bool SetSync(int dcam) 
//                    -- sets sync to another dcam; -1 turns it off
//                    -- have to set the other camera, too...
//
//   bool GetImage(unsigned char **, int ms, int *frame, int *time) 
//                    -- returns the next buffered image, waits up to
//                       ms milliseconds if no image is buffered; if
//                       no image is available, returns FALSE
//   bool ReadyImage(int ms)
//                    -- returns TRUE if there is an image available,
//                       waits up to ms milliseconds 
//   void FlushImages()
//                    -- gets rid of stale images from the buffers
//
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

class dCamera
{
 public:
  dCamera(dSystem *, int cam_ind) {};
  virtual ~dCamera() {};
  virtual bool  Start() = 0L;
  virtual bool  Stop() = 0L;
  virtual bool  Reset() = 0L;
  virtual int   Index() = 0L;
  virtual bool  SetSync(int) = 0L;
  virtual bool  SetFormat(dISIZE size, dITYPE type, dISPEED speed, int fdiv = 1, bool raw = false) = 0L;
  virtual dISIZE Size() = 0L;
  virtual dITYPE Type() = 0L;
  virtual dISPEED Speed() = 0L;
  virtual void SetExposure(int val, bool auto_flag = false) = 0L;
  virtual void SetGain(int val, bool auto_flag) = 0L;
  virtual void SetBrightness(int val, bool auto_flag = false) = 0L;
  virtual void SetWhiteBalance(int uval, int vval, bool auto_flag = false) = 0L;
  virtual void SetSaturation(int val) = 0L;
  virtual void SetGamma(bool on) = 0L;
  virtual void SetSharpness(int val) = 0L;
  virtual void SetYOffset(int val, int val2 = 0) = 0L;
  virtual void SetXOffset(int val) = 0L;
  virtual void SetBlackLevel(int val, int val2 = 0) = 0L;
  virtual bool SaveLocalParams() = 0L;
  virtual bool GetCalib(char *bb, int n) = 0L;
  virtual bool SetCalib(char *bb, int n) = 0L;
  virtual bool GetUser(char *bb, int n) = 0L;
  virtual bool SetUser(char *bb, int n) = 0L;
  virtual bool SetProcMode(proc_mode_type mode) = 0L; 
  virtual bool SetConfThresh(int thresh) = 0L; 
  virtual bool SetUniqThresh(int thresh) = 0L; 
  virtual bool GetImage(unsigned char **, int ms = 0, int *frame = 0L, unsigned long *time = 0L) = 0L;
  virtual bool ReadyImage(int ms) = 0L;
  virtual void FlushImages() = 0L;
  bool initialized;
  char *error;			// string for last error

  // camera type
  bool isStereo;                // stereo or monocular camera
  bool isBayer;                 // true if camera returns a Bayer pattern (no color processing)
  bool isBGGR;			// true if the color pattern starts with BG
  int  colorAlg;		// algorithm to use for color
  bool isReversed;		// true if camera image is reversed
  int  camImager;		// camera imager type
  int  fwFirmware;              // FW firmware release, 0 if no local params
  int  camFirmware;		// camera firmware release
  int  devID;			// device ID, lower 8 chars
  bool has_proc_mode;		// true if there is a STOC processor
  int  procFirmware;		// firmware number of STOC processor
  proc_mode_type procMode;	// current processing mode


  // digitization params
  bool has_auto_brightness;
  bool has_manual_brightness;
  bool auto_brightness;
  int brightness;
  int max_brightness;

  bool has_auto_exposure;
  bool has_manual_exposure;
  bool do_auto_exposure;
  bool do_auto_gain;
  int exposure;
  int max_exposure;

  bool has_manual_gain;
  bool has_auto_gain;
  int gain;
  int max_gain;

  bool has_sharpness;
  int sharpness;
  int max_sharpness;

  bool has_gamma;
  bool gamma;

  bool has_color;
  bool has_saturation;
  int saturation;
  int max_saturation;

  bool has_auto_whitebalance;
  bool has_manual_whitebalance;
  bool auto_whitebalance;
  int redgain, bluegain;
  int max_white_gain;
  bool has_chip_balance;	// true if on-chip white balance

  // software auto exposure values
  double auto_kp;               // proportional constant
  double auto_hyst;             // hysteresis
  double auto_bias;             // bias factor, >1 brighter, <1 dimmer

  // local parameters, not in DCAM 1.30
  int max_width;		// imager max size
  int max_height;

  bool has_y_offset;
  int max_y_offset;
  int y_offset;
  int y_offset_right;

  bool has_x_offset;
  int max_x_offset;
  int x_offset;
  int x_offset_right;

  bool has_frame_div;
  int max_frame_div;
  int frame_div;

  bool has_50Hz_rate;
  bool is_50Hz_rate;

  bool has_black_offset;
  int max_black_offset;
  int black_offset;
  int black_offset_right;

  // low-level read/write of quadlets
  virtual bool ReadQuadlet(unsigned long addr, unsigned long *quad) = 0L;
  virtual bool WriteQuadlet(unsigned long addr, unsigned long quad) = 0L;

};

#endif // DCAM_H
