/*#########################################
 * svs.h
 *
 * Header file for the SVM library
 *
 *#########################################
 */

#ifndef _svs_h_
#define _svs_h_

/**
 ** svs.h
 **
 ** Copyright 2001 by Kurt Konolige
 **
 ** The author hereby grants to SRI permission to use this software.
 ** The author also grants to SRI permission to distribute this software
 ** to educational institutes for non-commercial educational use only.
 **
 ** The author hereby grants to other individuals or organizations
 ** permission to use this software for non-commercial
 ** educational use only.  This software may not be distributed to others
 ** except by SRI, under the conditions above.
 **
 ** Other than these cases, no part of this software may be used or
 ** distributed without written permission of the author.
 **
 ** Neither the author nor SRI make any representations about the 
 ** suitability of this software for any purpose.  It is provided 
 ** "as is" without express or implied warranty.
 **
 ** Kurt Konolige
 ** Senior Computer Scientist
 ** SRI International
 ** 333 Ravenswood Avenue
 ** Menlo Park, CA 94025
 ** E-mail:  konolige@ai.sri.com
 **
 **/

#include <stdio.h>

#ifndef WIN32
#ifndef UNIX
#define UNIX
#endif
#endif

#ifdef UNIX
#include <sys/ipc.h>
#include <sys/shm.h>
// #include <string.h>
#define ALIGN8 __attribute__ ((aligned (8)))
#define ALIGN16 __attribute__ ((aligned (16)))
//#define ALIGN8
//#define ALIGN16
#define WINALIGN8
#define WINALIGN16
#define FALSE 0
#define TRUE 1
#endif

#ifdef WIN32
#include <windows.h>
#define ALIGN8
#define ALIGN16
#define WINALIGN8 __declspec(align(8))
#define WINALIGN16 __declspec(align(16))
#ifdef CALCLIB
#define REALLYIMPORT __declspec(dllimport)
#else
#define REALLYIMPORT IMPORT
#endif
#ifndef LIBCODE
#define IMPORT __declspec(dllimport)
#else
#define IMPORT __declspec(dllexport)
#endif
#define EXPORT __declspec(dllexport)
#else
#define IMPORT
#define EXPORT
#define REALLYIMPORT
#endif

//typedef WINALIGN16 unsigned char UCHTYPE16 ALIGN16;
//typedef WINALIGN16 unsigned short USHTYPE16 ALIGN16;
//typedef WINALIGN16 signed short SHTYPE16 ALIGN16;
typedef WINALIGN16 unsigned char UCHTYPE16;
typedef WINALIGN16 unsigned short USHTYPE16;
typedef WINALIGN16 signed short SHTYPE16;

#define ADJUST16(s,ty) {char *a; if (0xf&(int)s) \
   { a = (char *)s + 16 - (0xf & (int)s); \
     s = (ty *)a;  } }

/* This structure stores the intrinsic parameters of a camera.  The 
 *   first half of this structure deals with parameters recovered
 *   from camera calibration.  The second half allows for the 
 *   parameters to be adjusted for subsampled images and subwindows.
 *   Camera calibration should be done for the largest image size 
 *   possible, and the image subwindow parameters used if smaller 
 *   images are to be used.
 */
typedef struct {
  /* Intrinsic parameters */
  int       pwidth;             /* [pix]    Width of rectification images */
  int       pheight;            /* [pix]    Height of rectification images */
  double    dpx;                /* [mm/pix] X dimension of pixel in frame grabber */
  double    dpy;                /* [mm/pix] Y dimension of pixel in frame grabber */
  double    sx;                 /* []       Scale factor to compensate for any error in dpx   */
  double    Cx;                 /* [pix]    Z axis intercept of image plane */
  double    Cy;                 /* [pix]    Z axis intercept of image plane */
  double    f, fy;              /* [mm]     Focal length */
  double    alpha;              /* skew, usually zero */
  double    kappa1;             /* [1/mm^2] First coefficient of radial distortion */
  double    kappa2;             /* [1/mm^4] Second coefficient of radial distortion */
  double    kappa3;             /* [1/mm^6] Second coefficient of radial distortion */
  double    tau1, tau2;         /* tangential distortion, normalized */
  float     proj[3][4];         /* projection matrix of rectified image */
  float     rect[3][3];         /* rectification transform after correction for lens distortion */
} svsIP;

//
// The svsSP structure stores all info about a stereo image pair and the stereo
//   calculation.  Used as an argument to most functions.
//

typedef struct {
  int id;

  /* Transformation of left camera coords to global coords */
  double GTx;			/* [mm] */
  double GTy;			/* [mm] */
  double GTz;			/* [mm] */
  double GRx;			/* [rad] Yaw */
  double GRy;			/* [rad] Pitch */
  double GRz;			/* [rad] Roll */

  /* Stereo algorithm parameters */
  int convx, convy;             /* Edge convolution kernel size, pixels */
  int corrxsize, corrysize;	/* Correlation window size, pixels */
  int thresh;                   /* Confidence threshold, 0-40 */
  int unique;                   /* Uniqueness threshold, 0-40 */
  int specklediff;		/* Max diff between disparity values for region */
  int specklesize;		/* Minimum size for coherent disparity region */
  int lr;                       /* Left/right check, 1=on, 0=off */
  int ndisp;                    /* Number of pixel disparities to search */
  int dpp;                      /* Disparities per pixel */
  int offx, offy;               /* offx is horopter displacement, offy unused */
  double    frame;              /* framing factor, 1.0 is no scaling, less than 1.0 reduces rectified image */
  // disparity buffer offsets
  int dleft, dtop;              // offset of disparity image upper left corner
  int dwidth, dheight;          // disparity image actual width and height

  /* framegrabber/image parameters absolute parameters */
  int max_linelen;              /* for NTSC fields, 640 */
  int max_lines;                /* for NTSC fields, 240 */
  int max_decimation;           /* 1, 2 or 4 */
  int max_binning;              /* 1 or 2 */
  int max_framediv;		/* 1, 2, or 4 */
  double gamma;                 /* gamma correction needed for display, 1.0 is none */
  int color, color_right;       /* 0 for monochrome, 1 for color imager */

  /* digitization parameters */
  int autogain;                 /* 1 if currently on */
  int autoexposure;		/* 1 if currently on */
  int autowhite;                /* 1 if currently on  */
  int autobrightness;           // 1 if currently on 

  int gain;                     /* Image gain, from 0 to 100; -1 is auto */
  int exposure;                 /* Image exposure from 0 to 100, -1 is auto */
  int contrast;                 /* Image contrast, from 0 to 100, if auto mode */
  int brightness;               /* Image brightness, from 0 to 100, if auto mode */
  int saturation;               /* Image color saturation, from 0 to 100 */
  int red;                      /* Image red gain, from 0 to 100, -1 is auto */
  int blue;                     /* Image blue gain, from 0 to 100, -1 is auto */

  /* Image subwindow */
  int ix;                       /* Subimage start column */
  int iy;                       /* Subimage start row */
  int width;                    /* Subimage width, in pixels */
  int height;                   /* Subimage height, in pixels */

  /* vergence */
  double vergence;              /* Subimage vergence between images, in pixels */

  /* rectification flag */
  int rectified;                /* 1 if the image is rectified, else 0 */

  /* warping source subwindow parameters, only used in subwindow warping */
  int subwarp;                  /* 1 if we're warping subimage, 0 if not */
  int left_ix, left_iy;         /* source start column and row */
  int left_width, left_height;	/* source width and height */
  int right_ix, right_iy;       /* source start column and row */
  int right_width, right_height; /* source width and height */

  /* Current full-frame image size */
  int decimation;               /* current decimation */
  int binning;                  /* current binning */
  int framediv;			/* current frame division */
  int subwindow;                /* 1 = subwindow capability, 0 = none */
  /* Parameters below here are important for redoing warping addresses */
  int linelen;                  /* Image line length, in pixels */
  int lines;                    /* Number of image lines */

  /* Internal camera optical parameters */
  int have_rect;                /* 1 if this structure has valid rectification params */
  svsIP  left;                  /* Left camera parameters */
  svsIP  right;                 /* Right camera parameters */

  /* Transformation between left and right cameras */
  /* Coordinate system is attached to the center of projection of the
     left camera, with the X and Y axis aligned with the u and v axis,
     and Z along the line of sight */
  double Tx;                    /* [mm] */
  double Ty;                    /* [mm] */
  double Tz;                    /* [mm] */
  double Rx;                    /* [rad] rotation vector */
  double Ry;                    /* [rad] norm is angle to rotate */
  double Rz;                    /* [rad] direction is axis of rotation */

  unsigned long reserved;
} svsSP;


//
// 3D point structure
// X,Y,Z floats, plus an additional int A to give 16 bytes
// Have to make sure of order of elements - A should be last
//

typedef WINALIGN16 struct
{
 int A;
 float Z, Y, X;
 }  ALIGN16 svs3Dpoint;


//
// Color structure
// Parses the 4 bytes of each color pixel
//

typedef struct
{
  unsigned char r,g,b,a;
} svsColorPixel;


// error codes
#define VIDEO_ERROR_FRAME_SIZE   -6
#define VIDEO_ERROR_FRAME_SAMPLE -2
#define VIDEO_ERROR_DEC_MODE     -3
#define VIDEO_ERROR_BIN_MODE     -4
#define VIDEO_ERROR_SUBWINDOW    -5


//#define LINE_SIZE 160
//#define FRAME_SIZE 120

/*
 * Maximum sizes
 */
#define MAXX (4368+0)		/* maximum number of columns in image */
#define MAXY (2912+0)		/* maximum number of rows in image */
#define MAXD 16			/* stereo search range */
#define MAXDS 256		/* abs max search range */
#define MAXW 21			/* max window size */
#define MAXRVAL 4096		/* maximum range value returned */
#define SUBPIXELS 16		/* subpixel interpolation size */

/* addressing types for warping */
#define ADDRTYPE int
#define BILINTYPE struct bilinear_interp
#define BILINMULT struct bilinear_mult
#define BILINWRITE struct bilinear_write
#define BILIN_OFFSET 8
#define BILIN_DELTA 32
#define BILIN_TAB_SIZE (BILIN_DELTA * BILIN_DELTA)

struct bilinear_interp		// 8 bytes
{
  unsigned char a,b,c,d; int iaddr;
};

struct bilinear_mult
{
  short a, b, c, d;
};

struct bilinear_write
{
  int iaddrx, iaddry, oaddrx, oaddry; double alpha, beta;
};


//
// structure for subwindow warping indices
// ylow, yhigh are x-indices of where the warped line gets its pixel from,
//    for the low and high y-values of the line.  They are 0 if it's at the
//    end of the line, and the index if it's in the middle
// similarly for xlow, xhigh
//

struct subwin_index
{
  int xlow, xhigh, ylow, yhigh;
};


#define VID_ROWS 240
#define VID_COLS 320
#define XCORR  11
#define YCORR  11
#define NUM_DISP 16

/* These macros give offsets and size of the disparity image, relative
 *   to the left intensity image.  Takes svsSP pointer as the argument. 
 *   Offsets are in terms of the left image.
 */
#define ABS(x) (((x) < 0) ? (-(x)) : (x))
#define RESWIDTH(p) ((p)->width - (p)->convx - (p)->corrxsize - 2*(p)->ndisp - \
                     ABS((p)->offx) + 4)
#define RESLEN(p)   ((p)->height - (p)->convy - (p)->corrysize - ABS((p)->offy) + 2)
#define RESLEFT(p)  (((p)->convx +(p)->corrxsize + (p)->ndisp + (p)->ndisp - 4) /2 \
                     + ((p)->offx < 0 ? -(p)->offx : 0))
#define RESTOP(p)   (((p)->convy + (p)->corrysize - 2) / 2 + \
		     ((p)->offy < 0 ? -(p)->offy : 0))

/* These macros give offsets and size of the edge image, relative
 *   to the left intensity image.  Takes svsSP pointer as the argument. 
 */
#define EDGEWIDTH(p) (((p)->linelen) - (p)->convx + 1)
#define EDGELEN(p)   (((p)->height) - (p)->convy + 1)
#define EDGELEFT(p)  ((p)->convx/2)
#define EDGETOP(p)   ((p)->convy/2)

/* display mode */
#define DISP_NONE   0
#define DISP_LEFT   1
#define DISP_RIGHT  2
#define DISP_BOTH   3
#define DISP_MOSAIC 4
#define DISP_HIST   5
#define DISP_IMAGE  6
#define DISP_X1     7
#define DISP_X2     8

/* acquisition mode */
#define AQ_SINGLE     1
#define AQ_CONTIN     2
#define AQ_FREEZE    11
#define AQ_CALIBRATE 10

/* GM:acquisition type */
#define AQT_SVM       3
#define AQT_BUFFER    4
#define AQT_FRAMEGRAB 5
#define AQT_NONE      6
#define AQT_FILE      7

/* capture formats */
#define CAP_INTERLACE   0
#define CAP_RGCOMPONENT 1
#define CAP_DUAL        2
#define CAP_TRIPLE      3
#define CAP_CHANNELS    4

#define BI_YVU9  0x39555659		// YVU9, planar
#define BI_YV12  0x32315659		// YVU12 (4:1:1), planar
#define BI_I420  0x30323449		// YUV12 (4:2:0), planar 
#define BI_I422  0x32323449		// YUV12 (4:1:1), planar
#define BI_MRG2  0x3247524d		// YUV16 (4:2:2) ??

/*
 * SVS video capture main functions
 */

IMPORT extern char *svsVideoIdent;  // identifier string for this capture interface


// Debugging callbacks
IMPORT extern void (*capDebug)(char *str);
IMPORT extern void (*svsDebug)(char *str);
IMPORT void svsDebugMessage(char *str, ...);

// color algorithm constants

#define COLOR_ALG_BEST 2
#define COLOR_ALG_FAST 0


/* 
 * Sometimes we need to say which image we want
 */

#define svsLEFT  100
#define svsRIGHT 101
#define svsBOTH  102
#define svsLEFTCOLOR 103
#define svsRIGHTCOLOR 104
#define svsLEFTBAYER 105
#define svsRIGHTBAYER 106


/*
 * display output
 */
#define svsNONE       0
#define svsSTEREO     1
#define svsDISPARITY  1
#define svsCONFIDENCE 2
#define svsEDGES      3
#define svsMOTION     4
#define svsCOLOR      5
#define svsRGB24      5
#define svsMONOCHROME 6


/*
 * Display functions
 */

IMPORT extern void svsSleep(int ms); /* wait this long in milliseconds */

#ifdef UNIX
#endif

#ifdef WIN32
IMPORT extern HWND svsMainWin;		/* main window handle */
#endif

/*
 * image file I/O
 */

#define HEADER_STRING "srisunim"

#define HEADER_TYPE 0
#define DATA_INT 0
#define DATA_FLOAT 1
#define HEADER_SIZE 256

/* whole structure should take 256 bytes */
struct  srisunim_header
{ 
    char srisunim[8];
    char header_type;
    char data_type;
    char bits_per_pixel;
    char n_dimensions;
    unsigned char x_dimension_h;
    unsigned char x_dimension_l;
    unsigned char y_dimension_h;
    unsigned char y_dimension_l;
    unsigned char z_dimension_h;
    unsigned char z_dimension_l;
    unsigned char n_images;	/* number of images */
    char padding[237];
  };
    
/* Reads an .ssi file and put contents into buffer.  Creates
   buffer if null. */
IMPORT unsigned char *svsReadFile(char *name, unsigned char *buffer, 
			   int *height, int *width, int *num);
IMPORT int svsWriteImage(char *name, unsigned char *buf, int width, 
			 int height, char *type, int numframes);
// bpp should be either 8 or 24; error if file doesn't match
IMPORT int svsReadFileSizeBMP(char *name, int *width, int *height);
IMPORT int svsReadFileBMP(char *name, unsigned char *buffer, 
			   int *width, int *height, int bw);
IMPORT unsigned char *svsReadFile2BMP(char *name, unsigned char *buffer, 
			   int *width, int *height);
IMPORT int svsWriteImageBMP(char *name, unsigned char *im, int width, int height, 
			     int isDisparity);
IMPORT int svsWriteImageColorBMP(char *name, unsigned char *im, int width, int height, 
				 int isDisparity);
IMPORT int
svsWrite3DArray(char *fname, svs3Dpoint *pts, unsigned char *image, 
		short *disp, int type, svsSP *sp);
IMPORT int
svsWrite3DCloud(char *fname, svs3Dpoint *pts, unsigned char *image, 
		short *disp, int type, svsSP *sp, int nump);
IMPORT int
svsWriteWRL(char *fname, svs3Dpoint *pts, unsigned char *image, 
		int type, svsSP *sp);
IMPORT int svsWriteDisparity(char *fname, short *disp, svsSP *sp);


// parameter files
IMPORT int svsWriteParamFile(char *fname, svsSP *sp);	/* writes out stereo setting */
IMPORT int svsWriteRectFile(char *fname, svsSP *sp);	/* writes out stereo rectification */
IMPORT int svsReadParamFile(char *fname, svsSP *sp);	/* reads in stereo setting */
IMPORT int svsWriteParamBuffer(char *buf, svsSP *sp);	/* writes out stereo setting */
IMPORT int svsWriteRectBuffer(char *buf, svsSP *sp);	/* writes out stereo rectification */
IMPORT int svsReadParamBuffer(char *buf, svsSP *sp);	/* reads in stereo setting */

IMPORT extern char *svsError;


/* version of the software */
#define engine_version 44
IMPORT extern int svsEngineVersion;
#define engine_subversion 'f'
IMPORT extern char svsEngineSubVersion;

IMPORT extern int svsPreFilterTime, svsPostFilterTime, svsCorrTime, svsMaxTime;

//
// new FLTK-based drawing fns
//

IMPORT int  fltk_check(void);	/* process fltk events */

#endif    /* ifdef _svs_h_ */
