//
// Image I/O routines
//


#ifdef UNIX
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdarg.h>
#include <unistd.h>
#include <pthread.h>
#define STRNCASECMP strncasecmp
#else
#include <io.h>
#include <fcntl.h>
#define STRNCASECMP _strnicmp
#endif
#include <stdio.h>
#include <malloc.h>

#ifdef UNIX
#define EXPORT
#else
#define EXPORT __declspec(dllexport)
#endif

#define LIBCODE
#include "svsclass.h"

int Print_error_messages = 1;
EXPORT char *svsError = "";


#ifdef UNIX
// thread and mutex fns
void mlock(pthread_mutex_t *mutex, int ms = 0)
{
  pthread_mutex_lock(mutex);
}

void 
munlock(pthread_mutex_t *mutex)
{
  pthread_mutex_unlock(mutex);
}

static pthread_mutex_t *dlock = NULL;
pthread_mutex_t *startmutex(char *name = "mymutex")
{
  pthread_mutex_t *mutex = new(pthread_mutex_t);
  pthread_mutex_init(mutex,NULL);
  return mutex;
}

static pthread_t *dthread;
pthread_t *startthread(void *(*f)(void *), void *p = NULL)
{
  pthread_t *thread = new(pthread_t);
  pthread_attr_t other_attr;
  // create write thread
  pthread_create(thread, NULL, f, p);
  return thread;
}


#else
// Windows thread and mutex fns

HANDLE dthread;			// thread for writing to disk
HANDLE startthread(LPTHREAD_START_ROUTINE f, void *p = NULL)
{
  return CreateThread(NULL, 0, f, p, 0, NULL);
}
static HANDLE dlock = NULL;
HANDLE startmutex(char *name = "mutex")
{
  return CreateMutex(NULL, FALSE, name);
}

// returns when we get the lock
void mlock(HANDLE mutex, int ms = 0)
{
  WaitForSingleObject(mutex, ms); // wait for a fixed time, or forever if ms=0
}

// unlock
void munlock(HANDLE mutex)
{
  ReleaseMutex(mutex);
}


// need to define come UNIX fns
int bcmp(char *x, char *y, int n) 
{
  int i;
  for (i=0; i<n; i++)
    if (*x++ != *y++) return 1;
  return 0;
}

void bzero(char *x, int n)
{
  int i;
  for (i=0; i<n; i++)
    *x++ = 0;
}

void bcopy(char *x, char *y, int n) 
{
  int i;
  for (i=0; i<n; i++)
    *y++ = *x++;
}
#endif


/* BMP files */

int
read_bmp_word(unsigned char *p)
{
  int i;
  i = *p++;
  return ((*p << 8) | i);
}

long
read_bmp_long(unsigned char *p)
{
  int i, j, k;
  i = *p++;
  j = *p++;
  k = *p++;
  return ((*p << 24) | (k << 16) | (j << 8) | i);
}

#define BMP_HEADER_SIZE 14
#define BMP_INFO_SIZE 40
#define BMP_MAX_SIZE 20000000

char read_ret[100];

EXPORT int
svsReadFileSizeBMP(char *name, int *x, int *y)
{
  unsigned char buf[2000];
  int offset, size, i, width, height, fbpp;
  FILE *fp;
  int fd;

  if ((fp = fopen(name, "rb")) == NULL)
    {
      svsError = "Can't open file";
      return -1;
    }
  fd = fileno(fp);
#ifndef UNIX
  _setmode(fd,_O_BINARY);
#endif
  i = read(fd, buf, BMP_HEADER_SIZE+BMP_INFO_SIZE);
  if (buf[0] != 'B' || buf[1] != 'M')
    {
      sprintf(read_ret, "Not a BMP file: %s", name);
      svsError = read_ret;
      fclose(fp);
      return -1;
    }
  fbpp = read_bmp_word(&buf[28]);
  offset = read_bmp_long(&buf[10]);
  size = read_bmp_long(&buf[2]);
  if (size > BMP_MAX_SIZE)
    {
      sprintf(read_ret, "BMP file %s too large: size is %d bytes", name, size);
      svsError = read_ret;
      fclose(fp);
      return -1;	
    }
  width = read_bmp_long(&buf[18]);
  height = read_bmp_long(&buf[22]);
  *x = (width+3)&~0x3;		// make multiple of 4
  *y = height;
  fclose(fp);
  return 0;
}


//
// Reads in a BMP file
// For grayscale, can handle 8-bit files with arbitrary encoding
//   Also converts color (24 bpp) to grayscale if the bw flag is set
// For color, expects 24 bpp 
//

EXPORT int
svsReadFileBMP(char *name, unsigned char *op, int *x, int *y, int bw)
{
  unsigned char buf[2000];
  int offset, size, bpp, i, width, height, owidth;
  FILE *fp;
  int fd;

  if ((fp = fopen(name, "rb")) == NULL)
    {
      svsError = "Can't open file";
      return -1;
    }
  fd = fileno(fp);
#ifndef UNIX
  _setmode(fd,_O_BINARY);
#endif
  i = read(fd, buf, BMP_HEADER_SIZE+BMP_INFO_SIZE);
  if (buf[0] != 'B' || buf[1] != 'M')
    {
      svsError = "Not a BMP file";
	  fclose(fp);
      return -1;
    }
  offset = read_bmp_long(&buf[10]);
  size = read_bmp_long(&buf[2]);
  if (size > BMP_MAX_SIZE)
    {
	  sprintf(read_ret, "BMP file too large: size is %d bytes", size);
	  svsError = read_ret;
	  fclose(fp);
	  return -1;	
    }
  width = read_bmp_long(&buf[18]);
  owidth = (width+3) & ~0x3;	// always make multiple of 4
  height = read_bmp_long(&buf[22]);
  if (*x > 0 && owidth != *x)
    {
       svsError = "BMP file widths not consistent";
       fclose(fp);
       return -1;
    }

  if (*y > 0 && height != *y)
    {
       svsError = "BMP file heights not consistent";
	   fclose(fp);
       return -1;
    }

  *x = owidth;
  *y = height;
  bpp = read_bmp_word(&buf[28]);

  if (bpp == 24)		// color file, maybe convert to bw
    {
      int linewidth, j;
      unsigned char fbuf[3*MAXX+4];

      linewidth = width*3;
      linewidth = (linewidth + 0x3) & ~0x3;	// round to 4 bytes (????)

      if (bw == 0)		// color here
	{
	  op += 4*owidth*(height-1);
	  for (i=0; i<height; i++, op-=4*owidth)
	    {
	      read(fd, (char *)fbuf, linewidth);
	      for (j=0; j<width; j++)
		{
		  op[j*4+2] = fbuf[j*3];
		  op[j*4+1] = fbuf[j*3+1];
		  op[j*4]   = fbuf[j*3+2];
		  op[j*4+3] = 0;
		}
	    }
	}
      else			// grayscale here
	{
	  // currently just average all colors
	  op += owidth*(height-1);
	  for (i=0; i<height; i++, op-=owidth)
	    {
	      read(fd, (char *)fbuf, linewidth);
	      for (j=0; j<width; j++)
		{
		  op[j] = ((int)fbuf[j*3] + (int)fbuf[j*3+1] + (int)fbuf[j*3+2]) / 3;
		}
	    }
	}
    }
  else				// 8 bit BMPs
    {
      i = read(fd, buf, offset-BMP_INFO_SIZE-BMP_HEADER_SIZE); // read in trans table
      // get translation table
      unsigned char tab[256];
      for (i=0; i<256; i++)
	tab[i] = 0;
      for (i=0; i<offset-BMP_INFO_SIZE-BMP_HEADER_SIZE; i+=4)
	{
	  tab[i/4] = buf[i];
	}

      op = op + owidth*(height-1);
      for (i=0; i<height; i++, op-=owidth)
	{
	  int k = read(fd, op, owidth);
	  int j;
	  for (j=0; j<width; j++)
	    op[j] = tab[op[j]];
	}

    }
  fclose( fp );
  return 0;
}


/*
 * Loads 2 BMP images as XXX-L.bmp and XXX-R.bmp
 * Input name can be XXX or XXX-L/R.bmp
 * Returns images consecutively in buffer, or makes a new
 *  buffer if the original is NULL
 * Returns the buffer pointer, or NULL if an error
 * svsError holds error string
 * Also tries to read a parameter file, XXX.ini
 */

static void			/* shorten to just the base name */
bmp_name(char *n)
{
  int i;
  if (n == NULL) return;
  i = strlen(n);
  if (i < 5) return;
  if (n[i-1] == 'p' && n[i-2] == 'm' && n[i-3] == 'b' && n[i-4] == '.')
    {
      n[i-4] = 0;
      i = i-4;
      if (i < 3) return;
      if (n[i-2] == '-' && (n[i-1] == 'L' || n[i-1] == 'R'))
	n[i-2] = 0;
    }
}


EXPORT unsigned char *
svsReadFile2BMP(char *name, unsigned char *op, int *x, int *y)
{

  unsigned char buf[2000];
  int offset, size, i, width, height, ret;
  FILE *fp;
  int fd;
  int makebuf = 0;

  char full_name[255];
  char xname[250];

  /* Open first file */
  if (name == NULL) return NULL;
  strcpy(xname, name);
  bmp_name(xname);
  sprintf( full_name, "%s-L.bmp", xname );

  /* find the width and height */
  if ((fp = fopen(full_name, "rb")) == NULL)
    {
      svsError = "Can't open file";
      return NULL;
    }
  fd = fileno(fp);
#ifndef UNIX
  _setmode(fd,_O_BINARY);
#endif
  i = read(fd, buf, BMP_HEADER_SIZE+BMP_INFO_SIZE);
  if (buf[0] != 'B' || buf[1] != 'M')
    {
      svsError = "Not a BMP file";
      return NULL;
    }
  offset = read_bmp_long(&buf[10]);
  size = read_bmp_long(&buf[2]);
  if (size > BMP_MAX_SIZE)
    {
	  sprintf(read_ret, "BMP file too large: size is %d bytes", size);
	  svsError = read_ret;
	  return NULL;	
    }
  width = read_bmp_long(&buf[18]);
  height = read_bmp_long(&buf[22]);
  fclose(fp);

  /* get a new buffer if we need it */
  if (op == NULL)
    {
      makebuf = 1;
      op = (unsigned char *)malloc(width*height*2);
    }

  /* get first bitmap file */
  ret = svsReadFileBMP(full_name, op, &width, &height, 0);
  if (ret < 0) 
    {
      if (makebuf) free(op);
      return NULL;
    }

  /* get second bitmap file */
  sprintf( full_name, "%s-R.bmp", name );
  ret = svsReadFileBMP(full_name, op+width*height, &width, &height, 0);
  if (ret < 0) 
    {
      if (makebuf) free(op);
      return NULL;
    }
  *x = width;
  *y = height;
  return op;
}


/* writing a BMP */

void
write_bmp_word(unsigned char *p, int v)
{
  *p++ = v & 0xff;
  *p = (v>>8) & 0xff;
}

void
write_bmp_long(unsigned char *p, int v)
{
  *p++ = v & 0xff;
  *p++ = (v>>8) & 0xff;
  *p++ = (v>>16) & 0xff;
  *p = (v>>24) & 0xff;
}

static unsigned char fbuf[4000];


EXPORT int
svsWriteImageBMP(char *name, unsigned char *op, int width, int height, int disp)
{
  unsigned char *p;
  int i;
  int fd, n;
  FILE *fp;

  if ((fp = fopen(name, "wb")) == NULL)
    {
      svsError = "Can't open file for write";
      return -1;
    }
  fd = fileno(fp);
#ifndef UNIX
  _setmode(fd,_O_BINARY);
#endif
  fbuf[0] = 'B';
  fbuf[1] = 'M';
  write_bmp_long(&fbuf[2], width*height+BMP_HEADER_SIZE+BMP_INFO_SIZE
		 + 4*256);
  write_bmp_long(&fbuf[6], 0);
  write_bmp_long(&fbuf[10], BMP_HEADER_SIZE+BMP_INFO_SIZE+4*256);
  write_bmp_long(&fbuf[14], 40);	/* size of info header */
  write_bmp_long(&fbuf[18], width);
  write_bmp_long(&fbuf[22], height);
  write_bmp_word(&fbuf[26], 1);	/* number of planes */
  write_bmp_word(&fbuf[28], 8);	/* bits per pixel */
  write_bmp_long(&fbuf[30], 0);
  write_bmp_long(&fbuf[34], 0);
  write_bmp_long(&fbuf[38], 0);
  write_bmp_long(&fbuf[42], 0);
  write_bmp_long(&fbuf[46], 256);
  write_bmp_long(&fbuf[50], 256);
  p = fbuf + BMP_HEADER_SIZE + BMP_INFO_SIZE;

  if (!disp)			/* grayscale colormap */
    {
      for (i=0; i<256; i++)
	{
	  *p++ = i;
	  *p++ = i;
	  *p++ = i;
	  *p++ = 0;
	  }
    }
  else				/* disparity colormap, mostly grayscale */
    {
      for (i=0; i<254; i++)
	{
	  *p++ = i;
	  *p++ = i;
	  *p++ = i;
	  *p++ = 0;
	  }
      for (; i<256; i++)
	{
	  *p++ = 0;
	  *p++ = 0;
	  *p++ = 0;
	  *p++ = 0;
	}
    }
  n = write(fd, fbuf, BMP_HEADER_SIZE+BMP_INFO_SIZE+4*256);
  if (disp <= 0)		/* greyscale image */
    {
      op += width*(height-1);
      for (i=0; i<height; i++, op-=width)
	n = write(fd, (char *)op, width);
    }
  else				/* disparity image, pixels are short ints */
    {
      short *odp = (short *)op;
      char obuf[MAXX];
      int dd = disp/256;
      int j;
      if (dd <= 0) dd = 1;
      odp += width*(height-1);
      for (i=0; i<height; i++, odp-=width)
	{
	  for (j=0; j<width; j++)
	    {
	      if (odp[j] < 0)
		obuf[j] = (char)odp[j];
	      else
		obuf[j] = (char)(odp[j]/dd) + 1;
	    }
	  n = write(fd, obuf, width);      
	}
    }
  fclose( fp );
  return 0;
}


//
// Write a color (24-bit) BMP
// 
// Each line ends of 4-byte (or perhaps 8-byte?) boundary
//
// Input data is 4 bytes/pixel
// Output data is packed, 3 bytes/pixel
//
// Total size of data is line size (line width * 3 rounded up to byte
// boundary) times height.
//



EXPORT int
svsWriteImageColorBMP(char *name, unsigned char *op, int width, int height, int disp)
{
  int i, j;
  int fd, n;
  FILE *fp;
  int linewidth;
  int ret;

  if ((fp = fopen(name, "wb")) == NULL)
    {
      svsError = "Can't open file for write";
      return -1;
    }
  fd = fileno(fp);
#ifndef UNIX
// _setmode(fd,_O_BINARY);
#endif

  linewidth = 3*width;
  linewidth = (linewidth + 0x3) & ~0x3;	/* round to 4 bytes */

  fbuf[0] = 'B';
  fbuf[1] = 'M';
  write_bmp_long(&fbuf[2], linewidth*height+BMP_HEADER_SIZE+BMP_INFO_SIZE);
				//total size
  write_bmp_long(&fbuf[6], 0);	// ??


  write_bmp_long(&fbuf[10], BMP_HEADER_SIZE+BMP_INFO_SIZE);
				// 0x36 (dec 54) for 24 bit pixels

  write_bmp_long(&fbuf[14], BMP_INFO_SIZE); // size of info header 
					     // 0x28 (dec 40) for 24 bit image

  write_bmp_long(&fbuf[18], width); // line width in pixels
  write_bmp_long(&fbuf[22], height); // number of lines


  write_bmp_word(&fbuf[26], 1);	// number of planes, 0x1 for 24 bit

  write_bmp_word(&fbuf[28], 24);	// bits per pixel, 0x18 (dec 24) for 24 bit

  write_bmp_long(&fbuf[30], 0);  // compression
  write_bmp_long(&fbuf[34], linewidth*height);  // total size of data
  write_bmp_long(&fbuf[38], 0);  
  write_bmp_long(&fbuf[42], 0);  
  write_bmp_long(&fbuf[46], 0); 
  write_bmp_long(&fbuf[50], 0); 

  ret = write(fd, fbuf, BMP_HEADER_SIZE+BMP_INFO_SIZE);
  //  printf("%d", ret);
  op += 4*width*(height-1);
  for (i=0; i<height; i++, op-=4*width)
    {
      for (j=0; j<width; j++)
	{
	  fbuf[j*3] = op[j*4+2];
	  fbuf[j*3+1] = op[j*4+1];
	  fbuf[j*3+2] = op[j*4];
	}
      fbuf[j*3+3] = 0;
      fbuf[j*3+4] = 0;
      fbuf[j*3+5] = 0;
      n = write(fd, (char *)fbuf, linewidth);
    }
  fclose( fp );
  return 0;
}

static svsSP qp;

typedef enum {
  SVS_INT,
  SVS_DOUBLE,
  SVS_ARR3x3,
  SVS_ARR3x4
} DataType;

struct ventry
{
  char *name;
  DataType type;
  void *p;
};

struct ventry v_image[] = {
  {"max_linelen", SVS_INT, &qp.max_linelen}, /* Maximum image line length, in pixels */
  {"max_lines",   SVS_INT, &qp.max_lines},	/* Maximum mage lines */
  {"max_decimation", SVS_INT, &qp.max_decimation}, /* Maximum decimation: 1, 2, or 4 */
  {"max_binning",   SVS_INT, &qp.max_binning}, /* Maximum binning: 1 or 2 */
  {"max_framediv",  SVS_INT, &qp.max_framediv}, /* Maximum frame division: 1, 2 or 4 */
  {"gamma", SVS_DOUBLE, &qp.gamma},     /* Gamma correction needed on display */
  {"color_right",   SVS_INT, &qp.color_right},  /* 1 for color, 0 for monochrome */
  {"color",   SVS_INT, &qp.color},      /* 1 for color, 0 for monochrome */

  {"ix",      SVS_INT, &qp.ix},		/* Subimage start column */
  {"iy",      SVS_INT, &qp.iy},		/* Subimage start row */
  {"vergence",SVS_DOUBLE, &qp.vergence},	/* Vergence between images */
  {"rectified",SVS_INT, &qp.rectified},	/* 1 if image is rectified */
  {"width",   SVS_INT, &qp.width},	/* Subimage width, in pixels */
  {"height",  SVS_INT, &qp.height},	/* Subimage height, in pixels */
  {"linelen", SVS_INT, &qp.linelen},	/* Image line length, in pixels */
  {"lines",   SVS_INT, &qp.lines},	/* Image lines */
  {"decimation", SVS_INT, &qp.decimation}, /* Current decimation: 1, 2, or 4 */
  {"binning",   SVS_INT, &qp.binning},  /* Current binning: 1 or 2 */
  {"framediv",  SVS_INT, &qp.framediv},  /* Current framediv: 0 (off), 1, 2 or 4 */
  {"subwindow", SVS_INT, &qp.subwindow},/* 1 for subwindow capability */
  {"have_rect", SVS_INT, &qp.have_rect},/* 1 for valid rectification params */

  /* digitization parameters */

  {"autogain",  SVS_INT, &qp.autogain},	/* 1 if we're using this */
  {"autoexposure",  SVS_INT, &qp.autoexposure},	/* 1 if we're using this */
  {"autowhite", SVS_INT, &qp.autowhite}, /* 1 if we're using this */
  {"autobrightness", SVS_INT, &qp.autobrightness}, /* 1 if we're using this */
  {"gain",      SVS_INT, &qp.gain},     /* image gain from 0 to 100 */
  {"exposure",  SVS_INT, &qp.exposure}, /* image exposure from 0 to 100 */
  {"contrast",  SVS_INT, &qp.contrast}, /* image contrast from 0 to 100 */
  {"brightness",SVS_INT, &qp.brightness}, /* image brightness from 0 to 100, -1 for auto */
  {"saturation",SVS_INT, &qp.saturation}, /* color saturation from 0 to 100 */
  {"red",       SVS_INT, &qp.red}, /* image red gain offset, from -40 to 40 */
  {"blue",      SVS_INT, &qp.blue} /* image blue gain offset, from -40 to 40 */

};

struct ventry v_stereo[] = {
  {"convx",     SVS_INT, &qp.convx},     /* Edge convolution kernel size */
  {"convy",     SVS_INT, &qp.convy},     /* Same, ydim */
  {"corrxsize", SVS_INT, &qp.corrxsize}, /* Correlation window size, pixels */
  {"corrysize", SVS_INT, &qp.corrysize}, /* Same, ydim */
  {"thresh",    SVS_INT, &qp.thresh},    /* Confidence threshold, 0-40 */
  {"unique",    SVS_INT, &qp.unique},    /* Uniqueness threshold, 0-40 */
  {"lr",        SVS_INT, &qp.lr},        /* Left/right check, 1=on, 0=off */
  {"specklediff", SVS_INT, &qp.specklediff}, /* speckle filter difference */
  {"specklesize", SVS_INT, &qp.specklesize}, /* speckle extent in pixels */
  {"lr",        SVS_INT, &qp.lr},        /* Left/right check, 1=on, 0=off */
  {"ndisp",     SVS_INT, &qp.ndisp},     /* Pixel disparities to search */
  {"dpp",       SVS_INT, &qp.dpp},       /* Disparities per pixel */
  {"offx",      SVS_INT, &qp.offx},      /* x offset for horopter */
  {"offy",      SVS_INT, &qp.offy},      /* y offset, not used */
  {"frame",     SVS_DOUBLE, &qp.frame}   /* framing scale */
};

struct ventry v_external[] = {
  {"Tx", SVS_DOUBLE, &qp.Tx},		/* [mm] */
  {"Ty", SVS_DOUBLE, &qp.Ty},		/* [mm] */
  {"Tz", SVS_DOUBLE, &qp.Tz},		/* [mm] */
  {"Rx", SVS_DOUBLE, &qp.Rx},		/* [rad] Yaw */
  {"Ry", SVS_DOUBLE, &qp.Ry},		/* [rad] Pitch */
  {"Rz", SVS_DOUBLE, &qp.Rz}		/* [rad] Roll */
};

struct ventry v_leftcam[] = {
  {"pwidth", SVS_INT, &qp.left.pwidth},	 /* [pix] Width image */
  {"pheight", SVS_INT, &qp.left.pheight},/* [pix] Height image */
  {"dpx", SVS_DOUBLE, &qp.left.dpx}, 	 /* [mm/pix] width of pixel */
  {"dpy", SVS_DOUBLE, &qp.left.dpy}, 	 /* [mm/pix] height of pixel */
  {"sx", SVS_DOUBLE, &qp.left.sx},       /* Scale factor to compensate for any error in dpx */
  {"Cx", SVS_DOUBLE, &qp.left.Cx},       /* [pix] Z axis intercept of image */
  {"Cy", SVS_DOUBLE, &qp.left.Cy},  	 /* [pix] Z axis intercept of image */
  {"f", SVS_DOUBLE, &qp.left.f},	     /* [mm] Focal length */
  {"fy", SVS_DOUBLE, &qp.left.fy},       /* [mm] Focal length */
  {"alpha", SVS_DOUBLE, &qp.left.alpha},  /* skew */
  {"kappa1", SVS_DOUBLE, &qp.left.kappa1},/* [1/mm^2] 1st coeff. radial dist. */
  {"kappa2", SVS_DOUBLE, &qp.left.kappa2},/* [1/mm^4] 2nd coeff. radial dist. */
  {"kappa3", SVS_DOUBLE, &qp.left.kappa3},/* [1/mm^6] 2nd coeff. radial dist. */
  {"tau1", SVS_DOUBLE, &qp.left.tau1},  /* tangential distortion */
  {"tau2", SVS_DOUBLE, &qp.left.tau2},
  {"proj", SVS_ARR3x4, qp.left.proj},   /* projection matrix of rectified image */
  {"rect", SVS_ARR3x3, qp.left.rect}   /* rectification transform after correction for lens distortion */
};

struct ventry v_rightcam[] = {
  {"pwidth", SVS_INT, &qp.right.pwidth}, /* [pix] Width image */
  {"pheight", SVS_INT, &qp.right.pheight},/* [pix] Height image */
  {"dpx", SVS_DOUBLE, &qp.right.dpx}, 	 /* [mm/pix] width of pixel */
  {"dpy", SVS_DOUBLE, &qp.right.dpy}, 	 /* [mm/pix] height of pixel */
  {"sx", SVS_DOUBLE, &qp.right.sx},      /* Scale factor to compensate for any error in dpx */
  {"Cx", SVS_DOUBLE, &qp.right.Cx},	 /* [pix] Z axis intercept of image */
  {"Cy", SVS_DOUBLE, &qp.right.Cy},  	 /* [pix] Z axis intercept of image */
  {"f", SVS_DOUBLE, &qp.right.f},	 /* [mm] Focal length */
  {"fy", SVS_DOUBLE, &qp.right.fy},	 /* [mm] Focal length */
  {"alpha", SVS_DOUBLE, &qp.left.alpha},  /* skew */
  {"kappa1", SVS_DOUBLE, &qp.right.kappa1},/* [1/mm^2] 1st coeff.radial dist. */
  {"kappa2", SVS_DOUBLE, &qp.right.kappa2},/* [1/mm^4] 2nd coeff.radial dist. */
  {"kappa3", SVS_DOUBLE, &qp.right.kappa3},/* [1/mm^6] 2nd coeff. radial dist. */
  {"tau1", SVS_DOUBLE, &qp.right.tau1},  /* tangential distortion */
  {"tau2", SVS_DOUBLE, &qp.right.tau2},
  {"proj", SVS_ARR3x4, qp.right.proj},  /* projection matrix of rectified image */
  {"rect", SVS_ARR3x3, qp.right.rect}   /* rectification transform after correction for lens distortion */
};

struct ventry v_global[] = {
  {"GTx", SVS_DOUBLE, &qp.GTx},		/* [mm] */
  {"GTy", SVS_DOUBLE, &qp.GTy},		/* [mm] */
  {"GTz", SVS_DOUBLE, &qp.GTz},		/* [mm] */
  {"GRx", SVS_DOUBLE, &qp.GRx},		/* [rad] Yaw */
  {"GRy", SVS_DOUBLE, &qp.GRy},		/* [rad] Pitch */
  {"GRz", SVS_DOUBLE, &qp.GRz}		/* [rad] Roll */
};


struct mentry
{
  char *name;
  int  tag;
  int  size;
  struct ventry *v;
};

// don't change the order here!
static struct mentry ms[] = {
  {"image", 		1, sizeof(v_image)/sizeof(struct ventry), 	v_image}, 
  {"stereo", 		2, sizeof(v_stereo)/sizeof(struct ventry), 	v_stereo}, 
  {"external", 		3, sizeof(v_external)/sizeof(struct ventry), 	v_external},
  {"left camera", 	4, sizeof(v_leftcam)/sizeof(struct ventry), 	v_leftcam}, 
  {"right camera", 	5, sizeof(v_rightcam)/sizeof(struct ventry),	v_rightcam},
  {"global",    	6, sizeof(v_global)/sizeof(struct ventry),	v_global}
};

//
// main function for writing a param file to a string buffer
//

void bbwrite(char **p, char *s, ...)
{
  va_list ap;
  va_start(ap, s);
  int n = vsprintf(*p, s, ap);
  va_end(ap);
  *p = (*p) + n;
}

void svsWPF(char *buf, svsSP *sp)
{
  int i, j, k;
  float *ptr;

  qp = *sp;			// transfer params

  bbwrite(&buf, "# SVS Engine v 4.0 Stereo Camera Parameter File\n\n");
  for (i=0; i< sizeof(ms)/sizeof(struct mentry); i++) {
    bbwrite(&buf, "\n\n[%s]\n", ms[i].name);
    for (j=0; j<ms[i].size; j++) {
      switch ((ms[i].v)[j].type) {
      case SVS_INT:
	bbwrite(&buf, "%s %d \n", (ms[i].v)[j].name, 
		*((int *)(ms[i].v)[j].p));
	break;
      case SVS_DOUBLE:
	bbwrite(&buf, "%s %f \n", (ms[i].v)[j].name, 
		*((double *)(ms[i].v)[j].p));
	break;
      case SVS_ARR3x3:
	bbwrite(&buf, "%s \n", (ms[i].v)[j].name);
	ptr = (float *) (ms[i].v)[j].p;
	for (k = 0; k < 3; k++) {
	  bbwrite(&buf, "  %e %e %e \n", ptr[0], ptr[1], ptr[2]);
	  ptr += 3;
	}
	break;
      case SVS_ARR3x4:
	bbwrite(&buf, "%s \n", (ms[i].v)[j].name);
	ptr = (float *) (ms[i].v)[j].p;
	for (k = 0; k < 3; k++) {
	  bbwrite(&buf, "  %e %e %e %e \n", ptr[0], ptr[1], ptr[2], ptr[3]);
	  ptr += 4;
	}
	break;
      }
    }
  }
}


/* writes out a full parameter file */

EXPORT int
svsWriteParamFile(char *fname, svsSP *sp)
{
  FILE *fout;
  int i;
  char *bb;

  if (sp == NULL) return -1;
  fout = fopen(fname, "w");
  if (fout == NULL) return -1;	/* couldn't open it */
  
  bb = (char *)malloc(10000);
  svsWPF(bb, sp);

  i = 0;
  while ((bb[i] != 0) && i < 10000)
    fputc(bb[i++], fout);
  fclose(fout);
  if (i >= 10000)
    return -1;
  return 0;
}

EXPORT int
svsWriteParamBuffer(char *bb, svsSP *sp)
{
  svsWPF(bb, sp);
  return 0;
}


/* writes out rectification parameter file */

void svsWRF(char *buf, svsSP *sp)
{
  int i, j, k;
  float *ptr;

  bbwrite(&buf, "# SVS Engine v 4.0 Stereo Camera Parameter File\n\n");

  // write that we have a rectification
  bbwrite(&buf, "\n\n[image]\n");
  bbwrite(&buf, "have_rect 1 \n");

  // write framing at default 1.0
  bbwrite(&buf, "\n\n[stereo]\n");
  bbwrite(&buf, "frame 1.0 \n");

  // now write just the rectification params
  for (i=2; i< sizeof(ms)/sizeof(struct mentry); i++) {
    bbwrite(&buf, "\n\n[%s]\n", ms[i].name);
    for (j=0; j<ms[i].size; j++) {
      switch ((ms[i].v)[j].type) {
      case SVS_INT:
	bbwrite(&buf, "%s %d \n", (ms[i].v)[j].name, 
		*((int *)(ms[i].v)[j].p));
	break;
      case SVS_DOUBLE:
	bbwrite(&buf, "%s %f \n", (ms[i].v)[j].name, 
		*((double *)(ms[i].v)[j].p));
	break;
      case SVS_ARR3x3:
	bbwrite(&buf, "%s \n", (ms[i].v)[j].name);
	ptr = (float *) (ms[i].v)[j].p;
	for (k = 0; k < 3; k++) {
	  bbwrite(&buf, "  %e %e %e \n", ptr[0], ptr[1], ptr[2]);
	  ptr += 3;
	}
	break;
      case SVS_ARR3x4:
	bbwrite(&buf, "%s \n", (ms[i].v)[j].name);
	ptr = (float *) (ms[i].v)[j].p;
	for (k = 0; k < 3; k++) {
	  bbwrite(&buf, "  %e %e %e %e \n", ptr[0], ptr[1], ptr[2], ptr[3]);
	  ptr += 4;
	}
	break;
      }
    }
  }
}



EXPORT int
svsWriteRectFile(char *fname, svsSP *sp)
{
  FILE *fout;
  int i;
  char *bb;

  if (sp == NULL) return -1;
  qp = *sp;                     /* copy to buffer structure */
  fout = fopen(fname, "w");
  if (fout == NULL) return -1;	/* couldn't open it */
  
  bb = (char *)malloc(10000);
  svsWRF(bb, sp);

  i = 0;
  while ((bb[i] != 0) && i < 10000)
    fputc(bb[i++], fout);
  fclose(fout);
  if (i >= 10000)
    return -1;
  return 0;
}



//
// reads one line from a string buffer
//


char *bbgets(char *buf, int n, char **p)
{
  char *s = *p;
  char *ret = buf;
  int i = 0;
  if (s == NULL || *s == 0)
    return NULL;
  while (*s != 0 && *s != '\n' && *s != 0x0a && *s != 0x0d && n-- > 0)
    *buf++ = *s++;
  if (*s == '\n' || *s == 0x0a || *s == 0x0d)
    s++;
  *buf = 0;
  *p = s;
  return ret;
}

/* 
 * reads in a parameter file 
 * returns -1 if it can't open the file
 * returns -2 if the header isn't a valid parameter file header
 * returns -3 if the parameter file version is too old for this release
 */

int
svsRPF(char *bb, svsSP *sp)
{
  int i, j, v, vv, heading = -1;
  int lnum = 1;
  unsigned int n;
  float *ptr;
  char s[128], *p, *ss;

  //  printf("[ImageIO] Reading param file\n");

  /* check header for correct version */
  p=bbgets(s,120,&bb);		
  if (p == NULL) 
    {
      svsDebugMessage("[ImageIO] No header in param file");
      return -1;	/* no header */
    }
  if (strlen(p) < 20) 
    {
      svsDebugMessage("[ImageIO] No header in param file");
      return -2;	/* no header */
    }
  if (STRNCASECMP("# SVS Engine v ", p, 15) != 0)
    {
      svsDebugMessage("[ImageIO] No header in param file");
      return -2;	/* no header */
    }

  v = p[15] - '0';		/* major release number */
  vv = p[17] - '0';		/* minor release number */
  if ((v*10+vv) < 21)
    {
      svsDebugMessage("[ImageIO] Param file version too early: %d.%d", v, vv);      
      return -3;		/* too early a release */
    }

  /* defaults */
  sp->left.fy = sp->right.fy = -1.0; /* default, old params */
  sp->frame = 1.0;
  sp->gamma = 0.85;
  sp->GTx = sp->GTy = sp->GTz = sp->GRx = sp->GRy = sp->GRz = 0.0;
  sp->left.kappa1 = sp->right.kappa1 = 0.0;
  sp->left.kappa2 = sp->right.kappa2 = 0.0;
  sp->left.kappa3 = sp->right.kappa3 = 0.0;
  sp->left.tau1 = sp->right.tau1 = 0.0;
  sp->left.tau2 = sp->right.tau2 = 0.0;

  while ((p=bbgets(s,120,&bb))!=NULL) 
    {
      lnum++;			// line number

      if (s[0] == ' ' || s[0] == '#' || s[0] == '\n' || s[0] == '\r')	/* comment */
	continue;
      if (s[0]=='[') 
	{	  		/* new heading */
	  heading = 0;
	  for (i=0; i<sizeof(ms)/sizeof(struct mentry); i++)
	    if (STRNCASECMP(ms[i].name, &s[1], strlen(ms[i].name)) == 0) 
	      {	/* match */
		heading = i;
		break;
	      }
	  continue;      /* continue past heading */
	}
      if (heading == -1) continue;	/* not a valid heading */
      for (j=0; j<ms[heading].size; j++) 
	{
	  ss = (ms[heading].v)[j].name;
	  n = 0;
	  while (s[n] && s[n] != ' ' && s[n] != '\n' && s[n] != '\t')
	    n++;
	  if (strlen(ss) == n && STRNCASECMP(ss, s, n) == 0) 
	    {	/* match */
	      switch ((ms[heading].v)[j].type) {
	      case SVS_INT:
		if (sscanf(&s[strlen(ss)], " %d", (ms[heading].v)[j].p) != 1)
		  {
		    svsDebugMessage("[ImageIO] Error reading int, line %d", lnum);
		    return -1;
		  }
		break;
	      case SVS_DOUBLE:
		if (sscanf(&s[strlen(ss)], " %lf", (ms[heading].v)[j].p) != 1)
		  {
		    svsDebugMessage("[ImageIO] Error reading double, line %d", lnum);
		    return -1;
		  }
		break;

	      case SVS_ARR3x3:
		ptr = (float *) (ms[heading].v)[j].p;
		for (i = 0; i < 3; i++) 
		  {
		    if (bbgets(s,120,&bb) != NULL) 
		      {
			lnum++;
			if (sscanf (s, "%f %f %f", &ptr[0], &ptr[1], &ptr[2]) != 3) {
			  {
			    svsDebugMessage("[ImageIO] Error reading array, line %d", lnum);
			    return -1;
			  }
			}
			ptr += 3;
		      }
		  }
		break;

	      case SVS_ARR3x4:
		ptr = (float *) (ms[heading].v)[j].p;
		for (i = 0; i < 3; i++) 
		  {
		    if (bbgets(s,120,&bb) != NULL) 
		      {
			lnum++;
			if (sscanf (s, "%f %f %f %f", &ptr[0], &ptr[1], &ptr[2], &ptr[3]) != 4) 
			  {
			    svsDebugMessage("[ImageIO] Error reading array, line %d", lnum);
			    return -1;
			  }
		      }
		    ptr += 4;
		  }
		break;
	      }
	      break;
	    }
	}
    }
  qp.dpp = SUBPIXELS;		// this is always set by SVS 
  *sp = qp;			// copy to argument 
  return 0;
}


EXPORT int
svsReadParamFile(char *fname, svsSP *sp)
{
  FILE *fout;
  int i, j;
  char *bb;

  if (sp == NULL) return -1;
  qp = *sp;					/* copy defaults to buffer */
  fout = fopen(fname, "r");
  if (fout == NULL) return -1;	/* couldn't open it */
  
  bb = new char[10000];
  i = 0;
  while ((j = fgetc(fout)) != EOF && i < 9998)
    bb[i++] = j;
  if (i >= 9998)
    return -1;
  fclose(fout);
  bb[i++] = 0;
  bb[i++] = 0;

  svsDebugMessage("[ImageIO] Reading param file %s", fname);

  // call the main function
  i = svsRPF(bb, sp);
  delete [] bb;
  return i;
}

EXPORT int
svsReadParamBuffer(char *bb, svsSP *sp)
{
  int i = svsRPF(bb, sp);
  return i;
}


//
// 3D point output
// Each point is a single line, X,Y,Z in mm, 3 colors in [0,255] range
// Then, disparity and image x,y

void
print_one(FILE *fp, float x, float y, float z, int cr, int cg, int cb, int disp,
	  int ix, int iy)
{
  fprintf(fp, "%06f %06f %06f  %d %d %d [%d %d] d%d\n", x, y, z, cr, cg, cb,
	  ix, iy, disp);
}

// point cloud format

int
svsWrite3DCloud(char *fname, svs3Dpoint *pts, unsigned char *image, 
		short *disp, int type, svsSP *sp, int nump)
{
  int i,j, k;
  FILE *fout;
  int w = sp->width;
  int h = sp->height;
  svs3Dpoint *pt;

  fout = fopen(fname, "w");
  if (fout == NULL) return -1;	// couldn't open it
  
  fprintf(fout, "# SVS Engine v 4.0 3D Point Cloud File\n\n");
  fprintf(fout, "numPoints %d\n\n", nump);
  
  for (j=0; j<h; j++)
    {
      k = j*w;
      pt = pts + k;
      for (i=0; i<w; i++, k++, pt++)
	{
	  if (pt->A > 0.0)	// have an entry 
	    {
	      if (type == svsCOLOR)
		print_one(fout, pt->X, pt->Y, pt->X, 
                          image[k*4], image[k*4+1], image[k*4+2],
                          disp[k], i, j);
	      else
		print_one(fout, pt->X, pt->Y, pt->X, 
                          image[k], image[k], image[k],
                          disp[k], i, j);
	    }
	}
    }
  fclose(fout);
  return 0;
}


// array format

int
svsWrite3DArray(char *fname, svs3Dpoint *pts, unsigned char *image, 
		short *disp, int type, svsSP *sp)
{
  int i,j, k;
  FILE *fout;
  int w = sp->width;
  int h = sp->height;
  svs3Dpoint *pt;

  fout = fopen(fname, "w");
  if (fout == NULL) return -1;	// couldn't open it
  
  fprintf(fout, "# SVS Engine v 4.1 3D Point Array File\n\n");
  fprintf(fout, "width %d\nheight %d\n\n", w, h);
  
  for (j=0; j<h; j++)
    {
      k = j*w;
      pt = pts + k;
      for (i=0; i<w; i++, k++, pt++)
	{
	  if (type == svsCOLOR)
	    print_one(fout, pt->X, pt->Y, pt->Z, 
		      image[k*4], image[k*4+1], image[k*4+2],
                      disp[k], i, j);
	  else
	    print_one(fout, pt->X, pt->Y, pt->Z, 
		      image[k], image[k], image[k],
                      disp[k], i, j);
	}
    }
  fclose(fout);
  return 0;
}

//Colors is between 0 to 1
//n3d is number of points
//x3d y3d z3d are the x y z coordinates
//colors0 colors1 colors2 are the three colors between 0 and 1 

int
svsWriteWRL(char *wrl_file, svs3Dpoint *pts,
	    unsigned char *image, int type, svsSP *sp)
{
  FILE *ff;
  int i,j,k;
  float xx, yy, zz;
  svs3Dpoint *pt;

  int w = sp->width;
  int h = sp->height;

  ff=fopen(wrl_file, "w");
  if (ff == NULL) return -1;	// couldn't open it
  
  fprintf(ff, "#VRML V2.0 utf8\n");
  fprintf(ff, "DEF InitView Viewpoint\n{\n\tposition 0 0 0\n\t \
orientation 1 0 0 3.14159\n\n}");
  fprintf(ff, "DEF World Transform \n{\n   translation 0 0 0\n \
   children  [\n      DEF Tour Transform \n \
      {\n	 children [\n \
	    DEF Viewer Viewpoint { position 0 0 0 }\n	 ]\n \
      }\n\n      DEF Scene Transform\n      {\n	 translation 0 0 0\n \
	 children [\n");

  fprintf(ff, "Shape\n{\n\tgeometry PointSet\n\t{\n");
  fprintf(ff, "\t\tcoord Coordinate\n\t\t{\n");
  fprintf(ff, "\t\t\tpoint[\n");

  for (j=0; j<h; j++)
  {
      k = j*w;
      pt = pts + k;
      for (i=0; i<w; i++, k++, pt++)
      {
	if (pt->A > 0.0)	// have an entry 
	    {
	      xx = pt->X; yy = pt->Y; zz = pt->Z;
	      fprintf(ff, "\t\t\t\t%f %f %f, \n",(float)xx, (float)yy, (float)zz);
	    }
      }
  }
  fprintf(ff, "\t\t\t\t%f %f %f\n",(float)xx, (float)yy, (float)zz);
  fprintf(ff, "\t\t\t     ]\n");
  fprintf(ff, "\t\t}\n");
  
  fprintf(ff, "\t\tcolor Color\n\t\t{\n");
  fprintf(ff, "\t\t\tcolor[\n");

#define FACT ((float)(1.0/256.0))
  for (j=0; j<h; j++)
    {
      k = j*w;
      pt = pts + k;
      for (i=0; i<w; i++, k++, pt++)
	{
	  if (pt->A > 0.0)	// have an entry 
	    {
	      if (type == svsCOLOR)
	      {
		xx = (float)image[k*4]; yy = (float)image[k*4+1]; zz = (float)image[k*4+2];
	      }
	      else
	      {
		xx = (float)image[k]; yy = xx; zz = xx;
	      }
	      fprintf(ff, "\t\t\t\t%f %f %f, \n", xx*FACT, yy*FACT, zz*FACT);
	    }
	}
    }

  fprintf(ff, "\t\t\t\t%f %f %f\n", xx*FACT, yy*FACT, zz*FACT);
  fprintf(ff, "\t\t\t      ]\n");
  fprintf(ff, "\t\t}\n");
  fprintf(ff,"\t}\n}\n");

  // end of geometry section
  fprintf(ff,"\n	   ]\n \
      } # end of scene transform\n\
    ]\n \
 } # end of world transform\n \
 \n \
DEF Timer TimeSensor \n \
{ \n\
   cycleInterval 20 \n \
   loop FALSE\n \
#   stopTime 40\n \
#   startTime 0\n \
}\n\
\n\
DEF Translator PositionInterpolator {\n\
  key [0, 0.5, 1]\n\
  keyValue [0 0 0, 2 0 -2, -2 0 -2]\n\
}# end of Translator  \n\
\n\
DEF Rotator OrientationInterpolator {\n\
  key [0, 0.5, 1]\n\
  keyValue [1 0 0 3.14159, 1 0 0.2 3.14159, 1 0 -0.2 3.14159]\n\
}\n\
\n\
# Dummy object we can click on to start animation\n\
DEF Bottom Transform {\n\
      translation 0 0 20\n\
      children  [\n\
        Shape  {\n\
          geometry Box {}\n\
          appearance  Appearance {\n\
            material Material { diffuseColor 0 0 0 }\n\
	  }\n\
	}\n\
	 DEF StartTour TouchSensor {}\n\
      ]\n\
    }# end of Bottom transform--blue\n\
\n\
ROUTE StartTour.touchTime TO Timer.startTime\n\
ROUTE Timer.isActive TO Viewer.set_bind\n\
ROUTE Timer.fraction_changed TO Translator.set_fraction\n\
ROUTE Timer.fraction_changed TO Rotator.set_fraction\n\
ROUTE Translator.value_changed TO Tour.set_translation\n\
ROUTE Rotator.value_changed TO Tour.set_rotation\n" \
	  );

  fprintf(ff, "NavigationInfo { type \"EXAMINE\"}\n");
  fclose(ff);

  return 0;
}



// array format

int
svsWriteDisparity(char *fname, short *disp, svsSP *sp)
{
  int i,j, k;
  FILE *fout;
  int w = sp->width;
  int h = sp->height;

  fout = fopen(fname, "w");
  if (fout == NULL) return -1;	// couldn't open it
  
  fprintf(fout, "# SVS Engine v 4.0 Disparity File\n\n");
  fprintf(fout, "width %d\nheight %d\n\n", w, h);
  
  for (j=0; j<h; j++)
    {
      k = j*w;
      for (i=0; i<w; i++, k++)
        fprintf(fout, "%d ", disp[k]);
      fprintf(fout, "\n");
    }
  fclose(fout);
  return 0;
}


//
// write log file
// start by giving the log file name (with or without the .ssf extension)
//   to svsStreamFile().
// continue by using svsWriteStreamImage()
// end by using svsStreamClose()
//

#define HDR_SIZE 64
static FILE *logFP = NULL;
static int logFD = -1;
static char logName[256];
static long long fsize;
static int bufsize = (640*480*2+HDR_SIZE)*200; // 200 buffers
static int blocksize = 640*480*2 + HDR_SIZE;
static char *diskbuf = NULL;
static char *startptr;		// ring buffer ptrs
static char *endptr;
static bool empty;
static int missed;
static int framenum;
static int user_size = 0;


// disk writing thread
// parameter is not used
#ifdef UNIX
void *write_thread(void *parameter)
#else
DWORD WINAPI write_thread(void *parameter)
#endif
{
  int ret;
  int frames = 0;
  int ff = 0;
  printf("Starting thread\n");


  for (;;)
    {
      mlock(dlock);
      if (!empty)
	{
	  // write data
	  munlock(dlock);	    
//	  printf("Write: eptr %x\n", endptr);
	  ret = write(logFD,endptr,blocksize);
	  if (frames > 10)
	    {
	      //		fdatasync(logFD);
	      fflush(logFP);
	      frames = 0;
	    }
	  frames++;

	  mlock(dlock);	    
	  if (ret != blocksize)
	    //		printf("Small write: %d/%d\n", ret, blocksize);
	    {}
	  fsize += blocksize;
//	  printf("Not empty\n");
	  endptr += blocksize;
	  if (endptr >= diskbuf+bufsize) // check overflow
	    {
	      endptr = diskbuf;
	    }
	  ff++;
//	  printf("Eptr: %x  Sptr: %x  frame: %d\n", endptr, startptr, ff);
	  if (endptr == startptr)
	    {
//	      printf("Setting empty\n");
	      empty = true;
	    }
	}
      munlock(dlock);

      // check for end of writing
      if (logFP == NULL)
	break;

      // check if we have too large a file
      if (fsize > (2e9 - 1e6))
	{
	  fclose(logFP);

	  int n = strlen(logName);
	  int x = logName[n-5] - '0';	// last digit
	  if (x == 9)		// change first digit
	    {
	      logName[n-6] += 1;
	      logName[n-5] = '0';
	    }
	  else
	    logName[n-5] += 1;

	  if ((logFP = fopen(logName, "wb")) == NULL)
	    {
	      svsError = "Can't open file for write";
	      return NULL;
	    }

	  logFD = fileno(logFP);
#ifndef UNIX
	  _setmode(logFD,_O_BINARY);
#endif
	  fsize = 0;
	}


    }
  printf("Ending thread\n");
  return NULL;
}

EXPORT int
svsStreamFile(char *name, int width, int height, int bnum, int usize)
{
  unsigned char *p;
  int i;
  int fd, n;

  n = strlen(name);
  memcpy(logName, name, n);

  // get down to base name
  if (n > 4 && (strcmp(&name[n-4], ".ssf") == 0))
    n = n-4;

  logName[n] = '_';
  logName[n+1] = '0';
  logName[n+2] = '0';
  logName[n+3] = '.';
  logName[n+4] = 's';
  logName[n+5] = 's';
  logName[n+6] = 'f';
  logName[n+7] = 0;
  n += 7;

  if (logFP)
    fclose(logFP);

  if (dthread)
    {
      // stop the thread here and delete it...
      //      stopthread(dthread);
      delete dthread;
      dthread = NULL;
    }

  if (dlock)
    {
      delete dlock;
      dlock = NULL;
    }

  if ((logFP = fopen(logName, "wb")) == NULL)
    {
      svsError = "Can't open file for write";
      return -1;
    }
  logFD = fileno(logFP);

  // could use fcntl() to set NONBLOCK access
  // not sure we need it

  // set disk buffer
  user_size = usize;
  if (diskbuf)
    free(diskbuf);
  blocksize = width*height*2 + HDR_SIZE + usize;
  bufsize = blocksize*bnum;
  diskbuf = (char *)malloc(bufsize);
  if (diskbuf == NULL)
    {
      svsError = "Can't allocate disk buffer";
      return -1;
    }

  framenum = 0;
  startptr = endptr = diskbuf;
  empty = true;
  fsize = 0;
  missed = 0;
#ifndef UNIX
  _setmode(logFD,_O_BINARY);
#endif

  // maybe print a header...

  // start disk thread
  dlock = startmutex("dlock");
  dthread = startthread(write_thread);
	
  return logFD;
}

// write to circular buffer

EXPORT int
svsWriteStreamImage(int type, unsigned char *buf, int width, int height, unsigned char *user)
{
  int ret;
  if (!logFP)
    return -1;

  if (blocksize == 0)
    blocksize = width*height*2+HDR_SIZE+user_size; // save block size

  // increment frame number
  framenum++;

  // check for buffer full
  mlock(dlock);
  if (startptr == endptr && !empty)
    {
      missed++;
      printf("Buffer full %d\n", missed);
      munlock(dlock);
      return -1;
    }
  munlock(dlock);


  // write header
  char hd[256];
  ret = sprintf(startptr, "#%d %d %d %d %d ", HDR_SIZE+user_size, framenum, type, width, height);

  // write image
  memcpy(startptr+HDR_SIZE+user_size, buf, blocksize-HDR_SIZE-user_size);

  // write user info
  if (user_size > 0 && user != NULL)
    memcpy(startptr+HDR_SIZE, user, user_size);

  // sizes
  mlock(dlock);
  startptr += blocksize;
  if (startptr > diskbuf+bufsize-blocksize) // end of buffer
    startptr = diskbuf;

  empty = false;
  munlock(dlock);
//  printf("Ptr: %x\n", startptr);


  // return
  return ret;
}

EXPORT int
svsStreamClose()
{
  if (!logFP)
    return -1;
  mlock(dlock);
  fclose(logFP);
  logFP = NULL;
  munlock(dlock);
  return 0;
}


// open a streaming file

EXPORT int
svsStreamOpen(char *name)
{
  unsigned char *p;
  int i;
  int fd, n;

  n = strlen(name);
  memcpy(logName, name, n);
  // get down to base name
  if (n <= 4 || (n > 4 && (strcmp(&name[n-4], ".ssf"))))
    {
      logName[n] = '_';
      logName[n+1] = '0';
      logName[n+2] = '0';
      logName[n+3] = '.';
      logName[n+4] = 's';
      logName[n+5] = 's';
      logName[n+6] = 'f';
      n += 7;
    }      

  if ((logFP = fopen(logName, "rb")) == NULL)
    {
      svsError = "Can't open file for read";
      return -1;
    }
  logFD = fileno(logFP);

  // could use fcntl() to set NONBLOCK access
  // not sure we need it

  framenum = 0;
  fsize = 0;
#ifndef UNIX
  _setmode(logFD,_O_BINARY);
#endif

  return logFD;
}


// read one image from stream file

EXPORT int
svsReadStreamImage(int *type, unsigned char *buf, int *framenum, int *width, int *height, unsigned char *user)
{
  int ret;
  int size;
  char hdr[HDR_SIZE];

  if (logFP == NULL)
    return -1;

  // read header
  ret = read(logFD,hdr,HDR_SIZE);
  if (ret < HDR_SIZE)		// can't read header, check for next file
    {
      fclose(logFP);

      int n = strlen(logName);
      int x = logName[n-5] - '0'; // last digit
      if (x == 9)		// change first digit
	{
	  logName[n-6] += 1;
	  logName[n-5] = '0';
	}
      else
	logName[n-5] += 1;

      if ((logFP = fopen(logName, "rb")) == NULL)
	{
	  svsError = "Can't open file for write";
	  return -1;
	}

      logFD = fileno(logFP);
#ifndef UNIX
      _setmode(logFD,_O_BINARY);
#endif
      ret = read(logFD,hdr,HDR_SIZE);
      if (ret < HDR_SIZE)
	{
	  fclose(logFP);
	  logFP = NULL;
	  return -1;
	}
    }

  // parse header
  ret = sscanf(hdr, "#%d %d %d %d %d ", &size, framenum, type, width, height);
  if (ret < 5)
    {
      svsDebugMessage("[ImageIO] Stream file header parse error");
      fclose(logFP);
      logFP = NULL;
      return -1;
    }

  // read user info
  int len = size - HDR_SIZE;
  if (size > HDR_SIZE)		// have user info
    {
      if (user != NULL)
	ret = read(logFD,user,len);
      else
	ret = read(logFD,buf,len);

      if (ret != len)
	{
	  svsDebugMessage("[ImageIO] Stream file user header parse error");
	  fclose(logFP);
	  logFP = NULL;
	  return -1;
	}
    }

  // read image
  len = 2 * (*width) * (*height);
  ret = read(logFD,buf,len);
  if (ret != len)
    {
      svsDebugMessage("[ImageIO] Stream file image read error");
      fclose(logFP);
      logFP = NULL;
      return -1;
    }

    return 0;
}



//
// Debugging output
//

EXPORT void (* svsDebug)(char *str) = NULL;    // call to print a debug string

static char mess[512];

void
svsDebugMessage(char *str, ...)
{
  va_list ap;
  if (svsDebug)
    {
      va_start(ap,str);
      vsprintf(mess, str, ap);
      va_end(ap);
      (*svsDebug)(mess);
    }
}


