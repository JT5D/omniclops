//-----------------------------------
// Unwrapping Omnidirectional Images
//-----------------------------------
// Author: Andrzej Pronobis
// Contact: pronobis@csc.kth.se
//          www.csc.kth.se/~pronobis
//-----------------------------------

#include "unwarp.h"

unwarp::unwarp() {
}

unwarp::~unwarp() {
}

int unwarp::rnd(double d)
{
  return static_cast<int>(d+0.5);
}

int unwarp::rndf(float d)
{
  return static_cast<int>(d+0.5);
}

inline double unwarp::linear(double v0, double v1, double x)
{
  return (v1-v0)*x+v0;
}

double unwarp::bilinear(double v00, double v01, double v10, double v11, double x, double y)
{
  // Notation: vXY
  // Interpolate in X direction
  double vX0=linear(v00, v10, x);
  double vX1=linear(v01, v11, x);
  // Interpolation in Y direction
  return linear(vX0, vX1, y);
}

inline double unwarp::cubic(double v_1, double v0, double v1, double v2, double x)
{
  double a0, a1, a2, a3, x2;
  x2 = x*x;
  a0 = v2 - v1 - v_1 + v0;
  a1 = v_1 - v0 - a0;
  a2 = v1 - v_1;
  a3 = v0;

  return (a0*x*x2+a1*x2+a2*x+a3);
}

double unwarp::bicubic(double v_1_1, double v0_1, double v1_1, double v2_1,
               double v_10, double v00, double v10, double v20,
               double v_11, double v01, double v11, double v21,
               double v_12, double v02, double v12, double v22,
               double x, double y)
{
  // Notation: vXY
  // Interpolate in X direction
  double vX_1=cubic(v_1_1, v0_1, v1_1, v2_1, x);
  double vX0 =cubic(v_10,  v00,  v10,  v20, x);
  double vX1 =cubic(v_11,  v01,  v11,  v21, x);
  double vX2 =cubic(v_12,  v02,  v12,  v22, x);
  // Interpolation in Y direction
  return cubic(vX_1, vX0, vX1, vX2, y);
}

unsigned char unwarp::getInterpolation(
	bool obilinear, bool obicubic,
	IplImage* inputImg,
	int channel,
	double x,
	double y)
{
  // Notation: vXY
  // Get channel values for both interpolation types
  int x0=static_cast<int>(floor(x));
  int y0=static_cast<int>(floor(y));
  double v00=static_cast<double>(*reinterpret_cast<unsigned char *>
        (inputImg->imageData + y0*inputImg->widthStep+x0*3+channel));
  double v01=static_cast<double>(*reinterpret_cast<unsigned char *>
        (inputImg->imageData + (y0+1)*inputImg->widthStep+x0*3+channel));
  double v10=static_cast<double>(*reinterpret_cast<unsigned char *>
        (inputImg->imageData + y0*inputImg->widthStep+(x0+1)*3+channel));
  double v11=static_cast<double>(*reinterpret_cast<unsigned char *>
        (inputImg->imageData + (y0+1)*inputImg->widthStep+(x0+1)*3+channel));

  // Interpolate
  double interpResult=0;
  if (obilinear)
  { // BILINEAR INTERPOLATION
    interpResult = bilinear(v00, v01, v10, v11, x-static_cast<double>(x0), y-static_cast<double>(y0));
  }
  else if (obicubic)
  { // BICUBIC INTERPOLATION
    // Get additional channel values
    double v_1_1=static_cast<double>(*reinterpret_cast<unsigned char *>
          (inputImg->imageData + (y0-1)*inputImg->widthStep+(x0-1)*3+channel));
    double v0_1=static_cast<double>(*reinterpret_cast<unsigned char *>
          (inputImg->imageData + (y0-1)*inputImg->widthStep+(x0)*3+channel));
    double v1_1=static_cast<double>(*reinterpret_cast<unsigned char *>
          (inputImg->imageData + (y0-1)*inputImg->widthStep+(x0+1)*3+channel));
    double v2_1=static_cast<double>(*reinterpret_cast<unsigned char *>
          (inputImg->imageData + (y0-1)*inputImg->widthStep+(x0+2)*3+channel));

    double v_10=static_cast<double>(*reinterpret_cast<unsigned char *>
          (inputImg->imageData + (y0)*inputImg->widthStep+(x0-1)*3+channel));
    double v20=static_cast<double>(*reinterpret_cast<unsigned char *>
          (inputImg->imageData + (y0)*inputImg->widthStep+(x0+2)*3+channel));

    double v_11=static_cast<double>(*reinterpret_cast<unsigned char *>
          (inputImg->imageData + (y0+1)*inputImg->widthStep+(x0-1)*3+channel));
    double v21=static_cast<double>(*reinterpret_cast<unsigned char *>
          (inputImg->imageData + (y0+1)*inputImg->widthStep+(x0+2)*3+channel));

    double v_12=static_cast<double>(*reinterpret_cast<unsigned char *>
          (inputImg->imageData + (y0+2)*inputImg->widthStep+(x0-1)*3+channel));
    double v02=static_cast<double>(*reinterpret_cast<unsigned char *>
          (inputImg->imageData + (y0+2)*inputImg->widthStep+(x0)*3+channel));
    double v12=static_cast<double>(*reinterpret_cast<unsigned char *>
          (inputImg->imageData + (y0+2)*inputImg->widthStep+(x0+1)*3+channel));
    double v22=static_cast<double>(*reinterpret_cast<unsigned char *>
          (inputImg->imageData + (y0+2)*inputImg->widthStep+(x0+2)*3+channel));

    // Perform interpolation
    interpResult = bicubic(v_1_1, v0_1, v1_1, v2_1,
                           v_10,  v00,  v10,  v20,
                           v_11,  v01,  v11,  v21,
                           v_12,  v02,  v12,  v22,
                           x-static_cast<double>(x0), y-static_cast<double>(y0));
  }

  // Check result before conversion
  if (interpResult<0)
    interpResult=0;
  if (interpResult>255)
    interpResult=255;

  return static_cast<unsigned char>(interpResult);
}


void unwarp::update(
    int cx, // centre x coordinate
    int cy, // centre y coordinate
    int ri, // inner radius
    int ro, // outer radius
	bool interpolation,
	bool obilinear, // use bilinear interpolation
	bool obicubic, //use bicubic interpolation
	double sx, // produce output scaled horizontally by <x_ratio>
	double sy, // produce output scaled vertically by <y_ratio>
	IplImage* inputImg,
	IplImage* outputImg)
{
  // Create the unwrap image
  int uwWidth = static_cast<int>(ceil((ro * 2.0 * 3.1415926535897932384626)*sx));
  int uwHeight = static_cast<int>(ceil((ro-ri + 1)*sy));
  IplImage* unwrappedImg = outputImg;

  // Perform unwrapping
  for (int uwX=0; uwX<uwWidth; ++uwX)
    for (int uwY=0; uwY<uwHeight; ++uwY)
  {
    // Convert polar to cartesian
    double w=-static_cast<double>(uwX)*2.0*3.1415926535897932384626/static_cast<double>(uwWidth);
    double r=static_cast<double>(ri) +
             static_cast<double>(uwHeight-uwY)*static_cast<double>(ro-ri + 1)/static_cast<double>(uwHeight);
    double iX=r*cos(w)+cx;
    double iY=r*sin(w)+cy;

    // Do safety check
    if ((iX<1) || (iX>inputImg->width-2) || (iY<1) || (iY>inputImg->height-2))
    {
      *(unwrappedImg->imageData + uwY*unwrappedImg->widthStep+uwX*3+0) = 0;
      *(unwrappedImg->imageData + uwY*unwrappedImg->widthStep+uwX*3+1) = 0;
      *(unwrappedImg->imageData + uwY*unwrappedImg->widthStep+uwX*3+2) = 0;
    }
    else // Tansform image data
    {
      if (interpolation)
      { // With interpolation
        *reinterpret_cast<unsigned char *>(unwrappedImg->imageData + uwY*unwrappedImg->widthStep+uwX*3+0) =
            getInterpolation(obilinear, obicubic, inputImg, 0, iX, iY);
        *reinterpret_cast<unsigned char *>(unwrappedImg->imageData + uwY*unwrappedImg->widthStep+uwX*3+1) =
            getInterpolation(obilinear, obicubic, inputImg, 1, iX, iY);
        *reinterpret_cast<unsigned char *>(unwrappedImg->imageData + uwY*unwrappedImg->widthStep+uwX*3+2) =
            getInterpolation(obilinear, obicubic, inputImg, 2, iX, iY);
      }
      else
      { // No interpolation
        int tmpX=rnd(iX);
        int tmpY=rnd(iY);
        *(unwrappedImg->imageData + uwY*unwrappedImg->widthStep+uwX*3+0) =
          *(inputImg->imageData + tmpY*inputImg->widthStep+tmpX*3+0);
        *(unwrappedImg->imageData + uwY*unwrappedImg->widthStep+uwX*3+1) =
          *(inputImg->imageData + tmpY*inputImg->widthStep+tmpX*3+1);
        *(unwrappedImg->imageData + uwY*unwrappedImg->widthStep+uwX*3+2) =
          *(inputImg->imageData + tmpY*inputImg->widthStep+tmpX*3+2);
      } // if
    } // if
  } // for
}
