//-----------------------------------
// Unwrapping Omnidirectional Images
//-----------------------------------
// Author: Andrzej Pronobis
// Contact: pronobis@csc.kth.se
//          www.csc.kth.se/~pronobis
//-----------------------------------

#include <iostream>
#include "cv.h"
#include "highgui.h"
using namespace std;

struct Options
{
  int cx, cy, ri, ro;
  string filtered, test;
  bool interpolation, bilinear, bicubic;
  double sx, sy;
  bool fixedCenter;
  bool unwrap, createTest;
  int minRadius, maxRadius;
};

class unwarp {
private:
	static void createTestImage(const Options &opt, IplImage* inputImg);
	static unsigned char getInterpolation(
		bool obilinear, bool obicubic,
		IplImage* inputImg,
		int channel,
		double x,
		double y);
	static double bicubic(double v_1_1, double v0_1, double v1_1, double v2_1,
	               double v_10, double v00, double v10, double v20,
	               double v_11, double v01, double v11, double v21,
	               double v_12, double v02, double v12, double v22,
	               double x, double y);
	static inline double cubic(double v_1, double v0, double v1, double v2, double x);
	static double bilinear(double v00, double v01, double v10, double v11, double x, double y);
	static inline double linear(double v0, double v1, double x);
	static int rndf(float d);
	static int rnd(double d);

public:
	static void update(
	    int cx, int cy, int ri, int ro,
		bool interpolation, bool obilinear, bool obicubic,
		double sx, double sy,
		IplImage* inputImg,
		IplImage* outputImg);

	unwarp();
    ~unwarp();
};

