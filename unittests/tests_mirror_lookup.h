#ifndef unit_tests_mirror_lookup_h_
#define unit_tests_mirror_lookup_h_

#include <string>
#include <vector>
#include <assert.h>
#include <stdio.h>
#include <cv.h>
#include <highgui.h>

#include "../drawing.h"
#include "../bitmap.h"
#include "../omni.h"

#include "../cppunitlite/TestHarness.h"

TEST (Mirror5Lookup, MyTest)
{
	  int radius_mm = 600;

	  float mirror_diameter = 60;
	  float dist_to_mirror_backing = 150;
	  float focal_length = 3.6;
	  float outer_radius = 37;
	  float camera_height = 0;
	  int no_of_mirrors = 5;
	  int ww = 640;
	  int hh = 480;

	  float mirror_position[5*2];
	  float mirror_position_pixels[5*2];

 	  IplImage *output_image=cvCreateImage(cvSize(ww, hh), 8, 3);
 	  unsigned char *output_image_=(unsigned char *)output_image->imageData;
	  memset((void*)output_image_, '\0', ww*hh*3);


	  mirror_position_pixels[0] = 19.00;
	  mirror_position_pixels[1] = 20.00;

	  mirror_position_pixels[2] = 20.00;
	  mirror_position_pixels[3] = 84.00;

	  mirror_position_pixels[4] = 85.00;
	  mirror_position_pixels[5] = 81.00;

	  mirror_position_pixels[6] = 84.00;
	  mirror_position_pixels[7] = 20.00;

	  mirror_position_pixels[8] = 51.00;
	  mirror_position_pixels[9] = 50.00;

	  mirror_position[0] = -60.00;
	  mirror_position[1] = -75.00;

	  mirror_position[2] = 60.00;
	  mirror_position[3] = -75.00;

	  mirror_position[4] = 60.00;
	  mirror_position[5] = 75.00;

	  mirror_position[6] = -50.00;
	  mirror_position[7] = 75.00;

	  mirror_position[8] = 0.00;
	  mirror_position[9] = 0.00;

	  omni lcam(ww, hh);
	  lcam.create_ray_map(
	  	mirror_diameter,
	  	dist_to_mirror_backing,
	  	focal_length,
	  	outer_radius,
	  	camera_height,
	  	no_of_mirrors,
	  	mirror_position,
	  	mirror_position_pixels,
	    ww,hh);

	  omni::show_mirror_lookup(output_image_, ww, hh, lcam.mirror_lookup, true, outer_radius);

	  std::string filename = "test_MirrorLookupRadius.png";
	  cvSaveImage(filename.c_str(), output_image);

	  omni::show_mirror_lookup(output_image_, ww, hh, lcam.mirror_lookup, false, outer_radius);

	  filename = "test_MirrorLookupRotation.png";
	  cvSaveImage(filename.c_str(), output_image);

	  //CHECK_GREATER_THAN(84, min_radius);
	  //CHECK_LESS_THAN(93, max_radius);

	  cvReleaseImage(&output_image);
}


#endif
