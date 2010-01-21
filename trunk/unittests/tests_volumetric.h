#ifndef unit_tests_volumetric_h_
#define unit_tests_volumetric_h_

#include <string>
#include <vector>
#include <assert.h>
#include <stdio.h>

#include "../drawing.h"
#include "../bitmap.h"
#include "../omni.h"

#include "../cppunitlite/TestHarness.h"

/*
TEST (DetectGroundPlane, MyTest)
{
	  float mirror_diameter = 60;
	  float dist_to_mirror_backing = 150;
	  float focal_length = 3.6;
	  float outer_radius = 37;
	  float camera_height = 1000;
	  int no_of_mirrors = 5;
	  int ww = 640;
	  int hh = 480;

	  float mirror_position[5*2];
	  float mirror_position_pixels[5*2];
	  unsigned char* ground_img = new unsigned char[ww*hh*3];
	  unsigned char* projected_img = new unsigned char[ww*hh*3];
	  int* ground_height_mm = new int[ww*hh];
	  unsigned char* reprojected_img = new unsigned char[ww*hh*3];
 	  Bitmap* bmp;

 	  memset((void*)ground_height_mm,'\0',ww*hh*sizeof(int));
 	  memset((void*)projected_img,'\0',ww*hh*3);

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
	  	0,
	  	no_of_mirrors,
	  	mirror_position,
	  	mirror_position_pixels,
	    ww,hh);

	// create a ground image
	memset((void*)ground_img,'\0',ww*hh*3);

	// reproject the ground image into the camera image
	int ground_radius = 150;
  	int tx_mm = -ground_radius;
  	int ty_mm = -ground_radius;
  	int bx_mm = ground_radius;
  	int by_mm = ground_radius;

    omni::create_ground_grid(
    	ground_height_mm,
    	ground_img,
    	ww, hh,
    	tx_mm, ty_mm,
    	bx_mm, by_mm,
    	tx_mm, ty_mm,
    	bx_mm, by_mm,
    	25,
    	0,
    	5,
    	255,0,0);

    omni::create_obstacle(
    	ground_height_mm,
    	ground_img,
    	ww, hh,
    	tx_mm, ty_mm,
    	bx_mm, by_mm,
    	-50, -50, -50, 50,
    	300, 5,
    	0,255,0);

    omni::reproject(
	  	ground_img,
	  	ground_height_mm,
	  	dist_to_mirror_backing,
		camera_height,
	  	ww,hh,
	  	tx_mm, ty_mm,
	  	bx_mm, by_mm,
	  	lcam.ray_map,
	  	reprojected_img,
	  	ww,hh);

	bmp = new Bitmap(ground_img, ww, hh, 3);
	std::string filename = "test_VolumetricReprojectGround.ppm";
	bmp->SavePPM(filename.c_str());
	delete bmp;

	// find the position of the red dot with respect to the centre of each mirror
	int radius_pixels = (outer_radius * ww / 200) - 10;
	int radius_pixels_sqr = radius_pixels*radius_pixels;
	int dot_position[no_of_mirrors*2];
	for (int m = 0; m < no_of_mirrors; m++) {
		int cx = mirror_position_pixels[m*2] * ww / 100;
		int cy = mirror_position_pixels[m*2+1] * hh / 100;
        for (int y = cy - radius_pixels; y <= cy + radius_pixels; y++) {
        	if ((y > 0) && (y < hh)) {
				int dy = y - cy;
				for (int x = cx - radius_pixels; x <= cx + radius_pixels; x++) {
					if ((x > 0) && (x < ww)) {
						int dx = x - cx;
						int dist = dx*dx + dy*dy;
						if (dist < radius_pixels_sqr) {
							int n = ((y * ww) + x) * 3;
                            if (reprojected_img[n+2] == 255) {
    							dot_position[m*2] = dx;
    							dot_position[m*2+1] = dy;
                            }
						}
					}
				}
        	}
        }
	}

	bmp = new Bitmap(reprojected_img, ww, hh, 3);
	filename = "test_VolumetricReproject.ppm";
	bmp->SavePPM(filename.c_str());
	delete bmp;

	omni::project(
		reprojected_img,
		0,
	  	dist_to_mirror_backing,
		camera_height,
	  	ww,hh,
		tx_mm, ty_mm,
		bx_mm, by_mm,
		lcam.ray_map,
		projected_img,
		ww,hh);

	bmp = new Bitmap(projected_img, ww, hh, 3);
	filename = "test_VolumetricProject.ppm";
	bmp->SavePPM(filename.c_str());
	delete bmp;

	int projection_hits = 0;
	int projection_misses = 0;
	for (int i = ww*hh*3-3; i >= 0; i-=3) {
		if ((projected_img[i+1] > 0) ||
			(projected_img[i+2] > 0)) {
			if ((ground_img[i+1] == projected_img[i+1]) ||
				(ground_img[i+2] == projected_img[i+2])) {
				projection_hits++;
			}
			else {
				projection_misses++;
			}
		}
	}

	delete[] ground_img;
	delete[] ground_height_mm;
	delete[] reprojected_img;
	delete[] projected_img;

	for (int m = 0; m < no_of_mirrors; m++) {
	    CHECK(dot_position[m*2] < 0);
	    CHECK(dot_position[m*2 + 1] < 0);
	}

	CHECK(projection_hits > 2000);
	CHECK(projection_hits > projection_misses);
}
*/

#endif
