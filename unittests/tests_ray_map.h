#ifndef unit_tests_ray_map_h_
#define unit_tests_ray_map_h_

#include <string>
#include <vector>
#include <assert.h>
#include <stdio.h>

#include "../bitmap.h"
#include "../omni.h"

#include "../cppunitlite/TestHarness.h"

TEST (RayMap5Mirror, MyTest)
{
	  int radius_mm = 300;

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
	  unsigned char* l_ = new unsigned char[ww*hh*3];
	  memset((void*)l_, '\0', ww*hh*3);
 	  Bitmap* bmp;

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

    //CHECK(grid_dim_x == 20);
    //CHECK_GREATER_THAN(95, match_percent);

	int mirror_index = 4;
	float min_radius = 9999;
	float max_radius = 0;
	for (int angle = 0; angle < 360; angle++) {
		int px = (mirror_position_pixels[mirror_index*2] * ww / 100) +
		        (int)((outer_radius * ww / 600) * sin(angle * 3.1415927f / 180));
		int py = (mirror_position_pixels[mirror_index*2+1] * hh / 100) +
		        (int)((outer_radius * ww / 600) * cos(angle * 3.1415927f / 180));

		if ((px > -1) && (px < ww) &&
			(py > -1) && (py < hh)) {
		    int n = (py * ww) + px;

		    if (lcam.mirror_map[n]-1 == mirror_index) {
				float dx = lcam.ray_map[n*6+3] - mirror_position[mirror_index*2];
				float dy = lcam.ray_map[n*6+4] - mirror_position[mirror_index*2+1];
				float radius = (float)sqrt(dx*dx + dy*dy);
				if (radius > max_radius) max_radius = radius;
				if (radius < min_radius) min_radius = radius;
				//printf("radius = %f\n", radius);
				//printf("radius = %d %d\n", lcam.ray_map[n*6+3], lcam.ray_map[n*6+4]);

				int screen_x = (lcam.ray_map[n*6+3] + radius_mm) * hh / (radius_mm*2);
				int screen_y = (lcam.ray_map[n*6+4] + radius_mm) * hh / (radius_mm*2);
				if ((screen_x > 0) && (screen_x < ww-1) &&
					(screen_y > 0) && (screen_y < hh-1)) {
					int n2 = ((screen_y * ww) + screen_x)*3;
                    l_[n2] = 0;
                    l_[n2+1] = 255;
                    l_[n2+2] = 0;

                    n2 = ((py * ww) + px)*3;
                    l_[n2] = 0;
                    l_[n2+1] = 0;
                    l_[n2+2] = 255;
				}
		    }
		}
	}

	bmp = new Bitmap(l_, ww, hh, 3);
	std::string filename = "test_RayMap5MirrorRadius.ppm";
	bmp->SavePPM(filename.c_str());
	delete bmp;

	omni::show_rays(
	    lcam.mirror_map,
	    lcam.ray_map,
	    l_,ww,hh,
	    radius_mm,1);

	filename = "test_RayMap5Mirror.ppm";
	bmp = new Bitmap(l_, ww, hh, 3);
	bmp->SavePPM(filename.c_str());

	delete[] l_;
	delete bmp;

	CHECK_GREATER_THAN(84, min_radius);
	CHECK_LESS_THAN(93, max_radius);
}

#endif
