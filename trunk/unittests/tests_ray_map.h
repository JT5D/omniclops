#ifndef unit_tests_ray_map_h_
#define unit_tests_ray_map_h_

#include <string>
#include <vector>
#include <assert.h>
#include <stdio.h>

#include "../drawing.h"
#include "../bitmap.h"
#include "../omni.h"

#include "../cppunitlite/TestHarness.h"

TEST (RayMap5Mirror, MyTest)
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

	memset((void*)l_, '\0', ww*hh*3);
	int radius_points[ww];
	memset((void*)radius_points,'\0',ww*sizeof(int));
	int n3 = 0;
	for (int y = 0; y < hh; y++) {
		for (int x = 0; x < ww; x++, n3++) {
			if (lcam.mirror_map[n3] > 0) {
				int radius = (int)sqrt(lcam.ray_map[n3*6]*lcam.ray_map[n3*6] + lcam.ray_map[n3*6+1]*lcam.ray_map[n3*6+1])/8;

				int dx = x - (ww/2);
				int dy = y - (hh/2);
				int radius2 = (int)sqrt(dx*dx + dy*dy);
				if (radius_points[radius] == 0) {
				    radius_points[radius] = radius2;
				}
				else {
					radius_points[radius] = (radius_points[radius] + radius2)/2;
				}

				int r=0,g=0,b=0;
				switch(radius % 6)
				{
				case 0: {
					r = 255;
					g = 0;
					b = 0;
					break;
				}
				case 1: {
					r = 0;
					g = 255;
					b = 0;
					break;
				}
				case 2: {
					r = 0;
					g = 0;
					b = 255;
					break;
				}
				case 3: {
					r = 255;
					g = 0;
					b = 255;
					break;
				}
				case 4: {
					r = 0;
					g = 255;
					b = 255;
					break;
				}
				case 5: {
					r = 255;
					g = 255;
					b = 255;
					break;
				}
				}

                //int x_intensity = abs(lcam.ray_map[n3*6]) * 255*2 / radius_mm;
                //if (x_intensity > 255) x_intensity = 255;
                //int y_intensity = abs(lcam.ray_map[n3*6+1]) * 255*2 / radius_mm;
                //if (y_intensity > 255) y_intensity = 255;
                l_[n3*3+2] = (unsigned char)r;
                l_[n3*3+1] = (unsigned char)g;
                l_[n3*3] = (unsigned char)b;
			}
		}
	}
	bmp = new Bitmap(l_, ww, hh, 3);
	filename = "test_RayMap5MirrorDist.ppm";
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

	int prev_r = -1;
	for (int r = 1; r < ww; r++) {
		if (radius_points[r] > 0) {
			if (prev_r > -1) {
                CHECK(prev_r <= radius_points[r])
			}
			prev_r = radius_points[r];
		}
	}

}

TEST (RayMap5MirrorReproject, MyTest)
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
	  unsigned char* ground_img = new unsigned char[ww*hh*3];
	  unsigned char* reprojected_img = new unsigned char[ww*hh*3];
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

	// create a ground image
	memset((void*)ground_img,'\0',ww*hh*3);
    drawing::drawBox(ground_img,ww,hh,ww/2,hh/2,ww*45/100,ww*95/100,0,0,255,0,2);
    drawing::drawSpot(ground_img,ww,hh,(ww/2)-(ww*46/100),(hh/2)-(ww*20/100),30,255,0,0);

	// reproject the ground image into the camera image
	int ground_radius = 150;
  	int tx_mm = -ground_radius;
  	int ty_mm = -ground_radius;
  	int bx_mm = ground_radius;
  	int by_mm = ground_radius;

    omni::reproject(
	  	ground_img,
	  	ww,hh,
	  	tx_mm, ty_mm,
	  	bx_mm, by_mm,
	  	lcam.ray_map,
	  	reprojected_img,
	  	ww,hh);

	bmp = new Bitmap(ground_img, ww, hh, 3);
	std::string filename = "test_RayMap5MirrorReprojectGround.ppm";
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
	filename = "test_RayMap5MirrorReproject.ppm";
	bmp->SavePPM(filename.c_str());
	delete bmp;

	delete[] ground_img;
	delete[] reprojected_img;

	for (int m = 0; m < no_of_mirrors; m++) {
	    CHECK(dot_position[m*2] < 0);
	    CHECK(dot_position[m*2 + 1] < 0);
	}

	//CHECK_GREATER_THAN(84, min_radius);
	//CHECK_LESS_THAN(93, max_radius);
}


#endif
