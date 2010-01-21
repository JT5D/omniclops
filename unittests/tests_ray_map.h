#ifndef unit_tests_ray_map_h_
#define unit_tests_ray_map_h_

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

	printf("Mirror\n");

	int mirror_index = 4;
	float min_radius = 9999;
	float max_radius = 0;
	int sample_radius = (outer_radius * ww / 200) * 90/100;
	for (int angle = 0; angle < 360; angle++) {
		int px = (mirror_position_pixels[mirror_index*2] * ww / 100) +
		        (int)(sample_radius * sin(angle * 3.1415927f / 180));
		int py = (mirror_position_pixels[mirror_index*2+1] * hh / 100) +
		        (int)(sample_radius * cos(angle * 3.1415927f / 180));

		if ((px > -1) && (px < ww) &&
			(py > -1) && (py < hh)) {
		    int n = (py * ww) + px;

		    if (lcam.mirror_map[n]-1 == mirror_index) {
				float dx = lcam.ray_map[n*6] - mirror_position[mirror_index*2];
				float dy = lcam.ray_map[n*6+1] - mirror_position[mirror_index*2+1];
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
					output_image_[n2] = 0;
					output_image_[n2+1] = 255;
					output_image_[n2+2] = 0;

                    n2 = ((py * ww) + px)*3;
                    output_image_[n2] = 0;
                    output_image_[n2+1] = 0;
                    output_image_[n2+2] = 255;
				}
		    }
		}
	}


	std::string filename = "test_RayMap5MirrorRadius.png";
	cvSaveImage(filename.c_str(), output_image);

	int n3 = 0;

	/*
	for (int y = 0; y < hh; y++) {
		for (int x = 0; x < ww; x++, n3++) {
			if (lcam.mirror_map[n3] > 0) {
				int dx = x - (ww/2);
				int dy = y - (hh/2);

				float angle0 = (float)atan2(lcam.ray_map[n3*6], lcam.ray_map[n3*6 + 1]) / 3.1415927f * 180;
				float angle1 = (float)atan2(lcam.ray_map[n3*6 + 3], lcam.ray_map[n3*6 + 4]) / 3.1415927f * 180;
				printf("angle %f %f\n", angle0, angle1);
			}
		}
	}
	*/


	memset((void*)output_image_, '\0', ww*hh*3);
	int radius_points[ww];
	memset((void*)radius_points,'\0',ww*sizeof(int));
	n3 = 0;
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
                output_image_[n3*3+2] = (unsigned char)r;
                output_image_[n3*3+1] = (unsigned char)g;
                output_image_[n3*3] = (unsigned char)b;
			}
		}
	}

	filename = "test_RayMap5MirrorDist.png";
	cvSaveImage(filename.c_str(), output_image);

	printf("Rays\n");

	omni::show_rays(
	    lcam.mirror_map,
	    lcam.ray_map,
	    output_image_,ww,hh,
	    radius_mm,1);

	filename = "test_RayMap5Mirror.png";
	cvSaveImage(filename.c_str(), output_image);

	CHECK_GREATER_THAN(20, min_radius);
	CHECK_LESS_THAN(30, max_radius);

	int prev_r = -1;
	for (int r = 1; r < ww; r++) {
		if (radius_points[r] > 0) {
			if (prev_r > -1) {
                CHECK(prev_r <= radius_points[r])
			}
			prev_r = radius_points[r];
		}
	}

	cvReleaseImage(&output_image);
}

TEST (RayMap5MirrorReproject, MyTest)
{
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

	  int* colour_difference = new int[ww*hh*2];

 	  IplImage *ground_img=cvCreateImage(cvSize(ww, hh), 8, 3);
 	  unsigned char *ground_img_=(unsigned char *)ground_img->imageData;
	  memset((void*)ground_img_, '\0', ww*hh*3);

	  IplImage *projected_img=cvCreateImage(cvSize(ww, hh), 8, 3);
 	  unsigned char *projected_img_=(unsigned char *)projected_img->imageData;
	  memset((void*)projected_img_, '\0', ww*hh*3);

	  IplImage *reprojected_img=cvCreateImage(cvSize(ww, hh), 8, 3);
 	  unsigned char *reprojected_img_=(unsigned char *)reprojected_img->imageData;
	  memset((void*)reprojected_img_, '\0', ww*hh*3);

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
    drawing::drawBox(ground_img_,ww,hh,ww/2,hh/2,ww*45/100,ww*95/100,0,0,255,0,2);
    drawing::drawSpot(ground_img_,ww,hh,(ww/2)-(ww*46/100),(hh/2)-(ww*20/100),30,255,0,0);

	// reproject the ground image into the camera image
	int ground_radius = 150;
  	int tx_mm = -ground_radius;
  	int ty_mm = -ground_radius;
  	int bx_mm = ground_radius;
  	int by_mm = ground_radius;

  	printf("Reprojection\n");

  	int ground_height_mm2 = 0;
    omni::reproject(
	  	ground_img_,
	  	ground_height_mm2,
	  	focal_length,
	  	dist_to_mirror_backing,
		camera_height,
	  	ww,hh,
	  	tx_mm, ty_mm,
	  	bx_mm, by_mm,
	  	lcam.ray_map,
	  	reprojected_img_,
	  	ww,hh);

	std::string filename = "test_RayMap5MirrorReprojectGround.png";
	cvSaveImage(filename.c_str(), ground_img);

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
                            if (reprojected_img_[n+2] == 255) {
    							dot_position[m*2] = dx;
    							dot_position[m*2+1] = dy;
                            }
						}
					}
				}
        	}
        }
	}

	filename = "test_RayMap5MirrorReproject.png";
	cvSaveImage(filename.c_str(), reprojected_img);

	printf("Projection\n");

	omni::project(
		reprojected_img_,
		0,
		focal_length,
	  	dist_to_mirror_backing,
		camera_height,
	  	ww,hh,
		tx_mm, ty_mm,
		bx_mm, by_mm,
		lcam.ray_map,
		projected_img_,
		ww,hh,
		colour_difference);

	filename = "test_RayMap5MirrorProject.png";
	cvSaveImage(filename.c_str(), projected_img);

	int projection_hits = 0;
	int projection_misses = 0;
	for (int i = ww*hh*3-3; i >= 0; i-=3) {
		if ((projected_img_[i+1] > 0) ||
			(projected_img_[i+2] > 0)) {
			if ((ground_img_[i+1] == projected_img_[i+1]) ||
				(ground_img_[i+2] == projected_img_[i+2])) {
				projection_hits++;
			}
			else {
				projection_misses++;
			}
		}
	}

	cvReleaseImage(&ground_img);
	cvReleaseImage(&projected_img);
	cvReleaseImage(&reprojected_img);
	delete[] colour_difference;

	for (int m = 0; m < no_of_mirrors; m++) {
	    //CHECK(dot_position[m*2] < 0);
	    //CHECK(dot_position[m*2 + 1] < 0);
	}

	CHECK(projection_hits > 1000);
	CHECK(projection_hits > projection_misses);
}


TEST (MirrorHeight, MyTest)
{
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

	  memset((void*)output_image_, '\0', ww*hh*3);
	  int max_height[no_of_mirrors*3];
	  memset((void*)max_height,'\0',no_of_mirrors*3*sizeof(int));
	  int n3 = 0;
	  int n = 0;
	  for (int y = 0; y < hh; y++) {
		  for (int x = 0; x < ww; x++, n3+=6, n++) {
			  if (lcam.mirror_map[n] > 0) {
				  int mirror = lcam.mirror_map[n]-1;
				  int height_mm = abs((int)dist_to_mirror_backing - lcam.ray_map[n3+2]);
				  int v = height_mm * 255 / (int)(mirror_diameter/2);
				  output_image_[n*3] = 255-v;
				  output_image_[n*3+1] = v;
				  output_image_[n*3+2] = 0;

				  if (height_mm > max_height[mirror*3+2]) {
					  max_height[mirror*3] = x;
					  max_height[mirror*3+1] = y;
					  max_height[mirror*3+2] = height_mm;
				  }
			  }
		}
	}

	for (int i = 0; i < no_of_mirrors; i++) {
		drawing::drawCross(output_image_,ww,hh,max_height[i*3],max_height[i*3+1],4,0,0,0,0);
	}

	string filename = "test_MirrorHeight.png";
	cvSaveImage(filename.c_str(), output_image);

	cvReleaseImage(&output_image);

	// check positions of the highest points
	int search_radius = 40;
	for (int i = 0; i < no_of_mirrors; i++) {
		int x = mirror_position_pixels[i*2]*ww/100;
		int y = mirror_position_pixels[i*2+1]*hh/100;
		CHECK(max_height[i*3] > x - search_radius);
		CHECK(max_height[i*3] < x + search_radius);
		CHECK(max_height[i*3+1] > y - search_radius);
		CHECK(max_height[i*3+1] < y + search_radius);
	}

	// check that the peaks are approximately the same height
	CHECK(abs(max_height[4*3+2] - max_height[0*3+2]) < 10);
	CHECK(abs(max_height[4*3+2] - max_height[1*3+2]) < 10);
	CHECK(abs(max_height[4*3+2] - max_height[2*3+2]) < 10);
	CHECK(abs(max_height[4*3+2] - max_height[3*3+2]) < 10);
}


TEST (SingleMirrorGroundPlane, MyTest)
{
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



	  int range_mm = 150;
	  int tx_mm = -range_mm;
	  int ty_mm = -range_mm * hh / ww;
	  int bx_mm = range_mm;
	  int by_mm = range_mm * hh / ww;

	  int n = 0;
	  int r=0,g=0,b=0;

	  int projection_positions[no_of_mirrors*8];
	  int projection_centre[4];
	  projection_centre[0] = ww;
	  projection_centre[1] = hh;
	  projection_centre[2] = 0;
	  projection_centre[3] = 0;

	  for (int m = 0; m < no_of_mirrors; m++) {
		  memset((void*)output_image_, '\0', ww*hh*3);

		  projection_positions[m*8] = ww;
		  projection_positions[m*8+1] = hh;
		  projection_positions[m*8+2] = 0;
		  projection_positions[m*8+3] = 0;

		  projection_positions[m*8+4] = ww;
		  projection_positions[m*8+5] = hh;
		  projection_positions[m*8+6] = 0;
		  projection_positions[m*8+7] = 0;

		  n = 0;
		  for (int y = 0; y < hh; y++) {
			  for (int x = 0; x < ww; x++, n++) {
				  int mirror = lcam.mirror_map[n] - 1;
				  if (mirror == m) {
					  switch(mirror) {
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
						  r = 255;
						  g = 255;
						  b = 0;
						  break;
					  }
					  }

					  if (lcam.mirror_lookup[n*2] < 2) {
						  int px = (lcam.ray_map[n*6+3] - tx_mm) * ww / (bx_mm - tx_mm);
						  int py = (lcam.ray_map[n*6+4] - ty_mm) * hh / (by_mm - ty_mm);
						  if (px < projection_centre[0]) projection_centre[0] = px;
						  if (py < projection_centre[1]) projection_centre[1] = py;
						  if (px > projection_centre[2]) projection_centre[2] = px;
						  if (py > projection_centre[3]) projection_centre[3] = py;
					  }

					  if ((lcam.mirror_lookup[n*2] > 10) &&
						  (lcam.mirror_lookup[n*2] < 14)) {
						  int px = (lcam.ray_map[n*6+3] - tx_mm) * ww / (bx_mm - tx_mm);
						  int py = (lcam.ray_map[n*6+4] - ty_mm) * hh / (by_mm - ty_mm);
						  if ((px > -1) && (px < ww) &&
							  (py > -1) && (py < hh)) {
							  int n2 = (py*ww + px) * 3;
							  output_image_[n2] = b;
							  output_image_[n2 + 1] = g;
							  output_image_[n2 + 2] = r;

							  if (px < projection_positions[m*8]) projection_positions[m*8] = px;
							  if (py < projection_positions[m*8+1]) projection_positions[m*8+1] = py;
							  if (px > projection_positions[m*8+2]) projection_positions[m*8+2] = px;
							  if (py > projection_positions[m*8+3]) projection_positions[m*8+3] = py;
						  }
						  px = (lcam.ray_map[n*6] - tx_mm) * ww / (bx_mm - tx_mm);
						  py = (lcam.ray_map[n*6+1] - ty_mm) * hh / (by_mm - ty_mm);
						  if ((px > -1) && (px < ww) &&
							  (py > -1) && (py < hh)) {
							  int n2 = (py*ww + px) * 3;
							  output_image_[n2] = b;
							  output_image_[n2 + 1] = g;
							  output_image_[n2 + 2] = r;

							  if (px < projection_positions[m*8+4]) projection_positions[m*8+4] = px;
							  if (py < projection_positions[m*8+5]) projection_positions[m*8+5] = py;
							  if (px > projection_positions[m*8+6]) projection_positions[m*8+6] = px;
							  if (py > projection_positions[m*8+7]) projection_positions[m*8+7] = py;
						  }
					  }
				  }
			  }
		  }
		  string filename = "test_SingleMirror";
		  switch(m) {
		  case 0: { filename += "0"; break; }
		  case 1: { filename += "1"; break; }
		  case 2: { filename += "2"; break; }
		  case 3: { filename += "3"; break; }
		  case 4: { filename += "4"; break; }
		  }
		  filename += ".png";
		  cvSaveImage(filename.c_str(), output_image);
	  }

	  cvReleaseImage(&output_image);

	  // check that all mirrors project to the same point (the focal plane of the camera)
	  //printf("centre %d %d %d %d\n", projection_centre[0],projection_centre[1],projection_centre[2],projection_centre[3]);
	  CHECK(projection_centre[0] > 300);
	  CHECK(projection_centre[1] > 220);
	  CHECK(projection_centre[2] < 340);
	  CHECK(projection_centre[3] < 260);
}


#endif
