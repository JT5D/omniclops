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

TEST (ReconstructVolume, MyTest)
{
	  float mirror_diameter = 60;
	  float dist_to_mirror_backing = 150;
	  float focal_length = 3.6;
	  float outer_radius = 37;
	  float camera_height = 200;
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

  	int ground_height_mm2 = 100;
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

	std::string filename = "test_RayMap5MirrorReprojectGround2.png";
	cvSaveImage(filename.c_str(), ground_img);

	filename = "test_RayMap5MirrorReproject2.png";
	cvSaveImage(filename.c_str(), reprojected_img);

	printf("Projection\n");

	omni::project(
		reprojected_img_,
		ground_height_mm2,
		focal_length,
	  	dist_to_mirror_backing,
		camera_height,
	  	ww,hh,
		tx_mm, ty_mm,
		bx_mm, by_mm,
		lcam.ray_map,
    	lcam.mirror_map,
    	lcam.mirror_lookup,
    	-1,
    	0,
    	(int)(mirror_diameter/2),
		projected_img_,
		ww,hh,
		colour_difference);

	filename = "test_RayMap5MirrorProject3.png";
	cvSaveImage(filename.c_str(), projected_img);


	printf("Volume\n");

	int start_plane_height_mm = 200;
	int end_plane_height_mm = 0;
	int no_of_planes = 10;

	int patch_size_pixels = 5;
	int min_patch_observations = 3;
	unsigned char* height_field_img = new unsigned char[ww*hh*3];
	short* height_field = new short[ww*hh];

	omni::reconstruct_volume(
		reprojected_img_,
		start_plane_height_mm,
		end_plane_height_mm,
		no_of_planes,
		focal_length,
		dist_to_mirror_backing,
		camera_height,
		ww, hh,
		tx_mm, ty_mm,
		bx_mm, by_mm,
		lcam.ray_map,
		lcam.mirror_map,
		lcam.mirror_lookup,
		projected_img_,
		ww, hh,
		colour_difference,
		height_field,
		height_field_img,
		patch_size_pixels,
		min_patch_observations);

	filename = "test_VolumeOverhead.png";
	memcpy((void*)projected_img_, (void*)height_field_img,ww*hh*3);
	cvSaveImage(filename.c_str(), projected_img);

    omni::show_height_field(
    	projected_img_,
    	ww,hh,
    	(int)dist_to_mirror_backing,
    	height_field,
    	height_field_img,
    	ww,hh,3);

	filename = "test_HeightField.png";
	cvSaveImage(filename.c_str(), projected_img);

	int max = 0;
	int average_height = 0;
	int hits = 0;
	for (int i = ww*hh-1; i >= 0; i--) {
		if (height_field[i] > max) max = (int)height_field[i];
		if (height_field[i] > 0) {
			average_height += (int)height_field[i];
			hits++;
		}
	}
	if (hits > 0) {
		average_height /= hits;
	}
	CHECK(max > 0);
	//printf("average height %d\n", average_height);

	cvReleaseImage(&ground_img);
	cvReleaseImage(&projected_img);
	cvReleaseImage(&reprojected_img);
	delete[] colour_difference;
	delete[] height_field_img;
	delete[] height_field;
}


TEST (PhotometricConsistency, MyTest)
{
	  float mirror_diameter = 60;
	  float dist_to_mirror_backing = 150;
	  float focal_length = 3.6;
	  float outer_radius = 37;
	  float camera_height = 200;
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

  	int ground_height_mm2 = 50;
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

	printf("Projection\n");

	int start_plane_height_mm = 0;
	int end_plane_height_mm = 200;
	int no_of_planes = 10;
	int max_consistency = 0;
	int best_plane_height_mm = 0;

    for (int plane = no_of_planes-1; plane >= 0; plane--) {

    	int plane_height_mm = start_plane_height_mm + (plane * (end_plane_height_mm - start_plane_height_mm) / no_of_planes);

		omni::project(
			reprojected_img_,
			plane_height_mm,
			focal_length,
			dist_to_mirror_backing,
			camera_height,
			ww,hh,
			tx_mm, ty_mm,
			bx_mm, by_mm,
			lcam.ray_map,
			lcam.mirror_map,
			lcam.mirror_lookup,
			-1,
			0,
			(int)(mirror_diameter/2),
			projected_img_,
			ww,hh,
			colour_difference);

		char file_name[30];
		sprintf(file_name, "test_consistency_%d.png",plane_height_mm);
		cvSaveImage(file_name, projected_img);

		int tot_colour_consistency = 0;
		int tot_observations = 1;
		for (int i = ww*hh-1; i >= 0; i--) {
			tot_colour_consistency += colour_difference[i*2];
			tot_observations += colour_difference[i*2 + 1];
		}
		tot_colour_consistency = 10000 / (1 + (tot_colour_consistency/tot_observations));
		if (tot_colour_consistency > max_consistency) {
			max_consistency = tot_colour_consistency;
			best_plane_height_mm = plane_height_mm;
		}

    }

    CHECK(abs(best_plane_height_mm - ground_height_mm2) < 20);

	cvReleaseImage(&ground_img);
	cvReleaseImage(&projected_img);
	cvReleaseImage(&reprojected_img);
	delete[] colour_difference;
}


#endif
