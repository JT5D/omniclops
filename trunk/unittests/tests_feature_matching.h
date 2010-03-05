#ifndef unit_tests_feature_matching_h_
#define unit_tests_feature_matching_h_

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


TEST (FeatureMatching, MyTest)
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

	int ray_map_height_mm = 0;

    omni lcam(ww, hh);
	lcam.create_ray_map(
	  	mirror_diameter,
	  	dist_to_mirror_backing,
	  	focal_length,
	  	outer_radius,
	  	ray_map_height_mm,
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
	  	ray_map_height_mm,
	  	tx_mm, ty_mm,
	  	bx_mm, by_mm,
	  	lcam.ray_map,
	  	reprojected_img_,
	  	ww,hh);

	std::string filename = "test_FeatureMatchingReprojectGround.png";
	cvSaveImage(filename.c_str(), ground_img);

	filename = "test_FeatureMatchingReproject.png";
	cvSaveImage(filename.c_str(), reprojected_img);

	printf("Feature Matching\n");

	vector<int> features;
	vector<vector<int> > features_on_plane;
	for (int i = 0; i < no_of_mirrors; i++) {
		vector<int> v;
		features_on_plane.push_back(v);
	}

	int plane_height_mm = 0;
	float pixel_diameter_mm = mirror_diameter / (float)(outer_radius * ww / 200);
	pixel_diameter_mm *= 10;
	//if (pixel_diameter_mm < 5) pixel_diameter_mm = 5;

	vector<int> voxels;
	vector<int> projected_points;
	vector<int> matched_features;
	int minimum_matching_score_percent = 0;
	int ray_radius_mm = 0;

	for (int y = 0; y < hh; y++) {
		for (int x = 0; x < ww; x++) {
			int n = (y*ww + x)*3;
			if ((reprojected_img_[n] > 0) ||
				(reprojected_img_[n+1] > 0) ||
				(reprojected_img_[n+2] > 0)) {
				features.push_back(x);
				features.push_back(y);
			}
		}
	}

    omni::match_features_on_plane(
    	features,
    	features_on_plane,
    	no_of_mirrors,
    	reprojected_img_,
    	plane_height_mm,
    	focal_length,
    	(int)dist_to_mirror_backing,
    	(int)camera_height,
    	ww,hh,
    	ray_map_height_mm,
    	lcam.ray_map,
    	lcam.mirror_map,
    	-1,
    	pixel_diameter_mm,
    	voxels,
    	projected_points,
    	matched_features,
    	minimum_matching_score_percent,
    	ray_radius_mm,
    	false);

	filename = "test_FeatureMatchingProject.png";
	cvSaveImage(filename.c_str(), projected_img);

	omni::show_voxels(
		reprojected_img_,
		tx_mm, ty_mm, bx_mm, by_mm,
		projected_points,
		ww,hh,
		projected_img_);

	filename = "test_FeatureMatchingProjectedPoints.png";
	cvSaveImage(filename.c_str(), projected_img);

	omni::show_voxels(
		reprojected_img_,
		tx_mm, ty_mm, bx_mm, by_mm,
		voxels,
		ww,hh,
		projected_img_);

	filename = "test_FeatureMatchingVoxels.png";
	cvSaveImage(filename.c_str(), projected_img);

	printf("voxels %d\n",(int)voxels.size()/4);

	CHECK((int)voxels.size()/4 > 0);

	cvReleaseImage(&ground_img);
	cvReleaseImage(&projected_img);
	cvReleaseImage(&reprojected_img);

}

TEST (FeatureMatchingVoxels, MyTest)
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

	int ray_map_height_mm = 0;

    omni lcam(ww, hh);
	lcam.create_ray_map(
	  	mirror_diameter,
	  	dist_to_mirror_backing,
	  	focal_length,
	  	outer_radius,
	  	ray_map_height_mm,
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

  	int ground_height_mm2 = 200;
    omni::reproject(
	  	ground_img_,
	  	ground_height_mm2,
	  	focal_length,
	  	dist_to_mirror_backing,
		camera_height,
	  	ww,hh,
	  	ray_map_height_mm,
	  	tx_mm, ty_mm,
	  	bx_mm, by_mm,
	  	lcam.ray_map,
	  	reprojected_img_,
	  	ww,hh);

	printf("Feature Matching\n");

	vector<int> features;
	vector<vector<int> > features_on_plane;
	for (int i = 0; i < no_of_mirrors; i++) {
		vector<int> v;
		features_on_plane.push_back(v);
	}

	int plane_height_mm = 0;
	float pixel_diameter_mm = mirror_diameter / (float)(outer_radius * ww / 200);
	pixel_diameter_mm *= 10;

	vector<short> voxels;
	vector<int> projected_points;
	vector<int> matched_features;
	int minimum_matching_score_percent = 0;
	int ray_radius_mm = 0;

	for (int y = 0; y < hh; y++) {
		for (int x = 0; x < ww; x++) {
			int n = (y*ww + x)*3;
			if ((reprojected_img_[n] > 0) ||
				(reprojected_img_[n+1] > 0) ||
				(reprojected_img_[n+2] > 0)) {
				features.push_back(x);
				features.push_back(y);
			}
		}
	}

	int start_plane_height_mm = 0;
	int end_plane_height_mm = 200;
	int no_of_planes = 10;
	omni::voxels_from_features(
		features,
		reprojected_img_,
		ww,hh,
		ray_map_height_mm,
		start_plane_height_mm,
		end_plane_height_mm,
		no_of_planes,
		focal_length,
		dist_to_mirror_backing,
		(int)camera_height,
		lcam.ray_map,
		no_of_mirrors,
		lcam.mirror_map,
		outer_radius,
		(int)mirror_diameter,
		voxels);

	int view_type = 3;
	omni::show_feature_voxels(
		projected_img_,
    	ww,hh,
    	voxels,
    	view_type);

	std::string filename = "test_FeatureMatchingVoxels.png";
	cvSaveImage(filename.c_str(), projected_img);

	//printf("voxels: %d\n", (int)voxels.size()/4);
	CHECK((int)voxels.size()/4 > 0);

	cvReleaseImage(&ground_img);
	cvReleaseImage(&projected_img);
	cvReleaseImage(&reprojected_img);

}


#endif
