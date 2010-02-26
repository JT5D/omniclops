/*
    used to create a point cloud for observed features
    Copyright (C) 2010 Bob Mottram
    fuzzgun@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef POINTCLOUD_H_
#define POINTCLOUD_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <fstream>
#include <algorithm>
#include "drawing.h"
#include "omni.h"
#include "detectfloor.h"
//#include "grouping.h"
using namespace std;

class pointcloud {
public:
/*
	static void features_to_point_cloud(
		vector<int> &features,
		int mirror_index,
		float focal_length_mm,
		int camera_to_backing_dist_mm,
		int camera_height_mm,
		int ray_map_width,
		int ray_map_height,
		int* ray_map,
		unsigned char* mirror_map,
		int max_range_mm,
	    vector<int> &point_cloud);
*/

	static void cloud_to_perimeter(
		vector<int> &point_cloud,
		vector<int> &perimeter);

	static void get_feature_heights(
		unsigned char* img,
		vector<int> &features,
		float camera_height_mm,
		float camera_to_mirror_backing_dist_mm,
		float focal_length,
		float* mirror_position_pixels,
		float outer_radius_percent,
		float mirror_diameter_mm,
		int no_of_mirrors,
		int ray_map_width,
		int ray_map_height,
		int max_height_mm,
		int height_step_mm,
		int max_range_mm,
		int camera_width_percent,
		int camera_height_percent,
		int* ray_map,
		unsigned char* mirror_map,
		unsigned short* feature_map,
		unsigned short* ground_features_lookup,
		bool show_features,
		vector<int> &feature_heights,
		vector<int> &point_cloud);

	static void show(
		unsigned char *img,
		int width,
		int height,
		int max_range_mm,
		int max_height_mm,
		vector<int> &point_cloud,
		int view_type);

	static void show_perimeter(
		unsigned char *img,
		int width,
		int height,
		int max_range_mm,
		vector<int> &perimeter,
		int r, int g, int b,
		int view_type);

	static void update(
		unsigned char* img,
	    vector<int> &features,
	    float camera_height_mm,
	    float camera_to_mirror_backing_dist_mm,
	    float focal_length_mm,
	    float* mirror_position_pixels,
	    float outer_radius_percent,
	    float mirror_diameter_mm,
	    int no_of_mirrors,
	    int ray_map_width,
	    int ray_map_height,
	    int max_height_mm,
	    int height_step_mm,
	    int max_range_mm,
	    int camera_width_percent,
	    int camera_height_percent,
	    int* ray_map,
	    unsigned char* mirror_map,
	    unsigned short* feature_map,
	    unsigned short* ground_features_lookup,
	    int view_type,
	    vector<int> &point_cloud,
	    vector<int> &perimeter);

	static void save(
		std::string filename,
		vector<int> &point_cloud);

};

#endif /* POINTCLOUD_H_ */
