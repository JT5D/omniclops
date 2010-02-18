/*
    used to detect vertical features, such as walls or doorways
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

#ifndef DETECTVERTICALS_H_
#define DETECTVERTICALS_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <fstream>
#include <algorithm>
#include "drawing.h"
#include "omni.h"
using namespace std;

class detectverticals {
protected:
	static void get_vertical_features(
		vector<int> &features,
		vector<int> &floor_features,
		int no_of_mirrors,
		int ray_map_width,
		int ray_map_height,
		unsigned char* mirror_map,
	    vector<int> &vertical_features);

	static void get_vertical_features_centre(
		vector<int> &features,
		float* mirror_position_pixels,
		int min_radius_pixels,
		int max_radius_pixels,
		int no_of_mirrors,
		int ray_map_width,
		int ray_map_height,
		unsigned char* mirror_map,
	    vector<int> &possible_verticals);

	static void get_possible_verticals(
		vector<int> &features,
		float* mirror_centre_pixels,
		int min_radius_pixels,
		int max_radius_pixels,
		int no_of_mirrors,
		int ray_map_width,
		int ray_map_height,
		unsigned char* mirror_map,
	    vector<int> &possible_verticals);

public:
	static void show_point_cloud(
		unsigned char* img,
		int width,
		int height,
		vector<int> &point_cloud,
		int max_range_mm,
		int max_height_mm,
		int view_type);

	static void get_point_cloud(
		vector<int> &features,
		vector<int> &floor_features,
		float* mirror_position_pixels,
		float outer_radius_percent,
		int min_radius_percent,
		int max_radius_percent,
		int no_of_mirrors,
		int ray_map_width,
		int ray_map_height,
		int* ray_map,
		unsigned char* mirror_map,
		int min_range_mm,
		int max_range_mm,
		int max_height_mm,
		int max_vertical_separation_per_metre,
		int max_intersection_samples,
		vector<int> &vertical_features,
	    vector<int> &points);
};

#endif /* DETECTVERTICALS_H_ */
