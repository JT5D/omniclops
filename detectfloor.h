/*
    used to detect the floor in an omnidirectional image
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

#ifndef DETECTFLOOR_H_
#define DETECTFLOOR_H_

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

class detectfloor {
public:
	/*
    static void detect_from_features(
    	vector<int> &features,
    	int mirror_centre_x_pixels,
    	int mirror_centre_y_pixels,
    	int mirror_index,
    	int ray_map_width,
    	int ray_map_height,
    	unsigned char* mirror_map,
    	int angular_increment_degrees,
    	int camera_base_width_pixels,
    	int camera_base_height_pixels,
        vector<int> &floor_features);
    */

	static void detect(
		vector<int> &features,
		int no_of_mirrors,
		int ray_map_width,
		int ray_map_height,
		int floor_height_mm,
		int plane_tollerance_mm,
		float focal_length_mm,
		int camera_to_backing_dist_mm,
		int camera_height_mm,
		unsigned char* img,
		int* ray_map,
		unsigned char* mirror_map,
		unsigned short* feature_map,
		unsigned short *ground_features_lookup,
	    int ground_plane_tollerance_mm,
	    int image_plane_tollerance_pixels,
	    int max_range_mm,
	    int camera_tx,
	    int camera_ty,
	    int camera_bx,
	    int camera_by,
	    vector<int> &floor_features_positions,
	    vector<int> &floor_features);

	static void get_closest_features(
    	vector<int> &features,
    	int mirror_centre_x_pixels,
    	int mirror_centre_y_pixels,
    	int mirror_index,
    	int ray_map_width,
    	int ray_map_height,
    	unsigned char* img,
    	unsigned char* mirror_map,
    	int angular_increment_degrees,
    	int camera_base_width_pixels,
    	int camera_base_height_pixels,
        vector<int> &closest_features,
        vector<vector<unsigned char> > &floor_colour);

};

#endif /* DETECTFLOOR_H_ */
