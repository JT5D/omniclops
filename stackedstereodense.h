/*
    Stacked omnidirectional dense stereo correspondence
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

#ifndef STACKEDSTEREODENSE_H_
#define STACKEDSTEREODENSE_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <fstream>
#include <vector>
#include "stereodense.h"

using namespace std;

class stackedstereodense {
public:

	static void unwarped_to_stereo_images(
		unsigned char* img_unwarped,
		int img_width_unwarped,
		int img_height_unwarped,
		int offset_y_unwarped,
		unsigned char* img_left,
		unsigned char* img_right,
		int img_width,
		int img_height,
		int lower_mirror_height,
		int upper_mirror_height);

	static void show(
		unsigned char* img_unwarped,
		int img_width_unwarped,
		int img_height_unwarped,
		int img_width,
		int img_height,
		int vertical_sampling,
		int smoothing_radius,
		int max_disparity_percent,
		unsigned int *disparity_map);

	static void show_stereo_images(
		unsigned char* img_unwarped,
		int img_width_unwarped,
		int img_height_unwarped,
		int offset_y_unwarped,
		unsigned char* img_left,
		unsigned char* img_right,
		int img_width,
		int img_height,
	    int lower_mirror_height,
	    int upper_mirror_height);

	static void update_disparity_map(
		unsigned char* img_unwarped,
		int img_width_unwarped,
		int img_height_unwarped,
		int lower_mirror_height,
		int upper_mirror_height,
		int offset_y_unwarped,
		unsigned char* img_left,
		unsigned char* img_right,
		int img_width,
		int img_height,
		int offset_x,
		int offset_y,
		int vertical_sampling,
		int max_disparity_percent,
		int correlation_radius,
		int smoothing_radius,
		int disparity_step,
		int disparity_threshold_percent,
		bool despeckle,
		int cross_checking_threshold,
		unsigned int *disparity_space,
		unsigned int *disparity_map);

};

#endif
