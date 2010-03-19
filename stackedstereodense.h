/*
    Dense stereo correspondence for stacked omnidirectional vision
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
#include "drawing.h"
#include "omni.h"
using namespace std;

class stackedstereodense {
protected:

	static void get_sums(
		unsigned char* unwarped_img,
		int img_width,
		int img_height,
		int x,
		int* sums);

	static int get_correlation(
		unsigned char* magnitude,
		int y0,
		int y1,
		int patch_radius);

public:

	static void update_disparity_map(
		unsigned char* unwarped_img,
		int img_width,
		int img_height,
		int offset_y,
		int x_step,
		int max_disparity_percent,
		int correlation_radius,
		int smoothing_radius,
		int *disparity_space,
		int *disparity_map);

	static void show(
		unsigned char* img,
		int img_width,
		int img_height,
		int x_step,
		int max_disparity_percent,
		int *disparity_map);

};

#endif /* STACKEDSTEREODENSE_H_ */
