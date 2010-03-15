/*
    Stereo correspondence for stacked omnidirectional vision
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

#ifndef STACKEDSTEREO_H_
#define STACKEDSTEREO_H_

#define MAX_STACKED_STEREO_FEATURES   100

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <fstream>
#include <vector>
#include "drawing.h"
#include "omni.h"
using namespace std;

class stackedstereo {
public:

	/*
	static int get_features_from_unwarped(
		unsigned char* unwarped_img,
		int img_width,
		int img_height,
		int x,
		short* magnitude,
		short* edge_magnitude,
		short* features);
		*/

	static int SSD(
		unsigned char *img,
		int img_width,
		int img_height,
		int x0, int y0,
		int x1, int y1,
		int patch_radius_pixels);

	static void match_corner_features(
		unsigned char *img,
		int img_width,
		int img_height,
		int max_matches,
		vector<int> &features,
		vector<int> &matches);

	static void show(
		unsigned char* img,
		int img_width,
		int img_height,
		int no_of_matches,
		vector<int> &matches);
};

#endif /* STACKEDSTEREO_H_ */
