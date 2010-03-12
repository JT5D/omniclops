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
	static short get_feature_similarity(
		short* edge_magnitude,
		short* magnitude,
	    int r0,
	    int r1);

	static int get_features_from_unwarped(
		unsigned char* unwarped_img,
		int img_width,
		int img_height,
		int x,
		short* magnitude,
		short* edge_magnitude,
		short* features);

	static void match_unwarped(
		int x,
		int no_of_features,
		short* features,
		short* magnitude,
		short* edge_magnitude,
		int* unwarp_lookup_reverse,
		int* ray_map,
		int img_width,
		int img_height,
		int max_range_mm,
		vector<short> &points);

	static void get_point_cloud(
		unsigned char* unwarped_img,
		int img_width,
		int img_height,
		int* unwarp_lookup_reverse,
		int* ray_map,
		int max_range_mm,
		vector<short> &points,
		bool show);

	stackedstereo();
	virtual ~stackedstereo();
};

#endif /* STACKEDSTEREO_H_ */
