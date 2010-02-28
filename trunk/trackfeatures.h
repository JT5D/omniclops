/*
    track a set of features observed by omnidirectional vision
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

#ifndef TRACKFEATURES_H_
#define TRACKFEATURES_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <fstream>
#include <algorithm>
using namespace std;

class trackfeatures {
public:
    static void populate_IDs(
   		vector<int> &prev_features,
   		vector<unsigned long> &prev_features_IDs,
   		vector<int> &features,
   		vector<unsigned long> &features_IDs);

    static int match_rotation(
    	vector<int> &prev_features,
    	vector<unsigned long> &prev_features_IDs,
    	vector<int> &features,
    	vector<unsigned long> &features_IDs,
    	int angular_tollerance_degrees,
    	int no_of_samples,
    	int match_tollerance,
    	float &best_rotation_radians);

    static int match(
    	vector<int> &prev_features,
    	vector<unsigned long> &prev_features_IDs,
    	vector<int> &features,
    	vector<unsigned long> &features_IDs,
    	int angular_tollerance_degrees,
    	int translation_tollerance_x,
    	int translation_tollerance_y,
    	int no_of_samples,
    	int match_tollerance,
    	int &best_offset_x,
    	int &best_offset_y,
    	float &best_rotation_radians,
    	unsigned long &max_feature_ID);
};

#endif /* TRACKFEATURES_H_ */
