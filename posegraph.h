/*
    pose graph
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

#ifndef POSEGRAPH_H_
#define POSEGRAPH_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <fstream>
#include <algorithm>
#include "trackfeatures.h"
//#include "hypergraph/hypergraph.h"
using namespace std;

class posegraph {
public:
	vector<int> prev_features;
	vector<unsigned long> prev_features_IDs;
	vector<int> features;
	vector<unsigned long> features_IDs;
	int angular_tollerance_degrees;
	int translation_tollerance_x;
	int translation_tollerance_y;
	int no_of_samples;
	int match_tollerance;

	float pose_x;
	float pose_y;
	float pose_orientation_radians;
	unsigned long max_feature_ID;

	void update_graph();
	void update(vector<int> &feature_positions);

	posegraph();
	virtual ~posegraph();
};

#endif /* POSEGRAPH_H_ */
