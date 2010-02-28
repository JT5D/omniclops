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

#include "posegraph.h"

posegraph::posegraph() {

	angular_tollerance_degrees = 20;
	translation_tollerance_x = 10;
	translation_tollerance_y = 10;
	no_of_samples = 100;
	match_tollerance = 5;

	pose_x = 0;
	pose_y = 0;
	pose_orientation_radians = 0;
	max_feature_ID = 0;
}

posegraph::~posegraph() {

}


void posegraph::update(
	vector<int> &feature_positions)
{
	// store previous features
	prev_features.clear();
	prev_features_IDs.clear();
	for (int f = 0; f < (int)features.size(); f++) {
		prev_features.push_back(features[f]);
	}
	for (int f = 0; f < (int)features_IDs.size(); f++) {
		prev_features_IDs.push_back(features_IDs[f]);
	}

	// update current features
	features.clear();
	features_IDs.clear();
	for (int f = 0; f < (int)feature_positions.size(); f++) {
		features.push_back(feature_positions[f]);
	}

	// data association
	int best_offset_x = 0;
	int best_offset_y = 0;
	float best_rotation_radians = 0;
    int match_score = trackfeatures::match(
    	prev_features,
    	prev_features_IDs,
    	features,
    	features_IDs,
    	angular_tollerance_degrees,
    	translation_tollerance_x,
    	translation_tollerance_y,
    	no_of_samples,
    	match_tollerance,
    	best_offset_x,
    	best_offset_y,
    	best_rotation_radians,
    	max_feature_ID);

    pose_x += best_offset_x;
    pose_y += best_offset_y;
    pose_orientation_radians += best_rotation_radians;
    if (pose_orientation_radians > 2*3.1415927f) pose_orientation_radians -= 2*3.1415927f;
    if (pose_orientation_radians < 0) pose_orientation_radians += 2*3.1415927f;

    // update the graph using the current pose estimate
    update_graph();
}

void posegraph::update_graph()
{
}
