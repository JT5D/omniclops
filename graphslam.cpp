/*
    functions for compatibility with graph slam methods
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

#include "graphslam.h"

graphslam::graphslam() {

	angular_tollerance_degrees = 20;
	translation_tollerance_x = 10;
	translation_tollerance_y = 10;
	no_of_samples = 100;
	match_tollerance_mm = 50;

	pose_x = 0;
	pose_y = 0;
	pose_orientation_radians = 0;
	max_feature_ID = 0;

	filename = "test.graph";
}

graphslam::~graphslam() {

}

/*!
 * \brief update the graph
 * \param feature_positions cartesian coordinates of observed features
 */
void graphslam::update(
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
    	match_tollerance_mm,
    	best_offset_x,
    	best_offset_y,
    	best_rotation_radians,
    	max_feature_ID);

    pose_x += best_offset_x;
    pose_y += best_offset_y;
    pose_orientation_radians += best_rotation_radians;
    if (pose_orientation_radians > 2*3.1415927f) pose_orientation_radians -= 2*3.1415927f;
    if (pose_orientation_radians < 0) pose_orientation_radians += 2*3.1415927f;

    // save to file
    export_2D(filename);
}

/*!
 * \brief export the current observation to file for use with a SLAM utility
 * \param filename filename to export to
 */
void graphslam::export_2D(
	string filename)
{
    ofstream file;
    file.open(filename.c_str(), ios::app);

    max_feature_ID++;

  	file << "VERTEX2 " << max_feature_ID << " ";
	file << pose_x << " " << pose_y << " " << pose_orientation_radians << "\n";

    for (int f = (int)features_IDs.size()-1; f >= 0; f--) {
    	if (features_IDs[f] != 0) {
    		int x = features[f*2];
    		int y = features[f*2 + 1];
    		float angle_radians = (float)atan2(y,x);
    		file << "EDGE2 " << features_IDs[f] << " " << max_feature_ID << " ";
    		file << y << " " << x << " " << " " << angle_radians;
    		file << " 1 0 1 1 0 0\n";
    	}
    }

    file.close();
}
