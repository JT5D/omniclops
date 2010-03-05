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

int posegraph::IndexOf(
    vector<landmark*> &lm,
    unsigned long ID)
{
    int max = (int)lm.size()-1;
    if (max > -1) {
		int min = 0;
		int curr = min + ((max - min)/2);
		landmark *n = lm[curr];
		while (n->ID != ID) {
			if (n->ID > ID) {
				min = curr;
			}
			else {
				max = curr;
			}
			curr = min + ((max - min)/2);
			n = lm[curr];
			if (max == min) {
				break;
			}
		}
		if (n->ID == ID)
			return(curr);
		else
			return(-1);
    }
    else return(-1);
}


void posegraph::update_graph()
{
	// add the new pose
	landmark* previous_pose = NULL;
	if ((int)poses.size() > 0) previous_pose = poses[(int)poses.size()-1];
	landmark* pose = new landmark((unsigned long)poses.size(), (int)pose_x, (int)pose_y, 0, previous_pose);
	poses.push_back(pose);

	float relative_rotation_radians2 = pose_orientation_radians + 3.1415927f;
	float relative_rotation_radians_cos = (float)cos(pose_orientation_radians);
	float relative_rotation_radians_sin = (float)sin(pose_orientation_radians);
	float relative_rotation_radians_cos2 = (float)cos(relative_rotation_radians2);
	float relative_rotation_radians_sin2 = (float)sin(relative_rotation_radians2);

	// add the observed landmarks
	unsigned long prev_ID = 0;
	int no_of_features = (int)features_IDs.size();
	for (int f = 0; f < no_of_features; f++) {
		unsigned long ID = features_IDs[f];
		int x = features[f*2];
		int y = features[f*2 + 1];
		int z = 0;

		// convert from egocentric coordinates into world coordinates
		int x2 = pose_x + (int)((relative_rotation_radians_cos2 * x) - (relative_rotation_radians_sin2 * y));
		int y2 = pose_y + (int)((relative_rotation_radians_sin * x) + (relative_rotation_radians_cos * y));

		landmark* curr_landmark = NULL;
		int index = IndexOf(landmarks, ID);
		if (index == -1) {
			if (ID < prev_ID) printf("WARNING: new landmark incorrect ID ordering\n");
			curr_landmark = new landmark(ID, x2, y2, z, pose);
			landmarks.push_back(curr_landmark);
			prev_ID = ID;
		}
		else {
			curr_landmark = landmarks[index];
			curr_landmark->Add(x2, y2, z, pose);
		}

		// link landmark to the pose
        //pose.landmarks.push_back(curr_landmark);
	}
}
