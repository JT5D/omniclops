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

#include "trackfeatures.h"

/*!
 * \brief creates lists which store ID numbers
 * \param prev_features list of previously observed features (x,y)
 * \param prev_features_IDs list of ID numbers for previously observed features
 * \param features list of observed features (x,y)
 * \param features_IDs list of ID numbers for observed features
 */
void trackfeatures::populate_IDs(
	vector<int> &prev_features,
	vector<unsigned long> &prev_features_IDs,
	vector<int> &features,
	vector<unsigned long> &features_IDs)
{
	// populate ID lists
	if ((int)prev_features_IDs.size() != (int)prev_features.size()) {
		prev_features_IDs.clear();
		for (int i = (int)prev_features.size()-1; i >= 0; i--) {
			prev_features_IDs.push_back(0);
		}
	}

	features_IDs.clear();
	for (int i = (int)features.size()-1; i >= 0; i--) {
		features_IDs.push_back(0);
	}
}

/*!
 * \brief detect the relative rotation between the two sets of features
 * \param prev_features list of previously observed features (x,y)
 * \param prev_features_IDs list of ID numbers for previously observed features
 * \param features list of observed features (x,y)
 * \param features_IDs list of ID numbers for observed features
 * \param angular_tollerance_degrees range of angular search
 * \param no_of_samples the number of orientations to try
 * \param match_tollerance how close to features need to be in order to be considered to be matched (in pixels)
 * \param best_rotation_radians returned best relative rotation in radians
 * \return percentage of features matched in the range 0-10000
 */
int trackfeatures::match_rotation(
	vector<int> &prev_features,
	vector<unsigned long> &prev_features_IDs,
	vector<int> &features,
	vector<unsigned long> &features_IDs,
	int angular_tollerance_degrees,
	int no_of_samples,
	int match_tollerance,
	float &best_rotation_radians)
{
	if (((int)prev_features.size()/2) > 0) {
		populate_IDs(prev_features,prev_features_IDs,features,features_IDs);

		best_rotation_radians = 0;
		int max_matches = 0;
		int match_threshold = ((int)prev_features.size()/2) * 80 / 100;
		for (int sample = 0; sample < no_of_samples; sample++) {
			float relative_rotation_radians = ((sample * angular_tollerance_degrees * 2 / (float)no_of_samples) - angular_tollerance_degrees) * 3.1415927f / 180;

			float relative_rotation_radians2 = relative_rotation_radians + 3.1415927f;
			float relative_rotation_radians_cos = (float)cos(relative_rotation_radians);
			float relative_rotation_radians_sin = (float)sin(relative_rotation_radians);
			float relative_rotation_radians_cos2 = (float)cos(relative_rotation_radians2);
			float relative_rotation_radians_sin2 = (float)sin(relative_rotation_radians2);

			int matches = 0;
			for (int f = (int)prev_features.size()-2; f >= 0; f -= 2) {
				int x = prev_features[f];
				int y = prev_features[f + 1];
				int x2 = (int)((relative_rotation_radians_cos2 * x) - (relative_rotation_radians_sin2 * y));
				int y2 = (int)((relative_rotation_radians_sin * x) + (relative_rotation_radians_cos * y));
				for (int f2 = (int)features.size()-2; f2 >= 0; f2 -= 2) {
					int dx = features[f2] - x2;
					if ((dx > -match_tollerance) && (dx <= match_tollerance)) {
						int dy = features[f2+1] - y2;
						if ((dy > -match_tollerance) && (dy <= match_tollerance)) {
							matches++;
							break;
						}
					}
				}
			}

			if (matches > max_matches) {
				max_matches = matches;
				best_rotation_radians = relative_rotation_radians;
				if (matches > match_threshold) sample = no_of_samples;
			}
		}

		return(max_matches * 10000 / ((int)prev_features.size()/2));
	}
	else {
		return(0);
	}
}

/*!
 * \brief match previous and current set of features, and assign ID numbers to them
 * \param prev_features list of previously observed features (x,y)
 * \param prev_features_IDs list of ID numbers for previously observed features
 * \param features list of observed features (x,y)
 * \param features_IDs list of ID numbers for observed features
 * \param angular_tollerance_degrees range of angular search
 * \param translation_tollerance_x maximum x translation in pixels
 * \param translation_tollerance_y maximum y translation in pixels
 * \param no_of_samples the number of orientations to try
 * \param match_tollerance how close to features need to be in order to be considered to be matched (in pixels)
 * \param best_offset_x best matching x offset in pixels
 * \param best_offset_y best matching x offset in pixels
 * \param best_rotation_radians returned best relative rotation in radians
 * \param max_feature_ID maximum feature ID number
 * \return percentage of features matched in the range 0-10000
 */
int trackfeatures::match(
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
	unsigned long &max_feature_ID)
{
	int rotation_score = match_rotation(
		prev_features,
		prev_features_IDs,
		features,
		features_IDs,
		angular_tollerance_degrees,
		no_of_samples,
		match_tollerance,
		best_rotation_radians);

	//printf("rotation_score %d\n", rotation_score);

	float relative_rotation_radians2 = best_rotation_radians + 3.1415927f;
	float relative_rotation_radians_cos = (float)cos(best_rotation_radians);
	float relative_rotation_radians_sin = (float)sin(best_rotation_radians);
	float relative_rotation_radians_cos2 = (float)cos(relative_rotation_radians2);
	float relative_rotation_radians_sin2 = (float)sin(relative_rotation_radians2);

	best_offset_x = 0;
	best_offset_y = 0;
	int max_matches = 0;
	for (int offset_x = -translation_tollerance_x; offset_x <= translation_tollerance_x; offset_x++) {
		for (int offset_y = -translation_tollerance_y; offset_y <= translation_tollerance_y; offset_y++) {

			int matches = 0;
			for (int f = (int)prev_features.size()-2; f >= 0; f -= 2) {
				int x = prev_features[f];
				int y = prev_features[f + 1];
				int x2 = offset_x + (int)((relative_rotation_radians_cos2 * x) - (relative_rotation_radians_sin2 * y));
				int y2 = offset_y + (int)((relative_rotation_radians_sin * x) + (relative_rotation_radians_cos * y));
				for (int f2 = (int)features.size()-2; f2 >= 0; f2 -= 2) {
					int dx = features[f2] - x2;
					if ((dx > -match_tollerance) && (dx <= match_tollerance)) {
						int dy = features[f2+1] - y2;
						if ((dy > -match_tollerance) && (dy <= match_tollerance)) {
							matches++;
							break;
						}
					}
				}
			}

			if (matches > max_matches) {
				max_matches = matches;
				best_offset_x = offset_x;
				best_offset_y = offset_y;
			}
		}
	}

	if ((int)prev_features.size()/2 > 0) {

        // assign IDs to current features
		for (int f = (int)prev_features.size()-2; f >= 0; f -= 2) {
			if (prev_features_IDs[f/2] == 0) {
				max_feature_ID++;
				prev_features_IDs[f/2] = max_feature_ID;
			}
			int x = prev_features[f];
			int y = prev_features[f + 1];
			int x2 = best_offset_x + (int)((relative_rotation_radians_cos2 * x) - (relative_rotation_radians_sin2 * y));
			int y2 = best_offset_y + (int)((relative_rotation_radians_sin * x) + (relative_rotation_radians_cos * y));
			for (int f2 = (int)features.size()-2; f2 >= 0; f2 -= 2) {
				int dx = features[f2] - x2;
				if ((dx > -match_tollerance) && (dx <= match_tollerance)) {
					int dy = features[f2+1] - y2;
					if ((dy > -match_tollerance) && (dy <= match_tollerance)) {
						features_IDs[f2/2] = prev_features_IDs[f/2];
						break;
					}
				}
			}
		}

	    return(max_matches * 10000 / ((int)prev_features.size()/2));
	}
	else {
		return(0);
	}
}
