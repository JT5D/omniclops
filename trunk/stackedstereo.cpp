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

#include "stackedstereo.h"

stackedstereo::stackedstereo() {

}

stackedstereo::~stackedstereo() {
}

int stackedstereo::get_features_from_unwarped(
	unsigned char* unwarped_img,
	int img_width,
	int img_height,
	int x,
	short* magnitude,
	short* edge_magnitude,
	short* features)
{
	int no_of_features = 0;
	int suppression_radius = 10;

	for (int y = 3; y < img_height-3; y++) {
		int n = (y*img_width + x)*3;
		magnitude[y] = short(unwarped_img[n] + unwarped_img[n+1] + unwarped_img[n+2]);
	}
	for (int y = 3; y < img_height-3; y++) {
		edge_magnitude[y] =
			(short)((abs(magnitude[y-1] - magnitude[y+1]) * 4) +
			abs(magnitude[y-2] - magnitude[y+2]) +
			abs(magnitude[y-3] - magnitude[y+3]));
	}

	// non-maximal suppression
	for (int y = 3; y < img_height-suppression_radius; y++) {
		short mag = edge_magnitude[y];
		if (mag > 0) {
			for (int y2 = y+1; y2 < y + suppression_radius; y2++) {
				if (edge_magnitude[y2] <= mag) {
					edge_magnitude[y2] = 0;
				}
				else {
					edge_magnitude[y] = 0;
					break;
				}
			}
			if (edge_magnitude[y] > (short)0) {
				if (no_of_features < MAX_STACKED_STEREO_FEATURES) {
					features[no_of_features] = y;
					no_of_features++;
				}
				else {
					printf("Maximum number of stereo features per radial line reached\n");
				}
			}
		}
	}
	return(no_of_features);
}

void stackedstereo::match_unwarped(
	int x,
	int no_of_features,
	short* features,
	int* unwarp_lookup,
	int* ray_map,
	int img_width,
	int img_height,
	int max_range_mm,
	vector<short> &points)
{
	int max_upper_index = 0;
	for (int f = 0; f < no_of_features; f++) {
        int y = features[f];
        if (y >= img_height/2) {
        	max_upper_index = f;
        	break;
        }
	}

	vector<int> best_points;
	vector<int> projected_lower_mirror_features;
	vector<int> projected_upper_mirror_features;

	int min_range_mm = 300;
	int best_range_mm = 0;
	int max_hits = 0;
	for (int range_mm = min_range_mm; range_mm < max_range_mm; range_mm += 5) {
		// project the lower features
		projected_lower_mirror_features.clear();
	    for (int f_lower = no_of_features-1; f_lower > max_upper_index; f_lower--) {

	    	// get the edge feature position
            int y_lower = features[f_lower];
            int n_lower = unwarp_lookup[y_lower*img_width + x]*6;

            int x0_lower = ray_map[n_lower];
            int y0_lower = ray_map[n_lower+1];
            int x1_lower = ray_map[n_lower+3];
            int y1_lower = ray_map[n_lower+4];

            // get the ray vector and length
            int dx = x1_lower - x0_lower;
            int dy = y1_lower - y0_lower;
            int ray_length_mm = (int)sqrt(dx*dx + dy*dy);

            if (ray_length_mm > 0) {
                int z0_lower = ray_map[n_lower+2];
                int z1_lower = ray_map[n_lower+5];
                int dz = z1_lower - z0_lower;
                // project the ray to this desired range
                int projected_x = x0_lower + (range_mm * dx / ray_length_mm);
                int projected_y = y0_lower + (range_mm * dy / ray_length_mm);
                int projected_z = z0_lower + (range_mm * dz / ray_length_mm);
                projected_lower_mirror_features.push_back(projected_x);
                projected_lower_mirror_features.push_back(projected_y);
                projected_lower_mirror_features.push_back(projected_z);
            }
	    }

	    // project the upper features
		projected_upper_mirror_features.clear();
	    for (int f_upper = max_upper_index; f_upper >= 0; f_upper--) {

	    	// get the edge feature position
            int y_upper = features[f_upper];
            int n_upper = unwarp_lookup[y_upper*img_width + x]*6;

            int x0_upper = ray_map[n_upper];
            int y0_upper = ray_map[n_upper+1];
            int x1_upper = ray_map[n_upper+3];
            int y1_upper = ray_map[n_upper+4];

            // get the ray vector and length
            int dx = x1_upper - x0_upper;
            int dy = y1_upper - y0_upper;
            int ray_length_mm = (int)sqrt(dx*dx + dy*dy);

            if (ray_length_mm > 0) {
                int z0_upper = ray_map[n_upper+2];
                int z1_upper = ray_map[n_upper+5];
                int dz = z1_upper - z0_upper;
                // project the ray to this desired range
                int projected_x = x0_upper + (range_mm * dx / ray_length_mm);
                int projected_y = y0_upper + (range_mm * dy / ray_length_mm);
                int projected_z = z0_upper + (range_mm * dz / ray_length_mm);
                projected_upper_mirror_features.push_back(projected_x);
                projected_upper_mirror_features.push_back(projected_y);
                projected_upper_mirror_features.push_back(projected_z);
            }
	    }

	    // match the feature positions
        int max_separation = 20;
	    int hits = 0;
	    for (int f_lower = (int)projected_lower_mirror_features.size()-3; f_lower >= 0; f_lower -= 3) {
            int lower_z = projected_lower_mirror_features[f_lower + 2];
            for (int f_upper = (int)projected_upper_mirror_features.size()-3; f_upper >= 0; f_upper -= 3) {
                int upper_z = projected_upper_mirror_features[f_upper + 2];
                int dz = abs(upper_z - lower_z);
                if (dz < max_separation) {
                	hits++;
                	//break;
                }
            }
	    }

	    int threshold = 0;//((int)projected_lower_mirror_features.size()/3) * 70/100;

        if ((hits > threshold) && (hits > max_hits)) {
        	max_hits = hits;
        	best_range_mm = range_mm;
        	best_points.clear();
    	    for (int f_lower = (int)projected_lower_mirror_features.size()-3; f_lower >= 0; f_lower -= 3) {
                best_points.push_back(projected_lower_mirror_features[f_lower]);
                best_points.push_back(projected_lower_mirror_features[f_lower + 1]);
                best_points.push_back(projected_lower_mirror_features[f_lower + 2]);
    	    }
        }
	}

	for (int i = 0; i < (int)best_points.size(); i++) {
		points.push_back(best_points[i]);
	}

	printf("range %d (%d/%d)\n", best_range_mm, max_hits, (int)projected_lower_mirror_features.size()/3);
}

void stackedstereo::get_point_cloud(
	unsigned char* unwarped_img,
	int img_width,
	int img_height,
	int* unwarp_lookup,
	int* ray_map,
	int max_range_mm,
	vector<short> &points,
	bool show)
{
	points.clear();
	short magnitude[img_height];
	short edge_magnitude[img_height];
	short features[MAX_STACKED_STEREO_FEATURES];

	memset((void*)magnitude, '\0', img_height*sizeof(short));
	memset((void*)edge_magnitude, '\0', img_height*sizeof(short));

	for (int x = 0; x < img_width; x++) {
		int no_of_features = get_features_from_unwarped(
			unwarped_img,
			img_width,
			img_height,
			x,
			magnitude,
			edge_magnitude,
			features);

		match_unwarped(
			x,
			no_of_features,
			features,
			unwarp_lookup,
			ray_map,
			img_width,
			img_height,
			max_range_mm,
			points);
	}

}
