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

/*!
 * \brief returns a measure of similarity between a pair of features
 * \param edge_magnitude edge magnitudes along the radial line
 * \param magnitude pixel magnitudes along the radial line
 * \param r0 radial index of the first feature
 * \param r1 radial index of the first feature
 * \return measure of similarity
 */
short stackedstereo::get_feature_similarity(
	short* edge_magnitude,
	short* magnitude,
    int r0,
    int r1)
{
	short similarity = (short)(1 +
		(
		abs(edge_magnitude[r0-3] - edge_magnitude[r1-3]) +
		abs(edge_magnitude[r0-2] - edge_magnitude[r1-2]) +
		abs(edge_magnitude[r0-1] - edge_magnitude[r1-1]) +
		abs(edge_magnitude[r0] - edge_magnitude[r1]) +
		abs(edge_magnitude[r0+1] - edge_magnitude[r1+1]) +
		abs(edge_magnitude[r0+2] - edge_magnitude[r1+2]) +
		abs(edge_magnitude[r0+3] - edge_magnitude[r1+3]) +
		abs(magnitude[r0-3] - magnitude[r1-3]) +
		abs(magnitude[r0-2] - magnitude[r1-2]) +
		abs(magnitude[r0-1] - magnitude[r1-1]) +
		abs(magnitude[r0] - magnitude[r1]) +
		abs(magnitude[r0+1] - magnitude[r1+1]) +
		abs(magnitude[r0+2] - magnitude[r1+2]) +
		abs(magnitude[r0+3] - magnitude[r1+3])
		));

	return(similarity);
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
	short* magnitude,
	short* edge_magnitude,
	int* unwarp_lookup_reverse,
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

	int xx = img_width/2;
	float ix=0,iy=0;
	for (int f0 = 0; f0 < max_upper_index; f0++) {
        int y0 = features[f0];
        int n0 = unwarp_lookup_reverse[y0*img_width + xx]*6;

        int xx0 = ray_map[n0+1];
        int yy0 = ray_map[n0+2];
        int xx1 = ray_map[n0+4];
        int yy1 = ray_map[n0+5];

        if ((ray_map[n0+3] != 0) && (xx1 >= 0)) {
        	printf("ray not horizontal\n");
        }

        for (int f1 = max_upper_index; f1 < no_of_features; f1++) {
        	int y1 = features[f1];
        	int n1 = unwarp_lookup_reverse[y1*img_width + xx]*6;

            int xx2 = ray_map[n1+1];
            int yy2 = ray_map[n1+2];
            int xx3 = ray_map[n1+4];
            int yy3 = ray_map[n1+5];
        	omni::intersection(xx0,yy0,xx1,yy1,xx2,yy2,xx3,yy3,ix,iy);
        	printf("%d %d    %d %d %d  %d %d   %f %f\n", n0,n1, ray_map[n0], xx2,yy2,xx3,yy3,ix,iy);
        	//printf("%f %f\n", ix,iy);

        	if ((ix > -max_range_mm) && (ix < max_range_mm) &&
        		(iy > -max_range_mm) && (iy < max_range_mm)) {
        		short diff = get_feature_similarity(
        			edge_magnitude,
        			magnitude,
        		    y0, y1);

        		points.push_back((short)x);
        		points.push_back((short)ix);
        		points.push_back((short)iy);
        		points.push_back(diff);

        		//printf("%f %f %d\n", ix,iy,diff);
        	}
        }
	}
}

void stackedstereo::get_point_cloud(
	unsigned char* unwarped_img,
	int img_width,
	int img_height,
	int* unwarp_lookup_reverse,
	int* ray_map,
	int max_range_mm,
	vector<short> &points,
	bool show)
{
	points.clear();
	vector<short> new_points;
	short magnitude[img_height];
	short edge_magnitude[img_height];
	short features[MAX_STACKED_STEREO_FEATURES];

	memset((void*)magnitude, '\0', img_height*sizeof(short));
	memset((void*)edge_magnitude, '\0', img_height*sizeof(short));

	vector<int> feats;

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
			magnitude,
			edge_magnitude,
			unwarp_lookup_reverse,
			ray_map,
			img_width,
			img_height,
			max_range_mm,
			new_points);

	}

}
