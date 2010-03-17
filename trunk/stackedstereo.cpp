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

/*
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
*/

unsigned int stackedstereo::SSD(
	unsigned char *img,
	int img_width,
	int img_height,
	int x0, int y0,
	int x1, int y1,
	int patch_width_pixels,
	int patch_height_pixels,
	int sampling_step)
{
	unsigned int ssd = 0;

	for (int dx = -patch_width_pixels; dx <= patch_width_pixels; dx += sampling_step) {
		int xx0 = x0 + dx;
		int xx1 = x1 + dx;
		if ((xx0 > -1) && (xx1 > -1) && (xx0 < img_width) && (xx1 < img_width)) {
			for (int dy = -patch_height_pixels; dy <= patch_height_pixels; dy += sampling_step) {
				int yy0 = y0 + dy;
				int yy1 = y1 + dy;
				if ((yy0 > -1) && (yy1 > -1) && (yy0 < img_height) && (yy1 < img_height)) {
					int n0 = (yy0*img_width + xx0)*3;
					int n1 = (yy1*img_width + xx1)*3;
					int b = img[n0] - img[n1];
					int g = img[n0+1] - img[n1+1];
					int r = img[n0+2] - img[n1+2];
					ssd += (unsigned int)(r*r + g*g + b*b);
				}
			}
		}
	}
	return(ssd);
}


void stackedstereo::match_corner_features(
	unsigned char *img,
	int img_width,
	int img_height,
	int max_matches,
	vector<int> &features,
	vector<int> &matches)
{
	int offset_x = 0;
	matches.clear();

	const int downsample_factor = 5;

	// bucket the features so that subsequent searching is reduced
	vector<int> features_lower[img_width/downsample_factor + 1];
	vector<int> features_upper[img_width/downsample_factor + 1];
	vector<unsigned int> scores;

	int minimum_match_length = img_height*20/100;
	const int border = 10;
	int h = img_height/2;
	for (int f = (int)features.size()-2; f >= 0; f -= 2) {
		int x = features[f];
		int y = features[f + 1];

		bool is_border = false;
		if (y > border) {
			int n = ((y-border)*img_width + x)*3;
			if ((img[n] == 0) && (img[n+1] == 0) && (img[n+2] == 0)) is_border = true;
		}
		if (y < img_height-1-border) {
			int n = ((y+border)*img_width + x)*3;
			if ((img[n] == 0) && (img[n+1] == 0) && (img[n+2] == 0)) is_border = true;
		}

		if (!is_border) {
			if (y < h) {
				int idx = x/downsample_factor;
				features_upper[idx].push_back(x);
				features_upper[idx].push_back(y);
			}
			else {
				int idx = (x + offset_x)/downsample_factor;
				if (idx < 0) idx += img_width/downsample_factor;
				if (idx >= img_width/downsample_factor) idx -= img_width/downsample_factor;
				features_lower[idx].push_back(x);
				features_lower[idx].push_back(y);
			}
		}
	}

	const int horizontal_tollerance = 1;
	const int patch_radius_pixels = 5;
	for (int i = (img_width/downsample_factor) - 4; i >= 2; i--) {
		if ((int)features_upper[i].size() > 0) {
			// for each feature in the upper mirror
			for (int f_upper = (int)features_upper[i].size()-2; f_upper >= 0; f_upper -= 2) {
				int x0 = features_upper[i][f_upper];
				int y0 = features_upper[i][f_upper+1];
				int matched_x = -1;
				int matched_y = -1;
				unsigned int min_ssd = 50000000;
				// compare to features in the lower mirror, with some horizontal tollerance
		        for (int i2 = i-horizontal_tollerance; i2 <= i+horizontal_tollerance; i2++) {
		        	for (int f_lower = (int)features_lower[i2].size()-2; f_lower >= 0; f_lower -= 2) {
						int x1 = features_lower[i2][f_lower];
						int y1 = features_lower[i2][f_lower+1];

						if (y1 - y0 > minimum_match_length) {
							unsigned int ssd = SSD(img, img_width, img_height, x0, y0, x1, y1, patch_radius_pixels, patch_radius_pixels, 1);
							if (ssd < min_ssd) {
								min_ssd = ssd;
								matched_x = x1;
								matched_y = y1;
							}
						}
		        	}
		        }

		        if (matched_x != -1) {
		        	matches.push_back(x0);
		        	matches.push_back(y0);
		        	matches.push_back(matched_x);
		        	matches.push_back(matched_y);
		        	scores.push_back(min_ssd);
		        }

			}
		}
	}

	int no_of_matches = (int)scores.size();
    int max = max_matches;
    if ((int)scores.size() < max) max = (int)scores.size();
    for (int i = 0; i < max; i++) {
    	unsigned int sc = scores[i];
    	int winner = -1;
    	for (int j = i + 1; j < no_of_matches; j++) {
    		if (scores[j] < sc) {
    			sc = scores[j];
    			winner = j;
    		}
    	}
        if (winner > -1) {
        	unsigned int temp = scores[i];
        	scores[i] = scores[winner];
        	scores[winner] = temp;
        	for (int k = 0; k < 4; k++) {
        		temp = matches[i*4 + k];
        		matches[i*4 + k] = matches[winner*4 + k];
        		matches[winner*4 + k] = temp;
        	}
        }
    }

}

void stackedstereo::show_matches(
	unsigned char* img,
	int img_width,
	int img_height,
	int no_of_matches,
	vector<int> &matches)
{
	if ((int)matches.size()/4 < no_of_matches) no_of_matches = (int)matches.size()/4;

	int r = 255, g = 0, b = 0;
	for (int i = 0; i < no_of_matches; i++) {
        int x0 = matches[i*4];
        int y0 = matches[i*4+1];
        int x1 = matches[i*4+2];
        int y1 = matches[i*4+3];
        drawing::drawLine(img, img_width, img_height, x0, y0, x1, y1, r, g, b, 0, false);
	}
}

void stackedstereo::show_rays(
	unsigned char* img,
	int img_width,
	int img_height,
	int max_range_mm,
	vector<int> &rays)
{
	memset((void*)img, '\0', img_width*img_height*3);

	int w = img_width/2;
	int h = img_height/2;
	for (int i = (int)rays.size()-6; i >= 0; i -= 6) {
		int x = rays[i];
		float angle = (float)x * 3.1415927f * 2 / img_width;
		int near = rays[i+1];
		int far = rays[i+3];
		float c0 =  (float)sin(angle) * w / max_range_mm;
		float c1 =  (float)cos(angle) * w / max_range_mm;
		int x0 = w + (int)(near * c0);
		int y0 = h + (int)(near * c1);
		int x1 = w + (int)(far * c0);
		int y1 = h + (int)(far * c1);
		drawing::drawLine(img, img_width, img_height, x0, y0, x1, y1, 255,255,255, 0, false);
	}
}

/*!
 * \brief converts matched features in the unwarped image into ray models
 * \param img colour image
 * \param img_width width of the image
 * \param img_height height of the image
 * \param uncertainty_pixels uncertainty in feature position in pixels
 * \param stereo_lookup lookup table for stereo ranges
 * \param max_range_mm maximum range
 * \param max_matches maximum number of matches to be considered
 * \param matches list of matched features (x0,y0, x1,y1)
 * \param rays returned ray models (near range, near_elevation, far range, far elevation, peak probability percent)
 */
void stackedstereo::matches_to_rays(
	unsigned char *img,
	int img_width,
	int img_height,
	int uncertainty_pixels,
	int* stereo_lookup,
	int max_range_mm,
	int max_matches,
	vector<int> &matches,
	vector<int> &rays)
{
	rays.clear();
	if (stereo_lookup == NULL) {
		printf("stereo_lookup missing\n");
	}
	else {
		int ray[8];
		int h = img_height/2;
		int step = uncertainty_pixels*2;
		int max = max_matches*4;
		if ((int)matches.size() < max) max = (int)matches.size();
		for (int i = max-4; i >= 0; i -= 4) {
			int x0 = matches[i];
			int y0 = matches[i+1];
			int y1 = matches[i+3];

			int r = 0;
			for (int yy0 = y0-uncertainty_pixels; yy0 <= y0+uncertainty_pixels; yy0 += step) {
				for (int yy1 = y1-uncertainty_pixels; yy1 <= y1+uncertainty_pixels; yy1 += step, r += 2) {
					int n = (yy0*h + yy1)*2;
					int range_mm = stereo_lookup[n];
					int elevation_mm = stereo_lookup[n+1];
					ray[r] = range_mm;
					ray[r+1] = elevation_mm;
				}
			}

			int min_range_mm = 0;
			int min_range_elevation_mm = 0;
			int min_idx = 0;
			int max_range_mm = 0;
			int max_range_elevation_mm = 0;
			int max_idx = 0;
			int mid_idx = 0;
			for (r = 0; r < 8; r += 2) {
				if ((ray[r] < min_range_mm) || (r == 0)) {
					min_range_mm = ray[r];
					min_range_elevation_mm = ray[r+1];
					min_idx = r;
				}
				if ((ray[r] > max_range_mm) || (r == 0)) {
					max_range_mm = ray[r];
					max_range_elevation_mm = ray[r+1];
					max_idx = r;
				}
			}

			if ((min_range_elevation_mm > 0) &&
			    (min_range_elevation_mm < max_range_mm)) {
				for (r = 0; r < 8; r += 2) {
					if ((r != min_idx) && (r != max_idx)) {
						mid_idx = r;
						break;
					}
				}

				if (ray[max_idx] > ray[min_idx]) {
					// position of the peak probability along the ray
					int peak_probability_percent =
						(ray[mid_idx] - ray[min_idx]) * 1000 /
						(ray[max_idx] - ray[min_idx]);

					rays.push_back(x0);
					rays.push_back(min_range_mm);
					rays.push_back(min_range_elevation_mm);
					rays.push_back(max_range_mm);
					rays.push_back(max_range_elevation_mm);
					rays.push_back(peak_probability_percent);
				}
			}
		}
	}
}

void stackedstereo::anaglyph(
	unsigned char* img,
	int img_width,
	int img_height,
	int offset_y)
{
	int h = img_height/2;
	for (int y = h; y < img_height; y++) {
		int y2 = y - h + offset_y;
		if (y2 > -1) {
			for (int x = 0; x < img_width; x++) {
				int n_lower = (y*img_width + x)*3;
				if (!((img[n_lower] == 0) && (img[n_lower+1] == 0) && (img[n_lower+2] == 0))) {
					int n_upper = (y2*img_width + x)*3;
					img[n_upper] = img[n_lower];
					img[n_upper+1] = 0;
					//img[n_upper+1] = img[n_lower+1];// + ((img[n_upper+1] - img[n_lower+1])/2);
					img[n_lower] = 0;
					img[n_lower+1] = 0;
					img[n_lower+2] = 0;
				}
			}
		}
	}

	int n = 0;
	for (int y = 0; y < img_height; y++) {
		for (int x = 0; x < img_width; x++, n += 3) {
			if (img[n+1] != 0) {
				img[n] = 0;
				img[n+1] = 0;
				img[n+2] = 0;
			}
			else {
				//img[n+1] = img[n];
			}
		}
	}
}

void stackedstereo::calibrate(
	unsigned char* img,
	int img_width,
	int img_height,
	int &offset_y)
{
	int patch_width_pixels = img_width/2;
	int patch_height_pixels = img_height/4;
	int min_offset_y = -img_height/8;
	int max_offset_y = img_height/8;

	int step = 4;
	unsigned int min_ssd = 0;
	int x = img_width/2;
    for (int offset = min_offset_y; offset <= max_offset_y; offset++) {
    	int y_upper = patch_height_pixels;
    	int y_lower = (img_height/2) + patch_height_pixels + offset;
        unsigned int ssd = SSD(
        	img, img_width, img_height,
        	x, y_upper,
        	x, y_lower,
        	patch_width_pixels,
        	patch_height_pixels,
        	step);
        if ((ssd < min_ssd) || (min_ssd == 0)) {
        	min_ssd = ssd;
        	offset_y = -offset;
        	printf("SSD %d\n", min_ssd);
        }
    }
}
