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

int stackedstereo::SSD(
	unsigned char *img,
	int img_width,
	int img_height,
	int x0, int y0,
	int x1, int y1,
	int patch_radius_pixels)
{
	int ssd = 0;

	for (int dx = -patch_radius_pixels; dx <= patch_radius_pixels; dx++) {
		int xx0 = x0 + dx;
		int xx1 = x1 + dx;
		for (int dy = -patch_radius_pixels; dy <= patch_radius_pixels; dy++) {
			int yy0 = y0 + dy;
			int yy1 = y1 + dy;
            int n0 = (yy0*img_width + xx0)*3;
            int n1 = (yy1*img_width + xx1)*3;
            int b = img[n0] - img[n1];
            int g = img[n0+1] - img[n1+1];
            int r = img[n0+2] - img[n1+2];
            ssd += r*r + g*g + b*b;
		}
	}
	return(ssd);
}


void stackedstereo::match_corner_features(
	unsigned char *img,
	int img_width,
	int img_height,
	int max_ssd,
	vector<int> &features,
	vector<int> &matches)
{
	matches.clear();

	// bucket the features so that subsequent searching is reduced
	vector<int> features_lower[img_width/2 + 1];
	vector<int> features_upper[img_width/2 + 1];

	int h = img_height/2;
	for (int f = (int)features.size()-2; f >= 0; f -= 2) {
		int x = features[f];
		int y = features[f + 1];
        if (y < h) {
        	features_upper[x/2].push_back(x);
        	features_upper[x/2].push_back(y);
        }
        else {
        	features_lower[x/2].push_back(x);
        	features_lower[x/2].push_back(y);
        }
	}

	const int horizontal_tollerance = 1;
	const int patch_radius_pixels = 5;
	for (int i = (img_width/2) - 4; i >= 2; i--) {
		if ((int)features_upper[i].size() > 0) {
			// for each feature in the upper mirror
			for (int f_upper = (int)features_upper[i].size()-2; f_upper >= 0; f_upper -= 2) {
				int x0 = features_upper[i][f_upper];
				int y0 = features_upper[i][f_upper+1];
				int matched_x = -1;
				int matched_y = -1;
				int min_ssd = max_ssd;
				// compare to features in the lower mirror, with some horizontal tollerance
		        for (int i2 = i-horizontal_tollerance; i2 <= i+horizontal_tollerance; i2++) {
		        	for (int f_lower = (int)features_lower[i2].size()-2; f_lower >= 0; f_lower -= 2) {
						int x1 = features_lower[i2][f_lower];
						int y1 = features_lower[i2][f_lower+1];

						int n = ((y1-5)*img_width + x1)*3;
						if (!((img[n] == 0) && (img[n+1] == 0) && (img[n+2] == 0))) {
							int ssd = SSD(img, img_width, img_height, x0, y0, x1, y1, patch_radius_pixels);
							if (ssd < min_ssd) {
								min_ssd = ssd;
								matched_x = x1;
								matched_y = y1;
							}
						}
		        	}
		        }

		        if (matched_x > -1) {
		        	matches.push_back(x0);
		        	matches.push_back(y0);
		        	matches.push_back(matched_x);
		        	matches.push_back(matched_y);
		        }

			}
		}
	}
}

void stackedstereo::show(
	unsigned char* img,
	int img_width,
	int img_height,
	vector<int> &matches)
{
	int n = 0,r = 0, g = 255, b = 0;
	for (int i = (int)matches.size()-4; i >= 0; i -= 4, n++) {
        int x0 = matches[i];
        int y0 = matches[i+1];
        int x1 = matches[i+2];
        int y1 = matches[i+3];
        /*
        switch(n % 6) {
        case 0: {
        	r = 255;
        	g = 0;
        	b = 0;
        	break;
        }
        case 1: {
        	r = 0;
        	g = 255;
        	b = 0;
        	break;
        }
        case 2: {
        	r = 0;
        	g = 0;
        	b = 255;
        	break;
        }
        case 3: {
        	r = 255;
        	g = 0;
        	b = 255;
        	break;
        }
        case 4: {
        	r = 255;
        	g = 255;
        	b = 0;
        	break;
        }
        case 5: {
        	r = 0;
        	g = 255;
        	b = 255;
        	break;
        }
        }
        */
        drawing::drawLine(img, img_width, img_height, x0, y0, x1, y1, r, g, b, 0, false);
	}
}
