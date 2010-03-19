/*
    Dense stereo correspondence for stacked omnidirectional vision
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

#include "stackedstereodense.h"

void stackedstereodense::get_sums(
	unsigned char* unwarped_img,
	int img_width,
	int img_height,
	int x,
	int* sums)
{
	int n = x*3;
	int stride = img_width*3;
	sums[0] = 0;
	for (int y = 1; y < img_height; y++, n += stride) {
		sums[y] = sums[y-1] + unwarped_img[n] + unwarped_img[n+1] + unwarped_img[n+2];
		if (x > 2) {
			sums[y] += unwarped_img[n-3] + unwarped_img[n-2] + unwarped_img[n-1];
			sums[y] += unwarped_img[n-6] + unwarped_img[n-5] + unwarped_img[n-4];
			sums[y] += unwarped_img[n-9] + unwarped_img[n-8] + unwarped_img[n-7];
		}
		if (x < img_width - 3) {
			sums[y] += unwarped_img[n+3] + unwarped_img[n+4] + unwarped_img[n+5];
			sums[y] += unwarped_img[n+6] + unwarped_img[n+7] + unwarped_img[n+8];
			sums[y] += unwarped_img[n+9] + unwarped_img[n+10] + unwarped_img[n+11];
		}
	}
}

int stackedstereodense::get_correlation(
	unsigned char* magnitude,
	int y0,
	int y1,
	int patch_radius)
{
	int correlation = 0;
	int anticorrelation = 0;

	for (int y = -patch_radius; y <= patch_radius; y++) {
		int yy0 = y0 + y;
		int yy1 = y1 + y;

		if ((magnitude[yy0] != 0) && (magnitude[yy1] != 0)) {
			int diff = magnitude[yy0] - magnitude[yy1];
			if (diff < 0) diff = -diff;
			correlation += 255 - diff;

			//diff = magnitude[yy0] - (255 - magnitude[yy1]);
			//if (diff < 0) diff = -diff;
			//anticorrelation += 255 - diff;
		}
	}

   	correlation = correlation - anticorrelation;

	return(correlation);
}

void stackedstereodense::update_disparity_map(
	unsigned char* unwarped_img,
	int img_width,
	int img_height,
	int offset_y,
	int x_step,
	int max_disparity_percent,
	int correlation_radius,
	int smoothing_radius,
	int *disparity_space,
	int *disparity_map)
{
	int half_height = img_height/2;
	int max_disparity = max_disparity_percent * half_height / 100;
	int sums[img_height];

	int img_width2 = img_width / x_step;
	int x2 = 0;
	int stride = img_width*3;
	int correlation_radius_inner = correlation_radius/2;

	int disparity = 0;
	while (disparity < max_disparity) {

		// insert correlation values into the buffer
		for (int x = 0; x < img_width; x += x_step, x2++) {

			get_sums(unwarped_img, img_width, img_height, x, sums);

			int y_lower = half_height;
			for (int y_upper = disparity; y_upper < half_height - offset_y; y_upper++, y_lower++) {

				int upper_response =
					(sums[y_upper + correlation_radius] - sums[y_upper - correlation_radius]) -
					((sums[y_upper + correlation_radius_inner] - sums[y_upper - correlation_radius_inner])*4);

				int lower_response =
					(sums[y_lower + correlation_radius] - sums[y_lower - correlation_radius]) -
					((sums[y_lower + correlation_radius_inner] - sums[y_lower - correlation_radius_inner])*4);

				disparity_space[y_upper*img_width2 + x2] = -abs(upper_response - lower_response);
			}
		}

		// update the disparity map
		int smoothing_radius_horizontal = 1;
		for (int x = smoothing_radius_horizontal; x <= img_width2 - smoothing_radius_horizontal; x++) {
			for (int y = smoothing_radius; y < half_height - smoothing_radius; y++) {

				int area_correlation = 0;
				for (int y2 = y - smoothing_radius; y2 < y + smoothing_radius; y2++) {
				    for (x2 = x - smoothing_radius_horizontal; x2 <= x + smoothing_radius_horizontal; x2++) {
					    int n2 = y2*img_width2 + x2;
					    area_correlation += disparity_space[n2];
				    }
				}

				int n = (y*img_width2 + x)*2;
				if ((area_correlation > disparity_map[n]) || (disparity == 0)) {
					disparity_map[n] = area_correlation;
					disparity_map[n + 1] = disparity;
				}
			}
		}

		disparity += 10;// + ((max_disparity - disparity)/8);
	}
}

void stackedstereodense::show(
	unsigned char* img,
	int img_width,
	int img_height,
	int x_step,
	int max_disparity_percent,
	int *disparity_map)
{
	int half_height = img_height/2;
	int img_width2 = img_width / x_step;
	int max_disparity = half_height * max_disparity_percent / 100;

	for (int y = 0; y < half_height; y++) {
		for (int x = 0; x < img_width; x++) {
			int n = ((y*img_width) + x)*3;
			int n2 = ((y*img_width2) + (x/x_step))*2;
            int disparity = disparity_map[n2 + 1] * 255  / max_disparity;
            img[n] = (unsigned char)disparity;
            img[n+1] = (unsigned char)disparity;
            img[n+2] = (unsigned char)disparity;
		}
	}
}
