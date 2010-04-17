/*
    Stacked omnidirectional dense stereo correspondence
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

/*!
 * \brief converts the unwarped image to a set of conventional stereo images.  This means that a conventional stereo algorithm can be applied.
 * \param img_unwarped unwarped colour image
 * \param img_width_unwarped width of the unwarped image
 * \param img_height_unwarped height of the unwarped image
 * \param offset_y_unwarped vertical calibration adjustment
 * \param img_left left image for conventional stereo
 * \param img_right right image for conventional stereo
 * \param img_width width of the conventional stereo image
 * \param img_height height of the conventional stereo image
 */
void stackedstereodense::unwarped_to_stereo_images(
	unsigned char* img_unwarped,
	int img_width_unwarped,
	int img_height_unwarped,
	int offset_y_unwarped,
	unsigned char* img_left,
	unsigned char* img_right,
	int img_width,
	int img_height)
{
	int lower_mirror_ty = img_height_unwarped/2;
	int lower_mirror_by = img_height_unwarped;
	int upper_mirror_ty = offset_y_unwarped;
	int upper_mirror_by = (img_height_unwarped/2) + offset_y_unwarped;
	if (upper_mirror_by > img_height_unwarped/2) {
		upper_mirror_by = img_height_unwarped/2;
	}

    for (int mirror = 0; mirror < 2; mirror++) {

    	unsigned char* img = img_left;
    	int ty = lower_mirror_ty;
    	int by = lower_mirror_by;
    	if (mirror > 0) {
    		img = img_right;
        	ty = upper_mirror_ty;
        	by = upper_mirror_by;
    	}

    	for (int y = 0; y < img_height; y++) {
    		int xx = y * (img_width_unwarped-1) / img_height;
        	for (int x = 0; x < img_width; x++) {
        		int yy = by - (x * (by - ty) / img_width);
        		if ((yy > 0) && (yy < img_height_unwarped)) {
					int n = (y*img_width + x)*3;
					int n_unwarped = (yy*img_width_unwarped + xx)*3;
					img[n] = img_unwarped[n_unwarped];
					img[n+1] = img_unwarped[n_unwarped+1];
					img[n+2] = img_unwarped[n_unwarped+2];
        		}
        	}
    	}

    }
}

/*!
 * \brief show the disparity map
 * \param img_unwarped unwarped colour image data
 * \param img_width_unwarped width of the unwarped image
 * \param img_height_unwarped height of the unwarped image
 * \param img_width width of the image
 * \param img_height height of the image
 * \param vertical_sampling vertical sampling rate
 * \param smoothing_radius radius in pixels used for disparity space smoothing
 * \param max_disparity_percent maximum disparity as a percentage of image width
 * \param disparity_map disparity map to be shown
 */
void stackedstereodense::show(
	unsigned char* img_unwarped,
	int img_width_unwarped,
	int img_height_unwarped,
	int img_width,
	int img_height,
	int vertical_sampling,
	int smoothing_radius,
	int max_disparity_percent,
	unsigned int *disparity_map)
{
	int max_disparity_pixels = img_width * max_disparity_percent * STEREO_DENSE_SUB_PIXEL / 100;
	int width2 = img_width/smoothing_radius;

	memset((void*)img_unwarped, '\0', img_width_unwarped*img_height_unwarped*3);

	for (int y = 0; y < img_height; y++) {
		int x_unwarped = y * img_width_unwarped / img_height;
		int n2 = ((y/vertical_sampling)/STEREO_DENSE_SMOOTH_VERTICAL)*width2;
		for (int x = 0; x < img_width; x++) {
			int y_unwarped = x * img_height_unwarped / (img_width*2);

			int n = ((y_unwarped*img_width) + x_unwarped)*3;
			int n2b = (n2 + (x/smoothing_radius))*2;
            unsigned char disparity = (unsigned char)((int)disparity_map[n2b + 1] * 255  / max_disparity_pixels);
            img_unwarped[n] = disparity;
            img_unwarped[n+1] = disparity;
            img_unwarped[n+2] = disparity;
		}
	}
}

/*!
 * \brief creates a disparity map for stacked omnidirectional stereo vision
 * \param img_unwarped unwarped colour image
 * \param img_width_unwarped width of the unwarped image
 * \param img_height_unwarped height of the unwarped image
 * \param offset_y_unwarped vertical calibration adjustment
 * \param img_left left image for conventional stereo
 * \param img_right right image for conventional stereo
 * \param img_width width of the conventional stereo image
 * \param img_height height of the conventional stereo image
 * \param offset_x calibration offset x
 * \param offset_y calibration offset y
 * \param vertical_sampling vertical sampling rate - we don't need every row
 * \param max_disparity_percent maximum disparity as a percentage of image width
 * \param correlation_radius radius in pixels used for patch matching
 * \param smoothing_radius radius in pixels used for smoothing of the disparity space
 * \param disparity_step step size for sampling different disparities
 * \param disparity_threshold_percent a threshold applied to the disparity map
 * \param despeckle optionally apply despeckling to clean up the disparity map
 * \param disparity_space array used for the disparity space
 * \param disparity_map returned disparity map
 */
void stackedstereodense::update_disparity_map(
	unsigned char* img_unwarped,
	int img_width_unwarped,
	int img_height_unwarped,
	int offset_y_unwarped,
	unsigned char* img_left,
	unsigned char* img_right,
	int img_width,
	int img_height,
	int offset_x,
	int offset_y,
	int vertical_sampling,
	int max_disparity_percent,
	int correlation_radius,
	int smoothing_radius,
	int disparity_step,
	int disparity_threshold_percent,
	bool despeckle,
	int cross_checking_threshold,
	unsigned int *disparity_space,
	unsigned int *disparity_map)
{
	// create a pair of conventional stereo images
	unwarped_to_stereo_images(
		img_unwarped,
		img_width_unwarped,
		img_height_unwarped,
		offset_y_unwarped,
		img_left,
		img_right,
		img_width,
		img_height);

	// create the disparity map
	stereodense::update_disparity_map(
		img_left,
		img_right,
		img_width,
		img_height,
		offset_x,
		offset_y,
		vertical_sampling,
		max_disparity_percent,
		correlation_radius,
		smoothing_radius,
		disparity_step,
		disparity_threshold_percent,
		despeckle,
		cross_checking_threshold,
		disparity_space,
		disparity_map);

}

/*!
 * \brief shows the strereo images
 * \param img_unwarped unwarped colour image
 * \param img_width_unwarped width of the unwarped image
 * \param img_height_unwarped height of the unwarped image
 * \param offset_y_unwarped vertical calibration adjustment
 * \param img_left left image for conventional stereo
 * \param img_right right image for conventional stereo
 * \param img_width width of the conventional stereo image
 * \param img_height height of the conventional stereo image
 */
void stackedstereodense::show_stereo_images(
	unsigned char* img_unwarped,
	int img_width_unwarped,
	int img_height_unwarped,
	int offset_y_unwarped,
	unsigned char* img_left,
	unsigned char* img_right,
	int img_width,
	int img_height)
{
	// create a pair of conventional stereo images
	unwarped_to_stereo_images(
		img_unwarped,
		img_width_unwarped,
		img_height_unwarped,
		offset_y_unwarped,
		img_left,
		img_right,
		img_width,
		img_height);

	memset((void*)img_unwarped, '\0', img_width_unwarped*img_height_unwarped*3);

    for (int mirror = 0; mirror < 2; mirror++) {
    	int tx = 0;
    	int bx = img_width_unwarped/2;
    	unsigned char* img = img_left;
    	if (mirror > 0) {
    		tx = bx;
    		bx = img_width_unwarped;
    		img = img_right;
    	}
    	for (int y = 0; y < img_height_unwarped; y++) {
    		int yy = y * (img_height-1) / img_height_unwarped;
    		for (int x = tx; x < bx; x++) {
    			int xx = (x - tx) * (img_width-1) / (bx - tx);
    			int n = (yy*img_width + xx)*3;
    			int n_unwarped = (y*img_width_unwarped + x)*3;
    			img_unwarped[n_unwarped] = img[n];
    			img_unwarped[n_unwarped + 1] = img[n + 1];
    			img_unwarped[n_unwarped + 2] = img[n + 2];
    		}
    	}
    }
}
