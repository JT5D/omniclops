/*
    omnidirectional vision
    Copyright (C) 2009 Bob Mottram
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

#ifndef OMNI_H_
#define OMNI_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <fstream>
#include "drawing.h"

#define OMNI_MAX_FEATURES           4000
#define OMNI_MAX_IMAGE_WIDTH        1024
#define OMNI_MAX_IMAGE_HEIGHT       1024
#define OMNI_VERTICAL_SAMPLING      4
#define OMNI_HORIZONTAL_SAMPLING    4
#define OMNI_SUB_PIXEL              32
#define OMNI_GROUND_MAP_RADIUS      500
#define OMNI_GROUND_MAP_COMPRESS    2

#define pixindex(xx, yy)  ((yy * imgWidth + xx) * 3)

class omni {
public:
    unsigned int imgWidth, imgHeight;

    unsigned char* feature_radius_index;

    /* array storing x coordinates of detected features */
    short int* feature_x;

    /* array storing y coordinates of detected features */
    short int* feature_y;

    /* array storing the number of features detected on each row */
    unsigned short int* features_per_row;

    /* array storing the number of features detected on each column */
    unsigned short int* features_per_col;

    /* buffer which stores sliding sum */
    int* row_sum;

    /* buffer used to find peaks in edge space */
    unsigned int* row_peaks;
    unsigned int* temp_row_peaks;

    /* map used for image rectification */
    int* calibration_map;

    /* maps raw image pixels to 3D rays */
    int* ray_map;

    /* radial lines */
    int no_of_radial_lines;
    int* radial_lines;
    int prev_no_of_radial_lines;
    int* prev_radial_lines;

    unsigned int av_peaks;
    int* unwarp_lookup;
    int* unwarp_lookup_reverse;

    int epipole;

    static bool intersection(
        float x0,
        float y0,
        float x1,
        float y1,
        float x2,
        float y2,
        float x3,
        float y3,
        float& xi,
        float& yi);
    int update_sums(int cols, int y, unsigned char* rectified_frame_buf);
    void non_max(int cols, int inhibition_radius, unsigned int min_response);
    int get_features_horizontal(unsigned char* rectified_frame_buf, int inhibition_radius, unsigned int minimum_response, int calibration_offset_x, int calibration_offset_y, int outer_radius_percent, int inner_radius_percent);
    int get_features_vertical(unsigned char* rectified_frame_buf, int inhibition_radius, unsigned int minimum_response, int calibration_offset_x, int calibration_offset_y, int outer_radius_percent, int inner_radius_percent);

    void make_map_int(long centre_of_distortion_x, long centre_of_distortion_y, long* coeff, long scale_num, long scale_denom);
    void rectify(unsigned char* raw_image, unsigned char* rectified_frame_buf);
    void flip(unsigned char* raw_image, unsigned char* flipped_frame_buf);
    void remove(
    	unsigned char* img,
    	int img_width,
    	int img_height,
    	int bytes_per_pixel,
	int centre_x,
	int centre_y,
    	float outer_radius_percent,
    	float inner_radius_percent,
	float inner_aspect,
	int no_of_struts,
	float * strut_angles,
	float strut_width);

    bool FileExists(std::string filename);
    void save_edges(std::string filename, int no_of_feats_vertical, int no_of_feats_horizontal);
    void save_rays(std::string filename, int no_of_feats_vertical, int no_of_feats_horizontal);
    void save_radial_lines(std::string filename);
    void get_calibration_image(
    	unsigned char* img,
    	int img_width,
    	int img_height);
    void calibrate(
        unsigned char* img,
        int img_width,
        int img_height,
        int bytes_per_pixel,
        int inner_radius_percent,
        int outer_radius_percent,
        std::string direction);

    unsigned char* img_buffer;

    void create_ray_map(
    	float mirror_diameter_mm,
    	float dist_to_mirror_mm,
    	float focal_length_mm,
    	float inner_radius_percent,
    	float outer_radius_percent,
    	float camera_height_mm,
        int img_width,
        int img_height,
        bool clear_map,
        bool update_unwarp);
    void create_ray_map(
    	float mirror_diameter_mm,
    	float dist_to_mirror_mm,
    	float focal_length_mm,
    	float inner_radius_percent,
    	float outer_radius_percent,
    	float camera_height_mm,
        int img_width,
        int img_height);
    void show_ray_map_side(
    	unsigned char* img,
    	int img_width,
    	int img_height,
    	int max_height_mm,
    	int focal_length_mm,
    	int camera_height_mm);
    void show_ground_plane(
        unsigned char* img,
        int img_width,
        int img_height,
        int max_radius_mm);
    void show_ground_plane_features(
        unsigned char* img,
        int img_width,
        int img_height,
        int max_radius_mm,
    	int no_of_feats_vertical,
    	int no_of_feats_horizontal);
    void detect_radial_lines(
        unsigned char* img,
        int img_width,
        int img_height,
        int max_radius_mm,
    	int no_of_feats_vertical,
    	int no_of_feats_horizontal,
    	int threshold);
    void show_radial_lines(
        unsigned char* img,
        int img_width,
        int img_height,
        int max_radius_mm);
    void unwarp(
    	unsigned char* img,
    	int img_width,
    	int img_height,
    	int bytes_per_pixel);
    void unwarp_features(
    	unsigned char* img,
    	int img_width,
    	int img_height,
    	int bytes_per_pixel,
    	int no_of_feats_vertical,
    	int no_of_feats_horizontal);

    void compass(int max_variance_degrees);

    omni(int width, int height);
    ~omni();
};

#endif

