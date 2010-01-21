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
#include <vector>
#include <fstream>
#include <algorithm>
#include "drawing.h"
using namespace std;


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
protected:
	void rgb_to_hsv(
	    int r, int g, int b,
	    unsigned char& h,
	    unsigned char& s,
	    unsigned char& v);
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
    unsigned char* mirror_map; // mirror numbers
    float* mirror_lookup; // radii and angles

    /* radial lines */
    int no_of_radial_lines;
    int* radial_lines;
    int prev_no_of_radial_lines;
    int* prev_radial_lines;

    unsigned int av_peaks;
    int* unwarp_lookup;
    int* unwarp_lookup_reverse;

    int grid_centre_x_mm, grid_centre_y_mm, grid_centre_z_mm;
    int grid_cell_dimension_mm;
    int grid_dimension_cells;
    unsigned int* occupancy_grid_histograms;
    unsigned char* occupancy_grid;
    unsigned char* occupancy_grid_colour;
    vector<int> point_cloud;

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
    float intersection_ray_sphere(
        float ray_x0,
        float ray_y0,
        float ray_z0,
        float ray_x1,
        float ray_y1,
        float ray_z1,
        float sphere_x,
        float sphere_y,
        float sphere_z,
        float sphere_radius);

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
    	float outer_radius_percent,
    	float inner_radius_percent);

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
    	float mirror_diameter,
    	float dist_to_mirror_backing,
    	float focal_length,
    	float outer_radius_percent,
    	float camera_height_mm,
    	int no_of_mirrors,
    	float* mirror_position,
    	float* mirror_position_pixels,
        int img_width,
        int img_height);
    void show_ray_map_side(
    	unsigned char* img,
    	int img_width,
    	int img_height,
    	int max_height_mm,
    	int focal_length_mm,
    	int camera_height_mm,
    	bool show_all);
    void show_ray_map_above(
    	unsigned char* img,
    	int img_width,
    	int img_height,
    	int max_radius_mm);
    void show_ray_pixels(
    	unsigned char* img,
    	int img_width,
    	int img_height);
    void show_ray_directions(
    	unsigned char* img,
    	int img_width,
    	int img_height);
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
    void get_ray(
    	int img_width,
    	int pixel_x, int pixel_y,
    	int& ray_origin_x_mm,
    	int& ray_origin_y_mm,
    	int& ray_origin_z_mm,
    	int& ray_ground_x_mm,
    	int& ray_ground_y_mm,
    	int& ray_ground_z_mm);
    static bool save_configuration(
    	std::string filename,
    	int no_of_mirrors,
    	float* mirror_position_pixels,
    	float* mirror_position,
    	float focal_length,
    	float mirror_diameter,
    	float outer_radius_percent,
    	float inner_radius_percent,
    	float dist_to_mirror_centre,
    	float camera_height,
    	float baseline,
    	float range);
    static bool load_configuration(
    	std::string filename,
    	int& no_of_mirrors,
    	float* mirror_position_pixels,
    	float* mirror_position,
    	float &focal_length,
		float &mirror_diameter,
		float &outer_radius_percent,
		float &inner_radius_percent,
		float &dist_to_mirror_centre,
		float &camera_height,
		float &baseline,
		float &range);
    float min_distance_between_rays(
    	float ray1_x_start,
    	float ray1_y_start,
    	float ray1_z_start,
    	float ray1_x_end,
    	float ray1_y_end,
    	float ray1_z_end,
    	float ray2_x_start,
    	float ray2_y_start,
    	float ray2_z_start,
    	float ray2_x_end,
    	float ray2_y_end,
    	float ray2_z_end,
    	float &dx,
    	float &dy,
    	float &dz,
    	float &x,
    	float &y,
    	float &z);
    static void voxel_paint(
    	int* ray_map,
    	int dist_to_mirror_backing_mm,
    	float mirror_diameter_mm,
    	int no_of_mirrors,
    	unsigned char* mirror_map,
    	unsigned char* img,
    	unsigned char* img_occlusions,
        int img_width,
        int img_height,
    	int grid_cells_x,
    	int grid_cells_y,
    	int grid_cells_z,
    	int grid_cell_dimension_mm,
    	int grid_centre_x_mm,
    	int grid_centre_y_mm,
    	int grid_centre_z_mm,
    	int min_correlation,
    	vector<short> &occupied_voxels);
    static void show_voxels(
    	unsigned char* img,
    	int img_width,
    	int img_height,
    	vector<short> &voxels,
    	int voxel_radius_pixels,
    	int view_type);
    static void show_rays(
    	unsigned char* mirror_map,
    	int* ray_map,
    	unsigned char* img,
    	int img_width,
    	int img_height,
    	int radius_mm,
    	int point_radius_pixels);
    static void reproject(
    	unsigned char* ground_img,
    	int plane_height_mm,
    	float focal_length_mm,
    	int camera_to_backing_dist_mm,
    	int camera_height_mm,
    	int img_width,
    	int img_height,
    	int tx_mm,
    	int ty_mm,
    	int bx_mm,
    	int by_mm,
    	int* ray_map,
    	unsigned char* reprojected_img,
    	int ray_map_width,
    	int ray_map_height);
    static void project(
    	unsigned char* ray_map_img,
    	int plane_height_mm,
    	float focal_length_mm,
    	int camera_to_backing_dist_mm,
    	int camera_height_mm,
    	int ray_map_width,
    	int ray_map_height,
    	int tx_mm,
    	int ty_mm,
    	int bx_mm,
    	int by_mm,
    	int* ray_map,
    	unsigned char* projected_img,
    	int projected_img_width,
    	int projected_img_height,
    	int* colour_difference);
    static void create_obstacle(
    	int* ground_height_mm,
    	unsigned char* ground_img,
    	int ground_width,
    	int ground_height,
    	int tx_mm, int ty_mm,
    	int bx_mm, int by_mm,
    	int obstacle_tx_mm, int obstacle_ty_mm,
    	int obstacle_bx_mm, int obstacle_by_mm,
    	int height_mm,
    	int width_mm,
    	int r, int g, int b);
    static void create_ground_grid(
    	int* ground_height_mm,
    	unsigned char* ground_img,
    	int ground_width,
    	int ground_height,
    	int tx_mm, int ty_mm,
    	int bx_mm, int by_mm,
    	int grid_tx_mm, int grid_ty_mm,
    	int grid_bx_mm, int grid_by_mm,
    	int grid_dimension_mm,
    	int height_mm,
    	int line_width_mm,
    	int r, int g, int b);
    static void show_mirror_lookup(
    	unsigned char* img,
    	int img_width,
    	int img_height,
    	float* lookup,
    	bool show_radius,
    	float outer_radius_percent);

    static void create_ray_map_mirror_inner(
    	int centre_x_pixels,
    	int centre_y_pixels,
    	int centre_x_mm,
    	int centre_y_mm,
    	int radius_pixels,
    	int* ray_map,
    	float* mirror_lookup,
    	int ray_map_width,
    	int ray_map_height,
    	float tilt_radians,
    	float rotate_radians,
    	float mirror_diameter_mm,
    	float dist_to_mirror_centre_mm,
    	float camera_height_mm,
    	float centre_x,
    	float centre_y,
    	float centre_z,
    	float r,
    	bool negative);
    static void create_ray_map_mirror(
    	int centre_x_pixels,
    	int centre_y_pixels,
    	int centre_x_mm,
    	int centre_y_mm,
    	int radius_pixels,
    	int* ray_map,
    	float* mirror_lookup,
    	int ray_map_width,
    	int ray_map_height,
    	float tilt_radians,
    	float rotate_radians,
    	float mirror_diameter_mm,
    	float dist_to_mirror_backing_mm,
    	float camera_height_mm);
    static void create_ray_map(
    	float mirror_diameter_mm,
    	float dist_to_mirror_backing_mm,
    	float focal_length,
    	float outer_radius_percent,
    	float camera_height_mm,
    	int no_of_mirrors,
    	float* mirror_position,
    	float* mirror_position_pixels,
        int img_width,
        int img_height,
        int* ray_map,
        float* mirror_lookup);

    void compass(int max_variance_degrees);

    omni(int width, int height);
    ~omni();
};

#endif

