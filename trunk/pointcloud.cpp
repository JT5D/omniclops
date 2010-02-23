/*
    used to create a point cloud for observed features
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

#include "pointcloud.h"

/*!
 * \brief project the given image features into cartesian coordinates
 * \param features list of features plus height values in image coordinates (px,py,height_mm)
 * \param mirror_index index number of the mirror from which features will be projected
 * \param plane_height_mm height that features are to be projected to
 * \param focal_length_mm focal length of the camera
 * \param camera_to_backing_dist_mm distance between the camera and the mirror backing plane
 * \param camera_height_mm height of the camera above the ground
 * \param ray_map_width width of the camera image
 * \param ray_map_height height of the camera image
 * \param ray_map lookup table containing ray vectors
 * \param mirror_map map containing mirror indexes
 * \param max_range_mm maximum range in mm
 * \param point_cloud returned projected features (x,y,z)
 */
/*
void pointcloud::features_to_point_cloud(
	vector<int> &features,
	int mirror_index,
	float focal_length_mm,
	int camera_to_backing_dist_mm,
	int camera_height_mm,
	int ray_map_width,
	int ray_map_height,
	int* ray_map,
	unsigned char* mirror_map,
	int max_range_mm,
    vector<int> &point_cloud)
{
	point_cloud.clear();

	for (int f = 0; f < (int)features.size(); f += 3) {
		int fx = features[f];
		int fy = features[f+1];

		int n = fy*ray_map_width + fx;

		int mirror = mirror_map[n] - 1;
		if ((mirror > -1) &&
			((mirror == mirror_index) || (mirror_index == -1))) {
			if ((ray_map[n*6 + 2] != 0) &&
				(ray_map[n*6 + 5] < ray_map[n*6 + 2])) {

				int plane_height_mm = features[f+2];
				float z_fraction = omni::get_z_fraction(camera_height_mm, camera_to_backing_dist_mm, focal_length_mm, plane_height_mm);

				int start_x_mm = ray_map[n*6];
				int start_y_mm = ray_map[n*6 + 1];
				int end_x_mm = ray_map[n*6 + 3];
				int end_y_mm = ray_map[n*6 + 4];

				int ray_x_mm = start_x_mm + (int)((end_x_mm - start_x_mm) * z_fraction);
				int ray_y_mm = start_y_mm + (int)((end_y_mm - start_y_mm) * z_fraction);

				if ((ray_x_mm > -max_range_mm) && (ray_x_mm < max_range_mm) &&
					(ray_y_mm > -max_range_mm) && (ray_y_mm < max_range_mm)) {
					point_cloud.push_back(ray_x_mm);
					point_cloud.push_back(ray_y_mm);
					point_cloud.push_back(plane_height_mm);
				}

			}
		}
	}
}
*/

void pointcloud::get_feature_heights(
	unsigned char* img,
    vector<int> &features,
    float camera_height_mm,
    float camera_to_mirror_backing_dist_mm,
    float focal_length,
    float* mirror_position_pixels,
    float outer_radius_percent,
    float mirror_diameter_mm,
    int no_of_mirrors,
    int ray_map_width,
    int ray_map_height,
    int max_height_mm,
    int height_step_mm,
    int max_range_mm,
    int camera_width_percent,
    int camera_height_percent,
    int* ray_map,
    unsigned char* mirror_map,
    unsigned short* feature_map,
    unsigned short* ground_features_lookup,
    bool show_features,
    vector<int> &feature_heights,
    vector<int> &point_cloud)
{
	feature_heights.clear();
	point_cloud.clear();
	int image_plane_tollerance_pixels = 1;
	int camera_width_pixels = (int)((outer_radius_percent * ray_map_width / 200) * camera_width_percent/100); //45
	int camera_height_pixels = (int)((outer_radius_percent * ray_map_width / 200) * camera_height_percent/100); //30
	int x_offset = -5;
	int y_offset = 0;
	int camera_tx = (mirror_position_pixels[(no_of_mirrors-1)*2]*ray_map_width/100) - (camera_width_pixels/2) + x_offset;
	int camera_ty = (mirror_position_pixels[(no_of_mirrors-1)*2+1]*ray_map_height/100) - (camera_height_pixels/2) + y_offset;
	int camera_bx = camera_tx + camera_width_pixels;
	int camera_by = camera_ty + camera_height_pixels;

	// pixel size on the mirror backing plane
	float pixel_diameter_mirror_plane = mirror_diameter_mm / (outer_radius_percent * ray_map_width / 200.0f);

	// pixel diameter on the focal plane
	float pixel_diameter_mm = pixel_diameter_mirror_plane*focal_length*16 / (camera_to_mirror_backing_dist_mm+focal_length);

	// remove features around the area of the camera - we don't need to project these
	for (int i = (int)features.size()-2; i >= 0; i -= 2) {
		if ((features[i] > camera_tx) && (features[i + 1] > camera_ty) &&
			(features[i] < camera_bx) && (features[i + 1] < camera_by)) {
			features.erase(features.begin() + i);
			features.erase(features.begin() + i);
		}
	}

	// tollerance for features above or below the plane
	int plane_tollerance_mm = height_step_mm/2;

	vector<int> plane_features;
	vector<int> plane_features_positions;
	for (int plane_height_mm = 0; plane_height_mm <= max_height_mm; plane_height_mm += height_step_mm) {

		// approximate size of one pixel on this plane
		int ground_plane_tollerance_mm = (int)(pixel_diameter_mm * ((camera_height_mm - plane_height_mm) + (2*camera_to_mirror_backing_dist_mm)));
		if (ground_plane_tollerance_mm < 8) ground_plane_tollerance_mm = 8;

		// detect features at this height
		plane_features.clear();
		detectfloor::detect(
			features,
			no_of_mirrors,
			ray_map_width,ray_map_height,
			plane_height_mm,
			plane_tollerance_mm,
			focal_length,
			(int)camera_to_mirror_backing_dist_mm,
			(int)camera_height_mm,
			img,
			ray_map,
			mirror_map,
			feature_map,
			ground_features_lookup,
			ground_plane_tollerance_mm,
			image_plane_tollerance_pixels,
			max_range_mm,
			camera_tx,
			camera_ty,
			camera_bx,
			camera_by,
			plane_features_positions,
			plane_features);

		// add points to the cloud
		for (int i = (int)plane_features_positions.size()-3; i >= 0; i -= 3) {
			point_cloud.push_back(plane_features_positions[i]);
			point_cloud.push_back(plane_features_positions[i+1]);
			point_cloud.push_back(plane_features_positions[i+2]);
		}

		// add features for this plane
		for (int i = (int)plane_features.size()-2; i >= 0; i -= 2) {
			int x = plane_features[i];
			int y = plane_features[i + 1];

		    feature_heights.push_back(x);
	        feature_heights.push_back(y);
			feature_heights.push_back(plane_features_positions[(i/2)*3+2]);

			// remove this feature from further inquiries
			for (int j = (int)features.size()-2; j >= 0; j -= 2) {
				if ((features[j] == x) && (features[j + 1] == y)) {
					features.erase(features.begin() + j);
					features.erase(features.begin() + j);
					break;
				}
			}
		}

	}

	if (show_features) {
	    for (int j = (int)feature_heights.size()-3; j >= 0; j -= 3) {
	    	int x = feature_heights[j];
	    	int y = feature_heights[j+1];
	    	int plane_height_mm = feature_heights[j+2];
			int r = plane_height_mm * 255  / max_height_mm;
			int b = (max_height_mm - plane_height_mm) * 255  / max_height_mm;
			drawing::drawCross(img, ray_map_width, ray_map_height, x, y, 2, r, 0, b, 0);
	    }
	}
}

void pointcloud::show(
	unsigned char *img,
	int width,
	int height,
	int max_range_mm,
	int max_height_mm,
	vector<int> &point_cloud,
	int view_type)
{
	memset((void*)img, '\0', width*height*3);

	for (int i = (int)point_cloud.size()-3; i >= 0; i -= 3) {
		int x = point_cloud[i];
		int y = point_cloud[i+1];
		int z = point_cloud[i+2];

		if ((x > -max_range_mm) && (x < max_range_mm) &&
			(y > -max_range_mm) && (y < max_range_mm) &&
			(z > -1) && (z < max_height_mm)) {

			switch(view_type) {
			case 0: {
                x = (x + max_range_mm) * height / (max_range_mm*2);
                y = (y + max_range_mm) * (height-1) / (max_range_mm*2);
                drawing::drawCross(img, width, height, x, y, 2, 0, 255, 0, 0);
				break;
			}
			case 1: {
                x = (x + max_range_mm) * height / (max_range_mm*2);
                z = height - 1 - (z * height / max_height_mm);
                drawing::drawCross(img, width, height, x, z, 2, 0, 255, 0, 0);
				break;
			}
			case 2: {
                y = (y + max_range_mm) * height / (max_range_mm*2);
                z = height - 1 - (z * height / max_height_mm);
                drawing::drawCross(img, width, height, y, z, 2, 0, 255, 0, 0);
				break;
			}
			case 3: {
                int x2 = ((x + max_range_mm) * height / (max_range_mm*2)) / 2;
                int y2 = ((y + max_range_mm) * (height-1) / (max_range_mm*2)) / 2;
                int z2 = (height - 1 - (z * height / max_height_mm)) / 2;
                drawing::drawCross(img, width, height, x2, y2 + (height/4), 2, 0, 255, 0, 0);
                drawing::drawCross(img, width, height, x2 + (width/2), z2, 2, 255, 255, 0, 0);
                drawing::drawCross(img, width, height, y2 + (width/2), z2 + (height/2), 2, 0, 255, 255, 0);
				break;
			}
			}
		}
	}
}


void pointcloud::update(
	unsigned char* img,
    vector<int> &features,
    float camera_height_mm,
    float camera_to_mirror_backing_dist_mm,
    float focal_length_mm,
    float* mirror_position_pixels,
    float outer_radius_percent,
    float mirror_diameter_mm,
    int no_of_mirrors,
    int ray_map_width,
    int ray_map_height,
    int max_height_mm,
    int height_step_mm,
    int max_range_mm,
    int camera_width_percent,
    int camera_height_percent,
    int* ray_map,
    unsigned char* mirror_map,
    unsigned short* feature_map,
    unsigned short* ground_features_lookup,
    int view_type,
    vector<int> &point_cloud)
{
	bool show_features = false;
	if (view_type == 1) show_features = true;

	vector<int> feature_heights;
	get_feature_heights(
		img,
	    features,
	    camera_height_mm,
	    camera_to_mirror_backing_dist_mm,
	    focal_length_mm,
	    mirror_position_pixels,
	    outer_radius_percent,
	    mirror_diameter_mm,
	    no_of_mirrors,
	    ray_map_width,
	    ray_map_height,
	    max_height_mm,
	    height_step_mm,
	    max_range_mm,
	    camera_width_percent,
	    camera_height_percent,
	    ray_map,
	    mirror_map,
	    feature_map,
	    ground_features_lookup,
	    show_features,
	    feature_heights,
	    point_cloud);

	/*
	features_to_point_cloud(
		feature_heights,
		no_of_mirrors-1,
		focal_length_mm,
		camera_to_mirror_backing_dist_mm,
		camera_height_mm,
		ray_map_width,
		ray_map_height,
		ray_map,
		mirror_map,
		max_range_mm,
	    point_cloud);
*/
	if (view_type > 1) {
		max_range_mm = 3000;
		show(
			img,
			ray_map_width,
			ray_map_height,
			max_range_mm,
			max_height_mm,
			point_cloud,
			view_type-2);
	}
}
