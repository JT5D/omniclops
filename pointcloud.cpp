/*
 * pointcloud.cpp
 *
 *  Created on: 18-Feb-2010
 *      Author: motters
 */

#include "pointcloud.h"

void pointcloud::get_feature_heights(
	unsigned char* img,
    vector<int> &features,
    float camera_height_mm,
    float camera_to_mirror_backing_dist_mm,
    float focal_length,
    float* mirror_position_pixels,
    float outer_radius_percent,
    int no_of_mirrors,
    int ray_map_width,
    int ray_map_height,
    int max_height_mm,
    int height_step_mm,
    int max_range_mm,
    int camera_width_percent,
    int camera_height_percent,
    int ground_plane_tollerance_mm,
    int* ray_map,
    unsigned char* mirror_map,
    unsigned char* feature_map,
    bool show_features,
    vector<int> &feature_heights)
{
	feature_heights.clear();
	//ground_plane_tollerance_mm = 64;
	int image_plane_tollerance_pixels = 1;
	int camera_width_pixels = (int)((outer_radius_percent * ray_map_width / 200) * camera_width_percent/100); //45
	int camera_height_pixels = (int)((outer_radius_percent * ray_map_width / 200) * camera_height_percent/100); //30
	int x_offset = -5;
	int y_offset = 0;
	int camera_tx = (mirror_position_pixels[(no_of_mirrors-1)*2]*ray_map_width/100) - (camera_width_pixels/2) + x_offset;
	int camera_ty = (mirror_position_pixels[(no_of_mirrors-1)*2+1]*ray_map_height/100) - (camera_height_pixels/2) + y_offset;
	int camera_bx = camera_tx + camera_width_pixels;
	int camera_by = camera_ty + camera_height_pixels;
	vector<int> plane_features;
	for (int plane_height_mm = 0; plane_height_mm <= max_height_mm; plane_height_mm += height_step_mm) {

		// detect features at this height
		plane_features.clear();
		detectfloor::detect(
			features,
			no_of_mirrors,
			ray_map_width,ray_map_height,
			plane_height_mm,
			focal_length,
			(int)camera_to_mirror_backing_dist_mm,
			(int)camera_height_mm,
			img,
			ray_map,
			mirror_map,
			feature_map,
			ground_plane_tollerance_mm,
			image_plane_tollerance_pixels,
			max_range_mm,
			camera_tx,
			camera_ty,
			camera_bx,
			camera_by,
			plane_features);

		// add features for this plane
		for (int i = (int)plane_features.size()-2; i >= 0; i -= 2) {
			int x = plane_features[i];
			int y = plane_features[i + 1];

			//bool found = false;
			//for (int j = (int)feature_heights.size()-3; j >= 0; j -= 3) {
				//if ((feature_heights[j] == x) && (feature_heights[j+1] == y)) {
					//feature_heights[j+2] += (plane_height_mm - feature_heights[j+2])/64;
					//found = true;
					//break;
				//}
			//}
			//if (!found) {
			    feature_heights.push_back(x);
			    feature_heights.push_back(y);
			    feature_heights.push_back(plane_height_mm);
			//}

			// remove this feature from subsequent searching

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
