/*
    used to detect the floor in an omnidirectional image
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

#include "detectfloor.h"

/*!
 * \brief returns the closest features to the centre of the mirror.  This is a bit hacky, and not recommended
 * \param features list of features in image coordinates
 * \param mirror_centre_x_pixels x coordinate of the centre of the mirror
 * \param mirror_centre_y_pixels y coordinate of the centre of the mirror
 * \param mirror_index index of the mirror
 * \param ray_map_width width of the image
 * \param ray_map_height height of the image
 * \param img colour image
 * \param mirror_map map containing mirror indexes
 * \param angular_increment_degrees bucketing increment in degrees
 * \param camera_base_width_mm
 * \param camera_base_height_mm
 * \param closest_features returned closest features
 */
void detectfloor::get_closest_features(
	vector<int> &features,
	int mirror_centre_x_pixels,
	int mirror_centre_y_pixels,
	int mirror_index,
	int ray_map_width,
	int ray_map_height,
	unsigned char* img,
	unsigned char* mirror_map,
	int angular_increment_degrees,
	int camera_base_width_pixels,
	int camera_base_height_pixels,
    vector<int> &closest_features,
    vector<vector<unsigned char> > &floor_colour)
{
	closest_features.clear();
	int max_angular_index = 360 / angular_increment_degrees;
	int closest[(max_angular_index+1)*2];
	int max_dist_sqr = 999999;
    for (int i = 0; i < max_angular_index+1; i++) {
    	closest[i*2] = max_dist_sqr;
    	closest[i*2+1] = -1;
    }

    //printf("%d %d\n", mirror_centre_x_pixels, mirror_centre_y_pixels);

	for (int f = 0; f < (int)features.size(); f += 2) {
		int fx = features[f];
		int fy = features[f+1];

		int n = fy*ray_map_width + fx;

		int mirror = mirror_map[n] - 1;
		if ((mirror > -1) &&
			((mirror == mirror_index) || (mirror_index == -1))) {

			int dx = fx - mirror_centre_x_pixels;
			int dy = fy - mirror_centre_y_pixels;
			if (!((dx > -camera_base_width_pixels) &&
				  (dx < camera_base_width_pixels) &&
				  (dy > -camera_base_height_pixels) &&
				  (dy < camera_base_height_pixels))) {
				int dist = dx*dx + dy*dy;
				int index = (int)(((atan2(dx, dy) / (3.1415927*2)) + 0.5) * max_angular_index);
				if (dist < closest[index*2]) {
					closest[index*2] = dist;
					closest[index*2+1] = f;
				}
			}
		}
	}

	int search_radius = 3;
	for (int i = 0; i < max_angular_index; i++) {
		int f = closest[i*2+1];
		if (f > -1) {
			int fx0 = features[f];
			int fy0 = features[f+1];
			closest_features.push_back(fx0);
			closest_features.push_back(fy0);


			for (int f2 = (int)features.size()-2; f2 >= 0; f2 -= 2) {
				if (f2 != f) {
					int fx1 = features[f2];
					if ((fx1 > fx0 - search_radius) && (fx1 < fx0 + search_radius)) {
						int fy1 = features[f2+1];
						if ((fy1 > fy0 - search_radius) && (fy1 < fy0 + search_radius)) {
							int j = 0;
							for (j = (int)closest_features.size()-2; j >= 0; j -= 2) {
								if ((closest_features[j] == fx1) && (closest_features[j+1] == fy1)) {
									break;
								}
							}
							if (j == 0) {
							    closest_features.push_back(fx1);
						        closest_features.push_back(fy1);
							}
						}
					}
				}
			}

		}
	}

    // update colour histogram
	int colour_histogram[256*3];
	memset((void*)colour_histogram,'\0',256*3*sizeof(int));
	for (int i = (int)closest_features.size()-2; i >= 0; i -= 2) {
		int fx = closest_features[i];
		int fy = closest_features[i+1];

		int dx = fx - mirror_centre_x_pixels;
		int dy = fy - mirror_centre_y_pixels;
		int dist = (int)sqrt(dx*dx + dy*dy);
		for (int d = 0; d < dist; d++) {
			int x = mirror_centre_x_pixels + (dx * d / dist);
			int y = mirror_centre_y_pixels + (dy * d / dist);

			int n = (y*ray_map_width + x)*3;
			if (!((x > mirror_centre_x_pixels-camera_base_width_pixels) &&
				  (x < mirror_centre_x_pixels+camera_base_width_pixels) &&
				  (y > mirror_centre_x_pixels-camera_base_height_pixels) &&
				  (y < mirror_centre_x_pixels+camera_base_height_pixels))) {
				colour_histogram[img[n]]++;
				colour_histogram[256 + img[n+1]]++;
				colour_histogram[(256*2) + img[n+2]]++;
				n += 3;
				colour_histogram[img[n]]++;
				colour_histogram[256 + img[n+1]]++;
				colour_histogram[(256*2) + img[n+2]]++;
				n -= 6;
				colour_histogram[img[n]]++;
				colour_histogram[256 + img[n+1]]++;
				colour_histogram[(256*2) + img[n+2]]++;
			}
			else {
				//colour_histogram[img[n]]--;
				//colour_histogram[256 + img[n+1]]--;
				//colour_histogram[(256*2) + img[n+2]]--;
			}
		}
	}
    for (int col = 0; col < 3; col++) {
    	floor_colour[col].clear();
    	int max = 1;
    	for (int i = 0; i < 256; i++) {
    		if (colour_histogram[(col*256)+i] > max) max = colour_histogram[(col*256)+i];
    	}
    	int thresh = max/10;
    	for (int i = 0; i < 256; i++) {
    		if (colour_histogram[(col*256)+i] > thresh) {
    			floor_colour[col].push_back((unsigned char)i);
    		}
    	}
    }
}

/*!
 * \brief detect features close to the floor
 * \param features list of image features
 * \param no_of_mirrors number of mirrors
 * \param ray_map_width width of the image
 * \param ray_map_height height of the image
 * \param floor_height_mm height of the floor
 * \param focal_length_mm focal length of the camera
 * \param camera_to_backing_dist_mm distance between the camera and the mirror backing
 * \param camera_height_mm height of the camera above the ground
 * \param img colour image data
 * \param ray_map lookup table containing ray vectors
 * \param mirror_map lookup table containing mirror indexes
 * \param feature_map buffer used to store feature indexes
 * \param ground_plane_tollerance_mm tollerance for matching features on the ground plane
 * \param image_plane_tollerance_pixels tollerance for matching features on the image plane
 * \param max_range_mm maximum ranging distance
 * \param camera_tx bounding box for the camera
 * \param camera_ty bounding box for the camera
 * \param camera_bx bounding box for the camera
 * \param camera_by bounding box for the camera
 * \param floor_features returned features close to the floor
 */
void detectfloor::detect(
	vector<int> &features,
	int no_of_mirrors,
	int ray_map_width,
	int ray_map_height,
	int floor_height_mm,
	float focal_length_mm,
	int camera_to_backing_dist_mm,
	int camera_height_mm,
	unsigned char* img,
	int* ray_map,
	unsigned char* mirror_map,
	unsigned char* feature_map,
    int ground_plane_tollerance_mm,
    int image_plane_tollerance_pixels,
    int max_range_mm,
    int camera_tx,
    int camera_ty,
    int camera_bx,
    int camera_by,
    vector<int> &floor_features)
{
	floor_features.clear();

	for (int f = (int)features.size()-2; f >= 0; f -= 2) {
	    int fx = features[f];
	    int fy = features[f+1];
	    int n = fy*ray_map_width + fx;

	    if (!((fx > camera_tx) && (fx < camera_bx) && (fy > camera_ty) && (fy < camera_by))) {
			if (mirror_map[n] == no_of_mirrors) {
				if ((mirror_map[n-10] != no_of_mirrors) ||
					(mirror_map[n+10] != no_of_mirrors) ||
					(mirror_map[n-(ray_map_width*10)] != no_of_mirrors) ||
					(mirror_map[n+(ray_map_width*10)] != no_of_mirrors)) {
					features.erase(features.begin()+f);
					features.erase(features.begin()+f);
				}
			}
	    }
	}

	// project features from centre mirror to the floor plane
	vector<int> projected_features;
    omni::project_features(
    	features,
    	no_of_mirrors-1,
    	floor_height_mm,
    	focal_length_mm,
    	camera_to_backing_dist_mm,
    	camera_height_mm,
    	ray_map_width,
    	ray_map_height,
    	ray_map,
    	mirror_map,
    	max_range_mm,
        projected_features);

    //reproject floor plane features back into the image plane
    vector<int> reprojected_features;
    omni::reproject_features(
    	projected_features,
    	-1,
    	floor_height_mm,
    	focal_length_mm,
    	camera_to_backing_dist_mm,
    	camera_height_mm,
    	ray_map,
    	ray_map_width,
    	ray_map_height,
        mirror_map,
        ground_plane_tollerance_mm,
        max_range_mm,
        reprojected_features);

    int w = ray_map_width/image_plane_tollerance_pixels;
    int h = ray_map_height/image_plane_tollerance_pixels;
    memset((void*)feature_map, '\0', w*h);
    for (int f1 = (int)reprojected_features.size()-2; f1 >= 0; f1 -= 2) {
    	int fx1 = reprojected_features[f1]/image_plane_tollerance_pixels;
    	int fy1 = reprojected_features[f1 + 1]/image_plane_tollerance_pixels;
    	int n2 = fy1 * w + fx1;
    	feature_map[n2] = 1;
    }

    // find matching features in peripheral mirrors
    for (int f0 = (int)features.size()-2; f0 >= 0; f0 -= 2) {
    	int fx0 = features[f0];
    	int fy0 = features[f0 + 1];
    	int n = fy0 * ray_map_width + fx0;
    	if ((mirror_map[n] > 0) && (mirror_map[n] != no_of_mirrors)) {
    		int fx1 = fx0 / image_plane_tollerance_pixels;
    		int fy1 = fy0 / image_plane_tollerance_pixels;
    		int n2 = fy1 * w + fx1;
    		if ((n2 > -1) && (n2 < w*h)) {
    		    if (feature_map[n2] != 0) {
    			    floor_features.push_back(fx0);
    			    floor_features.push_back(fy0);
    		    }
    		}
    	}
    }

	// project floor features from peripheral mirrors to the floor plane
    omni::project_features(
    	floor_features,
    	-1,
    	floor_height_mm,
    	focal_length_mm,
    	camera_to_backing_dist_mm,
    	camera_height_mm,
    	ray_map_width,
    	ray_map_height,
    	ray_map,
    	mirror_map,
    	max_range_mm,
        projected_features);

    //reproject floor plane features back into the centre mirror image plane
    omni::reproject_features(
    	projected_features,
    	no_of_mirrors-1,
    	floor_height_mm,
    	focal_length_mm,
    	camera_to_backing_dist_mm,
    	camera_height_mm,
    	ray_map,
    	ray_map_width,
    	ray_map_height,
        mirror_map,
        ground_plane_tollerance_mm,
        max_range_mm,
        reprojected_features);

    floor_features.clear();

    memset((void*)feature_map, '\0', w*h);
    for (int f1 = (int)reprojected_features.size()-2; f1 >= 0; f1 -= 2) {
    	int fx1 = reprojected_features[f1]/image_plane_tollerance_pixels;
    	int fy1 = reprojected_features[f1 + 1]/image_plane_tollerance_pixels;
    	int n2 = fy1 * w + fx1;
    	feature_map[n2] = 1;
    }

    // find matching features in centre mirror
    for (int f0 = (int)features.size()-2; f0 >= 0; f0 -= 2) {
    	int fx0 = features[f0];
    	int fy0 = features[f0 + 1];

    	if (!((fx0 > camera_tx) && (fx0 < camera_bx) &&
    		(fy0 > camera_ty) && (fy0 < camera_by))) {

			int n = fy0 * ray_map_width + fx0;
			if (mirror_map[n] == no_of_mirrors) {
				int fx1 = fx0 / image_plane_tollerance_pixels;
				int fy1 = fy0 / image_plane_tollerance_pixels;
				int n2 = fy1 * w + fx1;
				if ((n2 > -1) && (n2 < w*h)) {
					if (feature_map[n2] != 0) {
						floor_features.push_back(fx0);
						floor_features.push_back(fy0);
					}
				}
			}

    	}
    }

}
