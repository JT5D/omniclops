/*
 * detectfloor.cpp
 *
 *  Created on: 15-Feb-2010
 *      Author: motters
 */

#include "detectfloor.h"

/*!
 * \brief returns the closest features to the centre of the mirror
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

void detectfloor::find_floor_using_colour(
	vector<int> &features,
	vector<vector<unsigned char> > &floor_colour,
	int mirror_index,
	int ray_map_width,
	int ray_map_height,
	unsigned char* img,
	unsigned char* mirror_map,
	bool update_image,
	int r, int g, int b,
	int tollerance,
    vector<int> &closest_features)
{
	closest_features.clear();
	bool matched;
	int n = 0;
	for (int y = 0; y < ray_map_height; y++) {
		for (int x = 0; x < ray_map_width; x++, n++) {
			int mirror = mirror_map[n] - 1;
			if (mirror == mirror_index) {
				for (int col = 0; col < 3; col++) {
					matched = false;
					for (int i = (int)floor_colour[col].size()-1; i >= 0; i--) {
				        int diff = (int)img[n*3 + col] - (int)floor_colour[col][i];
				        if ((diff >= -tollerance) && (diff <= tollerance)) {
				        	matched = true;
				        	break;
				        }
					}
					if (!matched) {
                        break;
					}
					else {
						if (col == 2) {
							if (update_image) {
								img[n*3] = b;
								img[n*3 + 1] = g;
								img[n*3 + 2] = r;
							}
						}
					}
				}
			}
		}
	}
}

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
    int ground_plane_tollerance_mm,
    int image_plane_tollerance_pixels,
    int max_range_mm,
    vector<int> &floor_features)
{
	floor_features.clear();

	for (int f = (int)features.size()-2; f >= 0; f -= 2) {
	    int fx = features[f];
	    int fy = features[f+1];
	    int n = fy*ray_map_width + fx;

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

	// project features from one mirror to the floor plane
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
        //floor_features);
        reprojected_features);

    // find matching features
    for (int f0 = (int)features.size()-2; f0 >= 0; f0 -= 2) {
    	int fx0 = features[f0];
    	int fy0 = features[f0 + 1];
    	int n = fy0 * ray_map_width + fx0;
    	if ((mirror_map[n] > 0) && (mirror_map[n] != no_of_mirrors)) {
    	    for (int f1 = (int)reprojected_features.size()-2; f1 >= 0; f1 -= 2) {
    	    	int fx1 = reprojected_features[f1];
    	    	int fy1 = reprojected_features[f1 + 1];
    	    	int n2 = fy1 * ray_map_width + fx1;
    	    	if (mirror_map[n2] == mirror_map[n]) {
    	    		int dx = fx1 - fx0;
    	    		if ((dx > -image_plane_tollerance_pixels) && (dx < image_plane_tollerance_pixels)) {
        	    		int dy = fy1 - fy0;
        	    		if ((dy > -image_plane_tollerance_pixels) && (dy < image_plane_tollerance_pixels)) {
        	    			floor_features.push_back(fx0);
        	    			floor_features.push_back(fy0);
        	    			break;
        	    		}
    	    		}
    	    	}
    	    }
    	}
    }

	// project floor features from other mirrors to the floor plane
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

    //reproject floor plane features back into the single mirror image plane
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

    // find matching features in the original mirror
	int z_mm = (camera_to_backing_dist_mm + focal_length_mm) - (floor_height_mm + focal_length_mm - camera_height_mm);
	float z_fraction = z_mm / ((float)camera_to_backing_dist_mm + focal_length_mm);

    for (int f0 = (int)features.size()-2; f0 >= 0; f0 -= 2) {
    	int fx0 = features[f0];
    	int fy0 = features[f0 + 1];
    	int n = fy0 * ray_map_width + fx0;
    	if ((mirror_map[n] == no_of_mirrors) &&
    		(ray_map[n*6 + 5] < ray_map[n*6 + 2]-20)) {
    	    for (int f1 = (int)reprojected_features.size()-2; f1 >= 0; f1 -= 2) {
    	    	int fx1 = reprojected_features[f1];
    	    	int fy1 = reprojected_features[f1 + 1];
    	    	int n2 = fy1 * ray_map_width + fx1;
    	    	if ((mirror_map[n2] == no_of_mirrors) &&
    	    	    (ray_map[n2*6 + 5] < ray_map[n2*6 + 2]-20)) {
    	    		int dx = fx1 - fx0;
    	    		if ((dx > -image_plane_tollerance_pixels) && (dx < image_plane_tollerance_pixels)) {
        	    		int dy = fy1 - fy0;
        	    		if ((dy > -image_plane_tollerance_pixels) && (dy < image_plane_tollerance_pixels)) {

        					int start_x_mm = ray_map[n*6];
        					int start_y_mm = ray_map[n*6 + 1];
        					int end_x_mm = ray_map[n*6 + 3];
        					int end_y_mm = ray_map[n*6 + 4];

        					int ray_x_mm = start_x_mm + (int)((end_x_mm - start_x_mm) * z_fraction);
        					if ((ray_x_mm > -max_range_mm) && (ray_x_mm < max_range_mm)) {
        						int ray_y_mm = start_y_mm + (int)((end_y_mm - start_y_mm) * z_fraction);
        						if ((ray_y_mm > -max_range_mm) && (ray_y_mm < max_range_mm)) {


        							if ((mirror_map[n-10] == no_of_mirrors) &&
        							    (mirror_map[n+10] == no_of_mirrors) &&
        							    (mirror_map[n-(ray_map_width*10)] == no_of_mirrors) &&
        							    (mirror_map[n+(ray_map_width*10)] == no_of_mirrors)) {

        	    			            floor_features.push_back(fx0);
        	    			            floor_features.push_back(fy0);
        	    			            break;
        							}
        						}
        					}

        	    		}
    	    		}
    	    	}
    	    }
    	}
    }
}
