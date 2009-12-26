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

#include "omni.h"

omni::omni(int width, int height) {

	imgWidth = width;
	imgHeight = height;

	/* array storing x coordinates of detected features */
	feature_x = new short int[OMNI_MAX_FEATURES];
	feature_y = new short int[OMNI_MAX_FEATURES];

	ray_map = NULL;
	mirror_map = NULL;
	calibration_map = NULL;
	feature_radius_index = NULL;
	unwarp_lookup = NULL;
	unwarp_lookup_reverse = NULL;
	occupancy_grid = NULL;

	/* array storing the number of features detected on each row */
	features_per_row = new unsigned short int[OMNI_MAX_IMAGE_HEIGHT
			/ OMNI_VERTICAL_SAMPLING];
	features_per_col = new unsigned short int[OMNI_MAX_IMAGE_WIDTH
			/ OMNI_HORIZONTAL_SAMPLING];

	/* buffer which stores sliding sum */
	row_sum = new int[OMNI_MAX_IMAGE_WIDTH];

	/* buffer used to find peaks in edge space */
	row_peaks = new unsigned int[OMNI_MAX_IMAGE_WIDTH];
	temp_row_peaks = new unsigned int[OMNI_MAX_IMAGE_WIDTH];

	epipole=0;
	img_buffer = NULL;

	no_of_radial_lines = 0;
	radial_lines = NULL;
	prev_no_of_radial_lines = 0;
	prev_radial_lines = NULL;
}

omni::~omni() {
	delete[] feature_x;
	delete[] features_per_row;
	delete[] row_sum;
	delete[] row_peaks;
	if (unwarp_lookup != NULL) {
		delete[] unwarp_lookup;
		delete[] unwarp_lookup_reverse;
	}
	if (prev_radial_lines != NULL) {
		delete[] prev_radial_lines;
	}
	if (radial_lines != NULL) {
		delete[] radial_lines;
	}
	if (feature_radius_index != NULL) {
		delete[] feature_radius_index;
	}
	if (calibration_map != NULL) {
		delete[] calibration_map;
	}
	if (mirror_map != NULL) {
		delete[] mirror_map;
	}
	if (ray_map != NULL) {
		delete[] ray_map;
	}
	if (img_buffer != NULL)
		delete[] img_buffer;
	if (occupancy_grid != NULL) {
		delete[] occupancy_grid;
	}
}

/* Updates sliding sums and edge response values along a single row or column
 * Returns the mean luminance along the row or column */
int omni::update_sums(int cols, /* if non-zero we're dealing with columns not rows */
int i, /* row or column index */
unsigned char* rectified_frame_buf) /* image data */
{

	int j, x, y, idx, max, sum = 0, mean = 0;

	if (cols == 0) {
		/* compute sums along the row */
		y = i;
		idx = imgWidth * y * 3 + 2;
		max = (int) imgWidth;
		int w = max*3*2;

		row_sum[0] = rectified_frame_buf[idx];
		for (x = 1; x < max; x++, idx += 3) {
			sum += rectified_frame_buf[idx] +
			       rectified_frame_buf[idx - w] +
			       rectified_frame_buf[idx + w];
			row_sum[x] = sum;
		}
	} else {
		/* compute sums along the column */
		idx = i * 3 + 2;
		x = i;
		max = (int) imgHeight;
		int stride = (int) imgWidth * 3;

		row_sum[0] = rectified_frame_buf[idx];
		for (y = 1; y < max; y++, idx += stride) {
			sum += rectified_frame_buf[idx];
			row_sum[y] = sum;
		}
	}

	/* row mean luminance */
	mean = row_sum[max - 1] / (max * 2);

	/* compute peaks */
	int p0, p1, p2, p;
	av_peaks = 0;
	for (j = 6; j < max - 6; j++) {
		sum = row_sum[j];
		/* edge using 1 pixel radius */
		p0 = (sum - row_sum[j - 1]) - (row_sum[j + 1] - sum);
		if (p0 < 0)
			p0 = -p0;

		/* edge using 3 pixel radius */
		p1 = (sum - row_sum[j - 3]) - (row_sum[j + 3] - sum);
		if (p1 < 0)
			p1 = -p1;

		/* edge using 5 pixel radius */
		p2 = (sum - row_sum[j - 5]) - (row_sum[j + 5] - sum);
		if (p2 < 0)
			p2 = -p2;

		/* overall edge response */
		p = p0*8 + p1*2 + p2;
		row_peaks[j] = p;
		av_peaks += p;
	}
	av_peaks /= (max - 8);

	memcpy((void*)temp_row_peaks, (void*)row_peaks, max*sizeof(unsigned int));

	return (mean);
}

/* performs non-maximal suppression on the given row or column */
void omni::non_max(int cols, /* if non-zero we're dealing with columns not rows */
int inhibition_radius, /* radius for non-maximal suppression */
unsigned int min_response) { /* minimum threshold as a percent in the range 0-200 */

	int i, r, max, max2;
	unsigned int v;

	/* average response */
	unsigned int av_peaks = 0;
	max = (int) imgWidth;
	if (cols != 0)
		max = (int) imgHeight;
	max2 = max - inhibition_radius;
	max -= 4;
	for (i = 4; i < max; i++) {
		av_peaks += row_peaks[i];
	}

	/* adjust the threshold */
	av_peaks = av_peaks * min_response / (100 * (max - 4));

	for (i = 4; i < max2; i++) {
		if (row_peaks[i] < av_peaks)
			row_peaks[i] = 0;
		v = row_peaks[i];
		if (v > 0) {
			for (r = 1; r < inhibition_radius; r++) {
				if (row_peaks[i + r] < v) {
					row_peaks[i + r] = 0;
				} else {
					row_peaks[i] = 0;
					r = inhibition_radius;
				}
			}
		}
	}
}

/* returns a set of vertically oriented edge features */
int omni::get_features_vertical(unsigned char* rectified_frame_buf, /* image data */
int inhibition_radius, /* radius for non-maximal supression */
unsigned int minimum_response, /* minimum threshold */
int calibration_offset_x, /* calibration x offset in pixels */
int calibration_offset_y,/* calibration y offset in pixels */
int outer_radius_percent,
int inner_radius_percent)
{

	unsigned short int no_of_feats;
	int x, y, row_mean, start_x, prev_x, mid_x,r,dx,dy;
	int no_of_features = 0;
	int row_idx = 0;
	int cx = imgWidth/2;
	int cy = imgHeight/2;
	int max_radius = (outer_radius_percent * imgWidth / 200) - 2;
	int min_radius = (inner_radius_percent * imgWidth / 200) + 2;
	max_radius *= max_radius;
	min_radius *= min_radius;

	memset((void*) (features_per_row), '\0', OMNI_MAX_IMAGE_HEIGHT
			/ OMNI_VERTICAL_SAMPLING * sizeof(unsigned short));

	start_x = imgWidth - 15;
	if ((int) imgWidth - inhibition_radius - 1 < start_x)
		start_x = (int) imgWidth - inhibition_radius - 1;

	for (y = 4 + calibration_offset_y; y < (int) imgHeight - 4; y
			+= OMNI_VERTICAL_SAMPLING) {

		/* reset number of features on the row */
		no_of_feats = 0;

		if ((y >= 4) && (y <= (int) imgHeight - 4)) {

			dy = y - cy;

			row_mean = update_sums(0, y, rectified_frame_buf);
			non_max(0, inhibition_radius, minimum_response);

			/* store the features */
			prev_x = start_x;
			for (x = start_x; x > 15; x--) {
				if (row_peaks[x] > 0) {

					dx = x - cx + calibration_offset_x;
					r = dx*dx + dy*dy;
					if ((r < max_radius) && (r > min_radius)) {

						int n = ((y * imgWidth) + x - 2)*3;
						int n2 = ((y * imgWidth) + x + 2)*3;
						if ((!((rectified_frame_buf[n]==0) && (rectified_frame_buf[n+1]==0) && (rectified_frame_buf[n+2]==0))) &&
							(!((rectified_frame_buf[n2]==0) && (rectified_frame_buf[n2+1]==0) && (rectified_frame_buf[n2+2]==0)))) {

							mid_x = prev_x + ((x - prev_x)/2);

							feature_x[no_of_features] = (short int) (x + calibration_offset_x)*OMNI_SUB_PIXEL;

							/* parabolic sub-pixel interpolation */
							int denom = 2 * ((int)temp_row_peaks[x-1] - (2*(int)temp_row_peaks[x]) + (int)temp_row_peaks[x+1]);
							if (denom != 0) {
								int num = (int)temp_row_peaks[x-1] - (int)temp_row_peaks[x+1];
								feature_x[no_of_features] += (num*OMNI_SUB_PIXEL)/denom;
							}

							no_of_features++;

							no_of_feats++;
							if (no_of_features == OMNI_MAX_FEATURES) {
								y = imgHeight;
								printf("feature buffer full\n");
								break;
							}
							prev_x = x;
						}
					}
				}
			}
		}

		features_per_row[row_idx++] = no_of_feats;
	}

	no_of_features = no_of_features;

#ifdef OMNI_VERBOSE
	printf("%d vertically oriented edge features located\n", no_of_features);
#endif

	return (no_of_features);
}

/* returns a set of horizontally oriented features
 these can't be matched directly, but their disparities might be infered */
int omni::get_features_horizontal(unsigned char* rectified_frame_buf, /* image data */
int inhibition_radius, /* radius for non-maximal supression */
unsigned int minimum_response, /* minimum threshold */
int calibration_offset_x,
int calibration_offset_y,
int outer_radius_percent,
int inner_radius_percent)
{
	unsigned short int no_of_feats;
	int x, y, col_mean, start_y,r,dx,dy;
	int no_of_features = 0;
	int col_idx = 0;
	int cx = imgWidth/2;
	int cy = imgHeight/2;
	int max_radius = (outer_radius_percent * imgWidth / 200) - 2;
	int min_radius = (inner_radius_percent * imgWidth / 200) + 2;
	max_radius *= max_radius;
	min_radius *= min_radius;

	/* create arrays */
	if (features_per_col == 0) {
		features_per_col = (unsigned short int*) malloc(OMNI_MAX_IMAGE_WIDTH
				/ OMNI_HORIZONTAL_SAMPLING * sizeof(unsigned short int));
		feature_y = (short int*) malloc(OMNI_MAX_FEATURES * sizeof(short int));
	}

	memset((void*) features_per_col, '\0', OMNI_MAX_IMAGE_WIDTH
			/ OMNI_HORIZONTAL_SAMPLING * sizeof(unsigned short));

	start_y = imgHeight - 15;
	if ((int) imgHeight - inhibition_radius - 1 < start_y)
		start_y = (int) imgHeight - inhibition_radius - 1;

	for (x = 4 + calibration_offset_x; x < (int) imgWidth - 4; x
			+= OMNI_HORIZONTAL_SAMPLING) {

		/* reset number of features on the row */
		no_of_feats = 0;

		if ((x >= 4) && (x <= (int) imgWidth - 4)) {

			dx = x - cx;

			col_mean = update_sums(1, x, rectified_frame_buf);
			non_max(1, inhibition_radius, minimum_response);

			/* store the features */
			for (y = start_y; y > 15; y--) {
				if (row_peaks[y] > 0) {

					dy = y - cy;
					r = dx*dx + dy*dy;
					if ((r < max_radius) && (r > min_radius)) {

						int n = ((y * imgWidth) + x - 2)*3;
						int n2 = ((y * imgWidth) + x + 2)*3;
						if ((!((rectified_frame_buf[n]==0) && (rectified_frame_buf[n+1]==0) && (rectified_frame_buf[n+2]==0))) &&
							(!((rectified_frame_buf[n2]==0) && (rectified_frame_buf[n2+1]==0) && (rectified_frame_buf[n2+2]==0)))) {

							feature_y[no_of_features] = (short int) (y + calibration_offset_y)*OMNI_SUB_PIXEL;

							/* parabolic sub-pixel interpolation */
							int denom = 2 * ((int)temp_row_peaks[y-1] - (2*(int)temp_row_peaks[y]) + (int)temp_row_peaks[y+1]);
							if (denom != 0) {
								int num = (int)temp_row_peaks[y-1] - (int)temp_row_peaks[y+1];
								feature_y[no_of_features] += (num*OMNI_SUB_PIXEL)/denom;
							}
							no_of_features++;

							no_of_feats++;
							if (no_of_features == OMNI_MAX_FEATURES) {
								x = imgWidth;
								printf("feature buffer full\n");
								break;
							}
						}
					}
				}
			}
		}

		features_per_col[col_idx++] = no_of_feats;
	}

#ifdef OMNI_VERBOSE
	printf("%d horizontally oriented edge features located\n", no_of_features);
#endif

	return (no_of_features);
}

/* takes the raw image and camera calibration parameters and returns a rectified image */
void omni::rectify(unsigned char* raw_image, /* raw image grabbed from camera */
unsigned char* rectified_frame_buf) { /* returned rectified image */

	int max, n, i, idx;

	if (calibration_map != NULL) {

		n = 0;
		max = imgWidth * imgHeight * 3;
		for (i = 0; i < max; i += 3, n++) {
			idx = calibration_map[n] * 3;
			rectified_frame_buf[i] = raw_image[idx];
			rectified_frame_buf[i + 1] = raw_image[idx + 1];
			rectified_frame_buf[i + 2] = raw_image[idx + 2];
		}
	}
}

/* takes the raw image and camera calibration parameters and returns a rectified image */
void omni::make_map_int(long centre_of_distortion_x, /* centre of distortion x coordinate in pixels xOMNI_MULT */
long centre_of_distortion_y, /* centre of distortion y coordinate in pixels xOMNI_MULT */
long* coeff, /* three lens distortion polynomial coefficients xOMNI_MULT_COEFF */
long scale_num, /* scaling numerator */
long scale_denom) /* scaling denominator */
{
	const long OMNI_MULT = 1;
	const long OMNI_MULT_COEFF = 10000000;

	long v, powr, radial_dist_rectified, radial_dist_original;
	long i, j, x, y, dx, dy;
	long n, n2, x2, y2;
	long ww = imgWidth;
	long hh = imgHeight;
	long half_width = ww / 2;
	long half_height = hh / 2;
	scale_denom *= OMNI_MULT;
	calibration_map = new int[imgWidth * imgHeight];
	for (x = 0; x < ww; x++) {

		dx = (x * OMNI_MULT) - centre_of_distortion_x;

		for (y = 0; y < hh; y++) {

			dy = (y * OMNI_MULT) - centre_of_distortion_y;

			v = dx * dx + dy * dy;
			/* integer square root */
			for (radial_dist_rectified = 0; v >= (2* radial_dist_rectified )
					+ 1; v -= (2* radial_dist_rectified ++) + 1)
				;

			//float radial_dist_rectified2 = (float) sqrt(dx * dx + dy * dy);

			//printf("radial_dist_rectified %ld %f\n", radial_dist_rectified, radial_dist_rectified2);

			if (radial_dist_rectified >= 0) {

				/* for each polynomial coefficient */
				radial_dist_original = 0;
				double radial_dist_original2 = 0;
				for (i = 0; i < 4; i++) {

					/* pow(radial_dist_rectified, i) */
					powr = 1;
					if (i != 0) {
						j = i;
						while (j != 0) {
							powr *= radial_dist_rectified;
							j--;
						}
					} else {
						powr = 1;
					}

					//printf("powr %ld %lf\n", powr, pow(radial_dist_rectified, i));

					/*
					 if (coeff[i] >= 0) {
					 radial_dist_original += (unsigned long)coeff[i] * powr;
					 }
					 else {
					 radial_dist_original -= (unsigned long)(-coeff[i]) * powr;
					 }
					 */
					radial_dist_original += coeff[i] * powr;
					radial_dist_original2 += coeff[i] * pow(
							radial_dist_rectified, i);

					//printf("powr %ld %lf\n", coeff[i] * powr, (double)coeff[i] * pow(radial_dist_rectified, i));
				}
				//printf("radial_dist_original %ld %lf\n", radial_dist_original, radial_dist_original2);

				if (radial_dist_original > 0) {

					radial_dist_original /= OMNI_MULT_COEFF;
					radial_dist_original2 /= OMNI_MULT_COEFF;
					//printf("radial_dist_original %ld %lf\n", radial_dist_original, radial_dist_original2);

					//printf("radial_dist_rectified: %d\n", (int)(radial_dist_rectified/OMNI_MULT));
					//printf("radial_dist_original:  %d\n", (int)(radial_dist_original/OMNI_MULT));

					x2 = centre_of_distortion_x + (dx * radial_dist_original
							/ radial_dist_rectified);
					x2 = (x2 - (half_width * OMNI_MULT)) * scale_num
							/ scale_denom;
					y2 = centre_of_distortion_y + (dy * radial_dist_original
							/ radial_dist_rectified);
					y2 = (y2 - (half_height * OMNI_MULT)) * scale_num
							/ scale_denom;

					x2 += half_width;
					y2 += half_height;

					if ((x2 > -1) && (x2 < ww) && (y2 > -1) && (y2 < hh)) {

						n = y * imgWidth + x;
						n2 = y2 * imgWidth + x2;

						//int diff = calibration_map[(int) n] - (int) n2;
						//if (diff < 0)
						//	diff = -diff;
						//printf("diff: %d\n", diff);

						calibration_map[(int) n] = (int) n2;
					}
				}
			}
		}
	}
}

/*!
 * \brief returns true if the given file exists
 * \param filename name of the file
 */
bool omni::FileExists(
	std::string filename)
{
    std::ifstream inf;

    bool flag = false;
    inf.open(filename.c_str());
    if (inf.good()) flag = true;
    inf.close();
    return(flag);
}

/* flip the given image so that the camera can be mounted upside down */
void omni::flip(unsigned char* raw_image, unsigned char* flipped_frame_buf) {
	int max = imgWidth * imgHeight * 3;
	for (int i = 0; i < max; i += 3) {
		flipped_frame_buf[i] = raw_image[(max - 3 - i)];
		flipped_frame_buf[i + 1] = raw_image[(max - 3 - i + 1)];
		flipped_frame_buf[i + 2] = raw_image[(max - 3 - i + 2)];
	}
	memcpy(raw_image, flipped_frame_buf, max * sizeof(unsigned char));
}

/* removes any background surrounding the mirror */
void omni::remove(
	unsigned char* img,
	int img_width,
	int img_height,
	int bytes_per_pixel,
	float outer_radius_percent,
	float inner_radius_percent)
{
	int x,y,dx,dy,r,n,col;
	int max = (int)(img_width * outer_radius_percent / 200);
	int min = (int)(img_width * inner_radius_percent / 200);
	max *= max;
	min *= min;
	int cx = img_width/2;
	int cy = img_height/2;
	for (y = 0; y < img_height; y++) {
		dy = y - cy;
		for (x = 0; x < img_width; x++) {
		    dx = x - cx;
            r = dx*dx + dy*dy;
            if ((r > max) || (r < min)) {
            	n = ((y * img_width) + x)*bytes_per_pixel;
            	for (col = 0; col < bytes_per_pixel; col++) {
            	    img[n+col]=0;
            	}
            }
		}
	}
}

/* saves edge feature coordinates to file for use by other programs */
void omni::save_edges(
	std::string filename,
	int no_of_feats_vertical,
	int no_of_feats_horizontal)
{
	FILE *file = fopen(filename.c_str(), "wb");
	if (file != NULL) {

		struct MatchData {
			unsigned short int x;
			unsigned short int y;
		};

		MatchData *m = new MatchData[no_of_feats_horizontal+no_of_feats_vertical];

		int index = 0;

		/* vertically oriented features */
		int row = 0;
		int feats_remaining = features_per_row[row];

		for (int f = 0; f < no_of_feats_vertical; f++, feats_remaining--) {

			int x = feature_x[f];
			int y = (int)((4 + (row * OMNI_VERTICAL_SAMPLING)) * OMNI_SUB_PIXEL);
			if ((x > -1) && (x < (int)imgWidth) &&
				(y > -1) && (y < (int)imgHeight)) {
			    m[index].x = x;
			    m[index].y = y;
			    index++;
			}

			/* move to the next row */
			if (feats_remaining <= 0) {
				row++;
				feats_remaining = features_per_row[row];
			}
		}

		/* horizontally oriented features */
		int col = 0;
		feats_remaining = features_per_col[col];

		for (int f = 0; f < no_of_feats_horizontal; f++, feats_remaining--) {

			int y = feature_y[f];
			int x = (int)((4 + (col * OMNI_HORIZONTAL_SAMPLING)) * OMNI_SUB_PIXEL);
			if ((x > -1) && (x < (int)imgWidth) &&
				(y > -1) && (y < (int)imgHeight)) {
			    m[index].x = x;
			    m[index].y = y;
			    index++;
			}

		    /* move to the next column */
			if (feats_remaining <= 0) {
				col++;
				feats_remaining = features_per_col[col];
			}
		}

		fprintf(file,"%d", index);
		fwrite(m, sizeof(MatchData), index, file);
		delete[] m;

		fclose(file);
	}
}

/* saves radial lines */
void omni::save_radial_lines(
	std::string filename)
{
	FILE *file = fopen(filename.c_str(), "wb");
	if (file != NULL) {

		struct MatchData {
			int rotation_degrees;
			int x0;
			int y0;
			int x1;
			int y1;
		};

		MatchData *m = new MatchData[no_of_radial_lines+1];
		for (int i = 0; i < no_of_radial_lines; i++) {
			m[i].rotation_degrees = radial_lines[i*5];
			m[i].x0 = radial_lines[i*5+1];
			m[i].y0 = radial_lines[i*5+2];
			m[i].x1 = radial_lines[i*5+3];
			m[i].y1 = radial_lines[i*5+4];
		}

		fprintf(file, "%d", no_of_radial_lines);
		fwrite(m, sizeof(MatchData), no_of_radial_lines, file);
		delete[] m;

		fclose(file);
	}
}


void omni::get_ray(
	int img_width,
	int pixel_x, int pixel_y,
	int& ray_origin_x_mm,
	int& ray_origin_y_mm,
	int& ray_origin_z_mm,
	int& ray_ground_x_mm,
	int& ray_ground_y_mm,
	int& ray_ground_z_mm)
{
	int n = (pixel_y*img_width+pixel_x)*6;
	ray_origin_x_mm = ray_map[n];
	ray_origin_y_mm = ray_map[n+1];
	ray_origin_z_mm = ray_map[n+2];

	ray_ground_x_mm = ray_map[n+3];
	ray_ground_y_mm = ray_map[n+4];
	ray_ground_z_mm = ray_map[n+5];
}

/* saves 3D rays to file for use by other programs */
void omni::save_rays(
	std::string filename,
	int no_of_feats_vertical,
	int no_of_feats_horizontal)
{
	FILE *file = fopen(filename.c_str(), "wb");
	if (file != NULL) {

		struct MatchData {
			short start_x;
			short start_y;
			short start_z;
			short end_x;
			short end_y;
			short end_z;
		};

		MatchData *m = new MatchData[no_of_feats_horizontal+no_of_feats_vertical];

		int index = 0;

		/* vertically oriented features */
		int row = 0;
		int feats_remaining = features_per_row[row];

		for (int f = 0; f < no_of_feats_vertical; f++, feats_remaining--) {

			int x = feature_x[f] / OMNI_SUB_PIXEL;
			int y = (int)((4 + (row * OMNI_VERTICAL_SAMPLING)));
			if ((x > -1) && (x < (int)imgWidth) &&
				(y > -1) && (y < (int)imgHeight) &&
				(ray_map[(y*imgWidth+x)*6] > -32000) &&
				(ray_map[(y*imgWidth+x)*6] < 32000) &&
				(ray_map[(y*imgWidth+x)*6+1] > -32000) &&
				(ray_map[(y*imgWidth+x)*6+1] < 32000) &&
				(ray_map[(y*imgWidth+x)*6+3] > -32000) &&
				(ray_map[(y*imgWidth+x)*6+3] < 32000) &&
				(ray_map[(y*imgWidth+x)*6+4] > -32000) &&
				(ray_map[(y*imgWidth+x)*6+4] < 32000)) {
				m[index].start_x = ray_map[(y*imgWidth+x)*6];
				m[index].start_y = ray_map[(y*imgWidth+x)*6+1];
				m[index].start_z = ray_map[(y*imgWidth+x)*6+2];
				m[index].end_x = ray_map[(y*imgWidth+x)*6+3];
				m[index].end_y = ray_map[(y*imgWidth+x)*6+4];
				m[index].end_z = ray_map[(y*imgWidth+x)*6+5];
				index++;
		    }

			/* move to the next row */
			if (feats_remaining <= 0) {
				row++;
				feats_remaining = features_per_row[row];
			}
		}

		/* horizontally oriented features */
		int col = 0;
		feats_remaining = features_per_col[col];

		for (int f = 0; f < no_of_feats_horizontal; f++, feats_remaining--) {

			int y = feature_y[f] / OMNI_SUB_PIXEL;
			int x = (int)((4 + (col * OMNI_HORIZONTAL_SAMPLING)));
			if ((x > -1) && (x < (int)imgWidth) &&
				(y > -1) && (y < (int)imgHeight) &&
				(ray_map[(y*imgWidth+x)*6] > -32000) &&
				(ray_map[(y*imgWidth+x)*6] < 32000) &&
				(ray_map[(y*imgWidth+x)*6+1] > -32000) &&
				(ray_map[(y*imgWidth+x)*6+1] < 32000) &&
				(ray_map[(y*imgWidth+x)*6+3] > -32000) &&
				(ray_map[(y*imgWidth+x)*6+3] < 32000) &&
				(ray_map[(y*imgWidth+x)*6+4] > -32000) &&
				(ray_map[(y*imgWidth+x)*6+4] < 32000)) {
				m[index].start_x = ray_map[(y*imgWidth+x)*6];
				m[index].start_y = ray_map[(y*imgWidth+x)*6+1];
				m[index].start_z = ray_map[(y*imgWidth+x)*6+2];
				m[index].end_x = ray_map[(y*imgWidth+x)*6+3];
				m[index].end_y = ray_map[(y*imgWidth+x)*6+4];
				m[index].end_z = ray_map[(y*imgWidth+x)*6+5];
				index++;
			}

		    /* move to the next column */
			if (feats_remaining <= 0) {
				col++;
				feats_remaining = features_per_col[col];
			}
		}

		fprintf(file, "%d", index);
		fwrite(m, sizeof(MatchData), index, file);
		delete[] m;

		fclose(file);
	}
}

void omni::calibrate(
    unsigned char* img,
    int img_width,
    int img_height,
    int bytes_per_pixel,
    int inner_radius_percent,
    int outer_radius_percent,
    std::string direction)
{
	int max_radius = outer_radius_percent * img_width / 200;
	int min_radius = inner_radius_percent * img_width / 200;
	int* response = new int[max_radius];
	int* edge = new int[30];
	int no_of_edges = 0;
	int mean = 0;

	int x,y,dx,dy,r,n,col,xx,yy;
	int cx = img_width/2;
	int cy = img_height/2;
	int max = 1;

	int end_x = cx;
	int end_y = 0;

	memset((void*)response, '\0', max_radius*sizeof(int));

	if ((direction == "N") || (direction == "n")) {
		end_x = cx;
		end_y = 0;
	}

	if ((direction == "S") || (direction == "s")) {
		end_x = cx;
		end_y = img_height-1;
	}

	if ((direction == "E") || (direction == "e")) {
		end_x = img_width-1;
		end_y = cy;
	}

	if ((direction == "W") || (direction == "w")) {
		end_x = 0;
		end_y = cy;
	}

	if ((direction == "NE") || (direction == "ne")) {
		end_x = img_width-1;
		end_y = 0;
	}

	if ((direction == "NW") || (direction == "nw")) {
		end_x = 0;
		end_y = 0;
	}

	if ((direction == "SE") || (direction == "se")) {
		end_x = img_width-1;
		end_y = img_height-1;
	}

	if ((direction == "SW") || (direction == "sw")) {
		end_x = 0;
		end_y = img_height-1;
	}

	dx = end_x - cx;
	dy = end_y - cy;

	for (r = min_radius; r < max_radius; r++) {
        x = cx + (r * dx / max_radius);
        y = cy + (r * dy / max_radius);

        if ((x > 0) && (x < img_width-1) &&
        	(y > 0) && (y < img_height-1)) {
			for (yy = y-1; yy <= y+1; yy++) {
				for (xx = x-1; xx <= x+1; xx++) {
					n = ((yy * img_width) + xx) * bytes_per_pixel;
					for (col = 0; col < bytes_per_pixel; col++) {
						response[r] += img[n+col];
					}
				}
			}
			if (response[r] > max) max = response[r];
			mean += response[r];
        }
	}
	mean /= (max_radius - min_radius);

	drawing::drawLine(img, img_width,img_height,cx,cy,end_x,end_y,255,0,0,0,false);

	int tx=0,ty,bx=img_width,by;

	if (end_y > cy) {
		ty = 0;
		by = cy/2;
	}
	else {
		ty = cy + (cy/2);
		by = img_height;
	}

	max = max * 120/100;
	int mean_y = by-1-(mean * (by-ty) / max);
	int prev_yy = mean_y;
	int prev_cross_x = 0;
	int width,prev_width=9999;

	for (x = tx; x < bx; x++) {
		r = min_radius + ((x - tx) * (max_radius - min_radius) / (bx - tx));
		yy = by-1-(response[r] * (by-ty) / max);
		for (y = by-1; y >= ty; y--) {
        	n = ((y * img_width) + x) * bytes_per_pixel;
        	for (col = 0; col < 3; col++) {
                if (y > yy) {
            		img[n+col] = 255;
            	}
                else {
                	img[n+col] = 0;
                }
            }
		}
		if ((x > tx+20) && (x < img_width*8/10)) {
			if ((prev_yy <= mean_y) && (yy >= mean_y)) {
				width = x - prev_cross_x;
				if (width > 5) {
                    if (no_of_edges < 30) {
                        drawing::drawSpot(img, img_width, img_height,x,mean_y,2,255,0,0);
                        edge[no_of_edges++] = r;
                        prev_width = width;
                    }

				}
				prev_cross_x = x;
			}
			if ((prev_yy >= mean_y) && (yy <= mean_y)) {
				width = x - prev_cross_x;
				if (width > 5) {
                    if (no_of_edges < 30) {
                        drawing::drawSpot(img, img_width, img_height,x,mean_y,2,255,0,0);
                        edge[no_of_edges++] = r;
                        prev_width = width;
                    }

				}
				prev_cross_x = x;
			}
		}
		prev_yy = yy;
	}

	drawing::drawLine(img, img_width,img_height,tx,mean_y,bx,mean_y,0,255,0,0,false);

	int threshold = 2000;
	if (no_of_edges > 1) {
		prev_width = edge[1] - edge[0];
		int dx2 = dx/8;
		int dy2 = dy/8;
		int i,x2,y2,v;
	    for (r = 1; r < no_of_edges; r++) {
	    	int r2 = edge[r-1] + ((edge[r] - edge[r-1])/2);
	    	int max_i = max_radius/4;
	    	int centre_v = 0;
	    	for (i = 0; i < max_i; i++) {
	    		x2 = cx + (r2 * dx / max_radius) + (i * dy2 / max_i);
	    		y2 = cy + (r2 * dy / max_radius) + (i * dx2 / max_i);
	    		v = 0;
	    		for (yy = y2-1; yy <= y2+1; yy++) {
	    			if ((yy > -1) && (yy < img_height)) {
						for (xx = x2-1; xx <= x2+1; xx++) {
							if ((xx > -1) && (xx < img_width)) {
								n = ((yy * img_width) + xx)*bytes_per_pixel;
								for (col = 0; col < bytes_per_pixel; col++) {
									v += img[n+col];
								}
							}
						}
	    			}
	    		}
	    		if (i == 0) {
	    			centre_v = v;
	    		}
	    		else {
	    			if (abs(centre_v-v) > threshold) {
	    			    drawing::drawSpot(img, img_width, img_height,x2,y2,2,255,255,0);
	    			    break;
	    			}
	    		}
	    	}

	    	for (i = 0; i < max_i; i++) {
	    		x2 = cx + (r2 * dx / max_radius) - (i * dy2 / max_i);
	    		y2 = cy + (r2 * dy / max_radius) - (i * dx2 / max_i);
	    		v = 0;
	    		for (yy = y2-1; yy <= y2+1; yy++) {
	    			if ((yy > -1) && (yy < img_height)) {
						for (xx = x2-1; xx <= x2+1; xx++) {
							if ((xx > -1) && (xx < img_width)) {
								n = ((yy * img_width) + xx)*bytes_per_pixel;
								for (col = 0; col < bytes_per_pixel; col++) {
									v += img[n+col];
								}
							}
						}
	    			}
	    		}
	    		if (i == 0) {
	    			centre_v = v;
	    		}
	    		else {
	    			if (abs(centre_v-v) > threshold) {
	    			    drawing::drawSpot(img, img_width, img_height,x2,y2,2,255,255,0);
	    			    break;
	    			}
	    		}
	    	}

	    }

		int epipole_r = 0;
		int epipole_r_hits = 0;
		prev_width = edge[1] - edge[0];
	    for (r = 2; r < no_of_edges; r++) {
		    width = edge[r] - edge[r-1];

		    if (prev_width > width) {
		    	int w = width;
		    	int r2 = edge[r];
		    	while (w > 0) {
		    	    r2 += w;
		    	    w -= (prev_width-width);
		    	}
		    	epipole_r += r2;
		    	epipole_r_hits++;
		    }
		    prev_width = width;
	    }
	    if (epipole_r_hits > 0) {
	    	epipole_r /= epipole_r_hits;
	    	//if (epipole > 0)
	    		//epipole = ((epipole*90) + (epipole_r*10))/100;
	    	//else
	    		//epipole = epipole_r;
	    	x = tx + (epipole * (bx-tx) / max_radius);
		    drawing::drawSpot(img, img_width, img_height,x,mean_y,2,255,255,0);

	        x = cx + (epipole * dx / max_radius);
	        y = cy + (epipole * dy / max_radius);
		    drawing::drawCircle(img, img_width, img_height,x,y,3,255,0,0,0);
	    }
	}

	delete[] response;
	delete[] edge;
}

void omni::get_calibration_image(
	unsigned char* img,
	int img_width,
	int img_height)
{
	int x,y;
	int squares_across = 20;
	int squares_down = 14;
	int state0,state1,n=0;

	for (y = 0; y < img_height; y++) {
		state0 = (y * squares_down / img_height) % 2;
		for (x = 0; x < img_width; x++,n+=3) {
			state1 = (x * squares_across / img_width) % 2;
			if (state0 != 0) state1 = 1-state1;
			if (state1 == 0) {
				img[n] = 0;
				img[n+1] = 0;
				img[n+2] = 0;
			}
			else {
				img[n] = 255;
				img[n+1] = 255;
				img[n+2] = 255;
			}
		}
	}
}


/*!
 * \brief does the line intersect with the given line?
 * \param x0 first line top x
 * \param y0 first line top y
 * \param x1 first line bottom x
 * \param y1 first line bottom y
 * \param x2 second line top x
 * \param y2 second line top y
 * \param x3 second line bottom x
 * \param y3 second line bottom y
 * \param xi intersection x coordinate
 * \param yi intersection y coordinate
 * \return true if the lines intersect
 */
bool omni::intersection(
    float x0,
    float y0,
    float x1,
    float y1,
    float x2,
    float y2,
    float x3,
    float y3,
    float& xi,
    float& yi)
{
    float a1, b1, c1,         //constants of linear equations
          a2, b2, c2,
          det_inv,            //the inverse of the determinant of the coefficient
          m1, m2, dm;         //the gradients of each line
    bool insideLine = false;  //is the intersection along the lines given, or outside them
    float tx, ty, bx, by;

    //compute gradients, note the cludge for infinity, however, this will
    //be close enough
    if ((x1 - x0) != 0)
        m1 = (y1 - y0) / (x1 - x0);
    else
        m1 = (float)1e+10;   //close, but no cigar

    if ((x3 - x2) != 0)
        m2 = (y3 - y2) / (x3 - x2);
    else
        m2 = (float)1e+10;   //close, but no cigar

    dm = fabs(m1 - m2);
    if (dm > 0.000001f)
    {
        //compute constants
        a1 = m1;
        a2 = m2;

        b1 = -1;
        b2 = -1;

        c1 = (y0 - m1 * x0);
        c2 = (y2 - m2 * x2);

        //compute the inverse of the determinate
        det_inv = 1 / (a1 * b2 - a2 * b1);

        //use Kramers rule to compute xi and yi
        xi = ((b1 * c2 - b2 * c1) * det_inv);
        yi = ((a2 * c1 - a1 * c2) * det_inv);

        //is the intersection inside the line or outside it?
        if (x0 < x1) { tx = x0; bx = x1; } else { tx = x1; bx = x0; }
        if (y0 < y1) { ty = y0; by = y1; } else { ty = y1; by = y0; }
        if ((xi >= tx) && (xi <= bx) && (yi >= ty) && (yi <= by))
        {
            if (x2 < x3) { tx = x2; bx = x3; } else { tx = x3; bx = x2; }
            if (y2 < y3) { ty = y2; by = y3; } else { ty = y3; by = y2; }
            if ((xi >= tx) && (xi <= bx) && (yi >= ty) && (yi <= by))
            {
                insideLine = true;
            }
        }
    }
    else
    {
        //parallel (or parallelish) lines, return some indicative value
        xi = 9999;
    }

    return (insideLine);
}

float omni::intersection_ray_sphere(
    float ray_x0,
    float ray_y0,
    float ray_z0,
    float ray_x1,
    float ray_y1,
    float ray_z1,
    float sphere_x,
    float sphere_y,
    float sphere_z,
    float sphere_radius)
{
  float w_x = ray_x0 - sphere_x;
  float w_y = ray_y0 - sphere_y;
  float w_z = ray_z0 - sphere_z;

  float r_v_x = ray_x1 - ray_x0;
  float r_v_y = ray_y1 - ray_y0;
  float r_v_z = ray_z1 - ray_z0;

  float A = r_v_x*r_v_x + r_v_y*r_v_y + r_v_z*r_v_z;
  float B = 2*(w_x*r_v_x + w_y*r_v_y + w_z*r_v_z);
  float C = (w_x*w_x + w_y*w_y + w_z*w_z) - (sphere_radius*sphere_radius);

  float D = B*B-4.0f*A*C;
  if (D >= 0.0f)
	  return((-B - sqrt(D)) / (2.0f*A));
  else
  	  return(0);
}

void omni::rgb_to_hsv(
    int r, int g, int b,
    unsigned char& h,
    unsigned char& s,
    unsigned char& v)
{

    float red = r / 255.0f, green = g / 255.0f, blue = b / 255.0f;

    float maxRGB = max(blue,max(red, green));
    float minRGB = min(blue,min(red, green));

    float hConvert = 0;
    float vConvert = maxRGB;
    float sConvert = (vConvert - (minRGB))/vConvert;

    if ((red == maxRGB) &&
        (green == minRGB))
        hConvert = 5 + ((red - blue)/(red - green));

    else if ((red == maxRGB) && (blue == minRGB))
        hConvert = 1 - ((red - green)/(red - blue));

    else if ((green == maxRGB) && (blue == minRGB))
        hConvert = 1 + ((green - red)/(green -blue));

    else if ((green == maxRGB) && (red == minRGB))
        hConvert = 3 - ((green - blue)/(green - red));

    else if ((blue == maxRGB) && (red == minRGB))
        hConvert = 3 + ((blue - green)/(blue - red));

    else if ((blue == maxRGB) && (green == minRGB))
        hConvert = 5 - ((blue - red)/(blue - green));

    hConvert = hConvert * 60;

    if (hConvert < 0) hConvert += 360;

    h = round(hConvert/360*31);
    s = round(sConvert*255);
    v = round(vConvert*255);
}

void omni::init_grid(
    int grid_centre_x_mm,
    int grid_centre_y_mm,
	int grid_centre_z_mm,
	int grid_cell_dimension_mm,
	int grid_dimension_cells)
{
    this->grid_centre_x_mm = grid_centre_x_mm;
    this->grid_centre_y_mm = grid_centre_y_mm;
    this->grid_centre_z_mm = grid_centre_z_mm;
    this->grid_cell_dimension_mm = grid_cell_dimension_mm;
    this->grid_dimension_cells = grid_dimension_cells;

    if (occupancy_grid != NULL) {
    	delete[] occupancy_grid;
    }
	occupancy_grid = new unsigned int[grid_dimension_cells*grid_dimension_cells*grid_dimension_cells*2];
	memset((void*)occupancy_grid,'\0',grid_dimension_cells*grid_dimension_cells*grid_dimension_cells*2*sizeof(unsigned int));
}


void omni::update_grid_map(
	float mirror_diameter,
	unsigned char* img,
    int img_width,
    int img_height)
{
	const int max_cols = 5;

	if (occupancy_grid != NULL) {

		int offset_x = (grid_dimension_cells/2) - (grid_centre_x_mm / grid_cell_dimension_mm);
		int offset_y = (grid_dimension_cells/2) - (grid_centre_y_mm / grid_cell_dimension_mm);
		int offset_z = (grid_dimension_cells/2) - (grid_centre_z_mm / grid_cell_dimension_mm);

		int min_radius_cells = (int)(mirror_diameter / grid_cell_dimension_mm);
		unsigned char h=0,s=0,v=0;
		int grid_dimension_cells_sqr = grid_dimension_cells*grid_dimension_cells;
		int dx,dy,dz,cell_x,cell_y,cell_z,r,length,idx,hue_bit;
		int ray_start_x, ray_start_y, ray_start_z, ray_end_x, ray_end_y, ray_end_z;
		int pixels = img_width*img_height;
		int n = (pixels-1)*6;
		for (int i = pixels-1; i >= 0; i--, n-=6) {
			if (mirror_map[i] > 0) {

				// extract the hue
				rgb_to_hsv(img[i*3+2],img[i*3+1],img[i*3],h,s,v);
				hue_bit = pow(2,h);

				// calculate the start and end grid cell coordinates of the ray
				ray_start_x = (ray_map[n] / grid_cell_dimension_mm) + offset_x;
				ray_start_y = (ray_map[n+1] / grid_cell_dimension_mm) + offset_y;
				ray_start_z = (ray_map[n+2] / grid_cell_dimension_mm) + offset_z;
				ray_end_x = (ray_map[n+3] / grid_cell_dimension_mm) + offset_x;
				ray_end_y = (ray_map[n+4] / grid_cell_dimension_mm) + offset_y;
				ray_end_z = (ray_map[n+5] / grid_cell_dimension_mm) + offset_z;

				dx = ray_end_x - ray_start_x;
				dy = ray_end_y - ray_start_y;
				dz = ray_end_z - ray_start_z;

				// calculate the length of the ray
				length = (int)sqrt(dx*dx + dy*dy + dz*dz);
				if (length > min_radius_cells) {

					// step along the length of the ray
					for (r = length-1; r > min_radius_cells; r--) {
						cell_x =  ray_start_x + (r * dx / length);
						if ((cell_x < 0) || (cell_x >= grid_dimension_cells)) break;
						cell_y =  ray_start_y + (r * dy / length);
						if ((cell_y < 0) || (cell_y >= grid_dimension_cells)) break;
						cell_z =  ray_start_z + (r * dz / length);
						if ((cell_z < 0) || (cell_z >= grid_dimension_cells)) break;
						idx = ((cell_z * grid_dimension_cells_sqr) +
							   (cell_y * grid_dimension_cells) +
							   cell_x)*2;
						occupancy_grid[idx] |= hue_bit;
						if (occupancy_grid[idx+1] < 4294967295u) {
							occupancy_grid[idx+1]++;
						}
						else {
							printf("grid cell counter overflow\n");
						}
					}
				}
			}
		}

		/* lookup table used for counting the number of set bits */
		const unsigned char BitsSetTable256[] =
		{
			0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
			1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
			1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
			2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
			1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
			2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
			2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
			3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
			1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
			2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
			2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
			3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
			2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
			3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
			3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
			4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8
		};

		// knock out grid cells with too much colour variance
		point_cloud.clear();
		int average_hits = 0;
		int no_of_cols,hits = 0;
		for (int i = grid_dimension_cells*grid_dimension_cells*grid_dimension_cells*2-1; i >= 0; i-=2) {
			if (occupancy_grid[i] > 0) {
				bool is_valid = false;
				if (occupancy_grid[i] > 1) {
					unsigned int cols = occupancy_grid[i-1];
					if (cols > 0) {
						// count the number of colours
						no_of_cols =
							BitsSetTable256[cols & 0xff] +
							BitsSetTable256[(cols >> 8) & 0xff] +
							BitsSetTable256[(cols >> 16) & 0xff] +
							BitsSetTable256[cols >> 24];

						if (no_of_cols <= max_cols) {
							average_hits += occupancy_grid[i];
							hits++;
							is_valid = true;
							point_cloud.push_back(i-1);
						}
					}
				}
				if (!is_valid) {
					occupancy_grid[i-1] = 0;
					occupancy_grid[i] = 0;
				}
			}
		}
		if (hits > 0) average_hits /= hits;

		unsigned int threshold = (unsigned int)(average_hits*300/100);
		vector<int> new_point_cloud;
		for (int i = (int)point_cloud.size()-1; i >= 0; i--) {
			idx = point_cloud[i];
			if (occupancy_grid[idx+1] > threshold) {
				cell_z = idx / grid_dimension_cells_sqr;
				cell_y = (idx - (cell_z*grid_dimension_cells_sqr)) / grid_dimension_cells;
				cell_x = idx - (cell_z*grid_dimension_cells_sqr) - (cell_y*grid_dimension_cells);

				cell_x /= 2;
				cell_y /= 2;
				cell_z /= 2;

				cell_x -= offset_x;
				cell_y -= offset_y;
				cell_z -= offset_z;

				cell_x *= grid_cell_dimension_mm;
				cell_y *= grid_cell_dimension_mm;
				cell_z *= grid_cell_dimension_mm;

				new_point_cloud.push_back(cell_z);
				new_point_cloud.push_back(cell_y);
				new_point_cloud.push_back(cell_x);
			}
			occupancy_grid[idx+1] = 0;
			occupancy_grid[idx] = 0;
		}
		point_cloud.clear();
		for (int i = (int)new_point_cloud.size()-1; i >= 0; i--) {
			point_cloud.push_back(new_point_cloud[i]);
		}
		//printf("threshold %d\n", (int)threshold);
		//printf("points %d\n", (int)point_cloud.size()/3);

	}
}

void omni::show_point_cloud(
	unsigned char* img,
	int img_width,
	int img_height,
	int min_x_mm, int max_x_mm,
	int min_y_mm, int max_y_mm,
	int min_z_mm, int max_z_mm)
{
	memset((void*)img,'\0',img_width*img_height*3);
	int n,x,y;
	int ww = img_width/2;
	int hh = img_height/2;
	for (int i = 0; i < (int)point_cloud.size(); i += 3) {
		// XY
        x = ((point_cloud[i] - min_x_mm) * ww / (max_x_mm - min_x_mm));
        y = hh - 1 - ((point_cloud[i+1] - min_y_mm) * hh / (max_y_mm - min_y_mm));
        n = ((y * img_width) + x) * 3;
        img[n] = 255;
        img[n+1] = 255;
        img[n+2] = 255;

        // XZ
        x = ww + ((point_cloud[i] - min_x_mm) * ww / (max_x_mm - min_x_mm));
        y = hh - 1 - ((point_cloud[i+2] - min_z_mm) * hh / (max_z_mm - min_z_mm));
        n = ((y * img_width) + x) * 3;
        img[n] = 255;
        img[n+1] = 255;
        img[n+2] = 255;

        // YZ
        x = ww + ((point_cloud[i+1] - min_y_mm) * ww / (max_y_mm - min_y_mm));
        y = (2*hh) - 1 - ((point_cloud[i+2] - min_z_mm) * hh / (max_z_mm - min_z_mm));
        n = ((y * img_width) + x) * 3;
        img[n] = 255;
        img[n+1] = 255;
        img[n+2] = 255;
	}
}

void omni::create_ray_map(
	float mirror_diameter,
	float dist_to_mirror_backing,
	float focal_length,
	float outer_radius_percent,
	float camera_height_mm,
	int no_of_mirrors,
	float* mirror_position,
	float* mirror_position_pixels,
    int img_width,
    int img_height)
{
	if (ray_map == NULL) {
		ray_map = new int[img_width*img_height*6];
		memset((void*)ray_map,'\0',img_width*img_height*6*sizeof(int));
		mirror_map = new unsigned char[img_width*img_height];
		memset((void*)mirror_map,'\0',img_width*img_height);
	}

	float half_pi = 3.1415927f/2;
	float ang_incr = 3.1415927f / (180*100);
	int max_index = (int)(half_pi / ang_incr)+1;
	float* lookup_pixel_x = new float[max_index];
	float* lookup_ray = new float[max_index*4];
	float* lookup_angle = new float[max_index];
	float* calibration_radius = new float[img_width*4];
	unwarp_lookup = new int[img_width*img_height];
	unwarp_lookup_reverse = new int[img_width*img_height];

	float outer_radius_pixels = outer_radius_percent * img_width / 200;
	int cx = img_width/2;
	int cy = img_height/2;

	memset((void*)ray_map,'\0', img_width*img_height*6*sizeof(int));

	// get the bounding box for all mirrors
	float bounding_box_tx = 9999;
	float bounding_box_ty = 9999;
	float bounding_box_bx = -9999;
	float bounding_box_by = -9999;
	float centre_x = 0, centre_y = 0;
	for (int mirror = 0; mirror < no_of_mirrors; mirror++) {
		centre_x += mirror_position[mirror*2];
		centre_y += mirror_position[mirror*2+1];
	}
	centre_x /= no_of_mirrors;
	centre_y /= no_of_mirrors;
	float max_dx = 0;
	float max_dy = 0;
	for (int mirror = 0; mirror < no_of_mirrors; mirror++) {
		max_dx += fabs(mirror_position[mirror*2] - centre_x);
		max_dy += fabs(mirror_position[mirror*2+1] - centre_y);
	}
	max_dx /= no_of_mirrors;
	max_dy /= no_of_mirrors;
	bounding_box_tx = centre_x - max_dx - (mirror_diameter*0.5f);
	bounding_box_ty = centre_y - max_dy - (mirror_diameter*0.5f);
	bounding_box_bx = centre_x + max_dx + (mirror_diameter*0.5f);
	bounding_box_by = centre_y + max_dy + (mirror_diameter*0.5f);

	for (int mirror = 0; mirror < no_of_mirrors; mirror++) {

		// mirror distance from the centre of the backing plane (xy)
		float mirror_dist_from_backing_centre =
			(float)sqrt(mirror_position[mirror*2]*mirror_position[mirror*2] +
					mirror_position[mirror*2+1]*mirror_position[mirror*2+1]);

		// distance from camera to the centre of the mirror
		float dist_to_mirror =
			(float)sqrt(mirror_dist_from_backing_centre*mirror_dist_from_backing_centre +
			dist_to_mirror_backing*dist_to_mirror_backing);

		// angle of the mirror on the xz plane
		float mirror_angle = (float)atan2(mirror_dist_from_backing_centre, dist_to_mirror_backing);
		float mirror_angle2 = mirror_angle + 3.1415927f;

		// rotation of the mirror on the backing plane (xy)
		float mirror_rotation = (float)atan2(mirror_position[mirror*2], mirror_position[mirror*2+1]);
		float mirror_rotation2 = mirror_rotation + 3.1415927f;

		float dist_to_mirror_centre = focal_length + dist_to_mirror;
		float sphere_x = (float)fabs(mirror_diameter*0.5*sin(half_pi));
		float sphere_y = dist_to_mirror_centre - (float)(mirror_diameter*0.5f*cos(half_pi));
		float max_viewing_angle = (float)fabs(atan(sphere_x / sphere_y));
		float pixel_x = 0;
		memset((void*)lookup_pixel_x,'\0',max_index*sizeof(float));
		memset((void*)lookup_ray,'\0',max_index*4*sizeof(float));
		memset((void*)lookup_angle,'\0',max_index*sizeof(float));

		int i = 0;
		float min_angle = 9999;
		float max_angle = -9999;
		for (float ang = ang_incr; ang < half_pi; ang += ang_incr, i++) {
			sphere_x = (float)fabs(mirror_diameter*0.5*sin(ang));
			sphere_y = dist_to_mirror_centre - (float)fabs(mirror_diameter*0.5f*cos(ang));
			float viewing_angle = (float)fabs(atan2(sphere_x, sphere_y));
			float ang2 = viewing_angle + (ang*2);
			float ray_end_y = 0;
			float ray_end_x = sphere_x + (sphere_y * (float)tan(ang2));
			if (ray_end_x < 0) {
				ray_end_x = sphere_x - (sphere_y * (float)tan(ang2));
				ray_end_y = sphere_y * 2;
			}

			pixel_x = viewing_angle * outer_radius_pixels / max_viewing_angle;

			lookup_pixel_x[i] = pixel_x;
			lookup_ray[i*4] = sphere_x;
			lookup_ray[i*4+1] = sphere_y;
			lookup_ray[i*4+2] = ray_end_x;
			lookup_ray[i*4+3] = ray_end_y;
			lookup_angle[i] = (float)atan2(ray_end_x, dist_to_mirror_centre);
			if (lookup_angle[i] < min_angle) min_angle = lookup_angle[i];
			if (lookup_angle[i] > max_angle) max_angle = lookup_angle[i];
		}

		// create unwarp lookup
		memset((void*)unwarp_lookup, '\0', img_width*img_height*sizeof(int));
		memset((void*)unwarp_lookup_reverse, '\0', img_width*img_height*sizeof(int));
		int n = 0;
		float angle;
		if (mirror == no_of_mirrors-1) {
			for (int j = 0; j < img_height; j++) {
				float target_ang = min_angle + (j * (max_angle-min_angle) / (img_width/2));
				int k = 0;
				for (k = 0; k < i; k++) {
					if (lookup_angle[k] >= target_ang) break;
				}
				calibration_radius[j] = lookup_pixel_x[k];
			}

			for (int x = 0; x < img_width; x++) {
				angle = x * 3.1415927f*2 / img_width;
				for (int y = 0; y < img_height; y++) {
					float raw_radius = calibration_radius[y];
					int raw_x = cx + (int)(raw_radius * sin(angle));
					int raw_y = cy + (int)(raw_radius * cos(angle));
					n = ((img_height-1-y) * img_width)+x;
					int n2 = (raw_y * img_width)+raw_x;
					if ((n2 > -1) && (n2 < img_width*img_height)) {
						unwarp_lookup[n] = n2;
						unwarp_lookup_reverse[n2] = n;
					}
				}
			}
		}

		float prev_x = 0;
		float prev_ray_start_x = 0;
		float prev_ray_start_y = 0;
		float prev_ray_end_x = 0;
		float prev_ray_end_y = 0;
		memset((void*)calibration_radius,'\0', img_width*4*sizeof(float));
		for (int j = 0; j < i; j++) {
			int x = (int)lookup_pixel_x[j];
			if (x != prev_x) {
				for (int k = prev_x; k <= x; k++) {
				   calibration_radius[k*4] = (prev_ray_start_x + (((k-prev_x) * (lookup_ray[j*4] - prev_ray_start_x)) / (float)(x-prev_x)));
				   calibration_radius[k*4+1] = (prev_ray_start_y + (((k-prev_x) * (lookup_ray[j*4+1] - prev_ray_start_y)) / (float)(x-prev_x)));

				   calibration_radius[k*4+2] = (prev_ray_end_x + (((k-prev_x) * (lookup_ray[j*4+2] - prev_ray_end_x)) / (float)(x-prev_x)));
				   calibration_radius[k*4+3] = (prev_ray_end_y + (((k-prev_x) * (lookup_ray[j*4+3] - prev_ray_end_y)) / (float)(x-prev_x)));
				}
				prev_x = x;
				prev_ray_start_x = lookup_ray[j*4];
				prev_ray_start_y = lookup_ray[j*4+1];
				prev_ray_end_x = lookup_ray[j*4+2];
				prev_ray_end_y = lookup_ray[j*4+3];
			}
		}

		float mirror_cx = mirror_position_pixels[mirror*2] * img_width / 100;
		float mirror_cy = mirror_position_pixels[mirror*2+1] * img_height / 100;
		float mirror_r = outer_radius_pixels;

		int min_x = (int)(mirror_cx - mirror_r);
		int max_x = (int)(mirror_cx + mirror_r);
		int min_y = (int)(mirror_cy - mirror_r);
		int max_y = (int)(mirror_cy + mirror_r);
		if (min_x < 0) min_x = 0;
		if (min_y < 0) min_y = 0;
		if (max_x >= img_width) max_x = img_width-1;
		if (max_y >= img_height) max_y = img_height-1;

		for (int y = min_y; y <= max_y; y++) {
			float dy = y - mirror_cy;
			for (int x = min_x; x <= max_x; x++) {
				float dx = x - mirror_cx;
				int r = (int)sqrt(dx*dx + dy*dy);
				if (r < outer_radius_pixels)
				{
					int r_tilted = (int)(sqrt(dx*dx + dy*dy) * cos(mirror_angle));
					n = (y * img_width) + x;

					angle = 0;
					if (r > 0) angle = (float)atan2(dx,dy);

					angle = -mirror_rotation - angle;

				    float xx0 = calibration_radius[r_tilted*4]*sin(angle);
				    float yy0 = calibration_radius[r_tilted*4]*cos(angle);
					float zz0 = calibration_radius[r_tilted*4+1];

					// tilt
					float xx1 = (float)((cos(mirror_angle2) * xx0) - (sin(mirror_angle2) * zz0));
					float yy1 = yy0;
					float zz1 = (float)((sin(mirror_angle) * xx0) + (cos(mirror_angle) * zz0));

					// rotate
					float xx2 = (float)((cos(mirror_rotation2) * xx1) - (sin(mirror_rotation2) * yy1));
					float yy2 = (float)((sin(mirror_rotation) * xx1) + (cos(mirror_rotation) * yy1));
					float zz2 = zz1 + camera_height_mm;

					float ix=0, iy=0;
					if (no_of_mirrors > 1) {
						if (intersection(
								bounding_box_tx,dist_to_mirror_backing+camera_height_mm, bounding_box_bx,dist_to_mirror_backing+camera_height_mm,
								xx2, zz2, 0, camera_height_mm, ix, iy)) {
							xx2 = ix;
							zz2 = iy;
						}
						if (intersection(
								bounding_box_ty,dist_to_mirror_backing+camera_height_mm, bounding_box_by,dist_to_mirror_backing+camera_height_mm,
								yy2, zz2, 0, camera_height_mm, ix, iy)) {
							yy2 = ix;
							zz2 = iy;
						}
					}

					ray_map[n*6] = (int)xx2;
					ray_map[n*6+1] = (int)yy2;
					ray_map[n*6+2] = (int)zz2;

					xx0 = calibration_radius[r_tilted*4+2]*sin(angle);
					yy0 = calibration_radius[r_tilted*4+2]*cos(angle);
					zz0 = calibration_radius[r_tilted*4+3];

					// tilt
					xx1 = (float)((cos(mirror_angle2) * xx0) - (sin(mirror_angle2) * zz0));
					yy1 = yy0;
					zz1 = (float)((sin(mirror_angle) * xx0) + (cos(mirror_angle) * zz0));

					// rotate
					xx2 = (float)((cos(mirror_rotation2) * xx1) - (sin(mirror_rotation2) * yy1));
					yy2 = (float)((sin(mirror_rotation) * xx1) + (cos(mirror_rotation) * yy1));
					zz2 = zz1 + camera_height_mm;

					bool hits_backing = false;
					if (no_of_mirrors > 1) {
						if (intersection(
								bounding_box_tx,dist_to_mirror_backing+camera_height_mm, bounding_box_bx,dist_to_mirror_backing+camera_height_mm,
								xx2, zz2, ray_map[n*6], ray_map[n*6+2], ix, iy)) {
							xx2 = ix;
							zz2 = iy;
							hits_backing = true;
						}
						if (intersection(
								bounding_box_ty,dist_to_mirror_backing+camera_height_mm, bounding_box_by,dist_to_mirror_backing+camera_height_mm,
								yy2, zz2, ray_map[n*6+1], ray_map[n*6+2], ix, iy)) {
							yy2 = ix;
							zz2 = iy;
							hits_backing = true;
						}
					}

					ray_map[n*6+3] = (int)xx2;
					ray_map[n*6+4] = (int)yy2;
					ray_map[n*6+5] = (int)zz2;

					mirror_map[n] = (unsigned char)(mirror+1);

					if (no_of_mirrors > 1) {
						if (!hits_backing) {
							if (ray_map[n*6+5] < ray_map[n*6+2]) {
								intersection(
									0,0, 100,0,
									ray_map[n*6], ray_map[n*6+2], ray_map[n*6+3], ray_map[n*6+5], ix, iy);
								ray_map[n*6+3] = ix;
								ray_map[n*6+5] = iy;

								intersection(
									0,0, 100,0,
									ray_map[n*6+1], ray_map[n*6+2], ray_map[n*6+4], ray_map[n*6+5], ix, iy);
								ray_map[n*6+4] = ix;
								ray_map[n*6+5] = iy;
							}
							else {
								intersection(
									0,(dist_to_mirror_backing+camera_height_mm)*2, 100,(dist_to_mirror_backing+camera_height_mm)*2,
									ray_map[n*6], ray_map[n*6+2], ray_map[n*6+3], ray_map[n*6+5], ix, iy);
								ray_map[n*6+3] = ix;
								ray_map[n*6+5] = iy;

								intersection(
									0,(dist_to_mirror_backing+camera_height_mm)*2, 100,(dist_to_mirror_backing+camera_height_mm)*2,
									ray_map[n*6+1], ray_map[n*6+2], ray_map[n*6+4], ray_map[n*6+5], ix, iy);
								ray_map[n*6+4] = ix;
								ray_map[n*6+5] = iy;
							}
						}

						for (int mirror2 = 0; mirror2 < no_of_mirrors; mirror2++) {

							if (mirror2 != mirror) {
								if (intersection_ray_sphere(
									ray_map[n*6],
									ray_map[n*6+1],
									ray_map[n*6+2],
									ray_map[n*6+3],
									ray_map[n*6+4],
									ray_map[n*6+5],
									mirror_position[mirror2*2],
									mirror_position[mirror2*2+1],
									dist_to_mirror_backing+camera_height_mm,
									mirror_diameter*0.5f) > 1) {

									ray_map[n*6]=0;
									ray_map[n*6+1]=0;
									ray_map[n*6+2]=0;
									ray_map[n*6+3]=0;
									ray_map[n*6+4]=0;
									ray_map[n*6+5]=0;
								}
							}
						}
					}

				}

			}
		}
	}

    delete[] calibration_radius;
    delete[] lookup_pixel_x;
    delete[] lookup_ray;
    delete[] lookup_angle;
}


void omni::show_ray_map_side(
	unsigned char* img,
	int img_width,
	int img_height,
	int max_height_mm,
	int focal_length_mm,
	int camera_height_mm,
	bool show_all)
{
	for (int i = 0; i < img_width*img_height*3; i++)
		img[i]=255;

    int camera_height_y = camera_height_mm * img_height / max_height_mm;

    int start_y = img_height/2;
    int end_y = (img_height/2)+1;

    if (show_all) {
    	start_y = 0;
    	end_y = img_height;
    }

    for (int y = start_y; y < end_y; y+=2) {
		int n = (y * img_width)*6;
		for (int x = 0; x < img_width; x+=4, n+=6*4) {
			float start_x = ray_map[n+1];
			float start_z = ray_map[n+2];
			float end_x = ray_map[n+4];
			float end_z = ray_map[n+5];

			int x0 = (img_width/2) + (int)(start_x * img_width / max_height_mm);
			int y0 = img_height-1-(int)(start_z * img_height / max_height_mm);

			int x1 = (img_width/2) + (int)(end_x * img_width / max_height_mm);
			int y1 = img_height-1-(int)(end_z * img_height / max_height_mm);

			drawing::drawLine(img,img_width,img_height,x0,y0,x1,y1,0,0,0,0,false);
			drawing::drawLine(img,img_width,img_height,x0,y0,img_width/2,img_height-1-camera_height_y,0,0,0,0,false);
		}
    }
    int focal_plane_y = img_height-1-camera_height_y-(focal_length_mm * img_height / max_height_mm);
    drawing::drawLine(img,img_width,img_height,img_width*48/100,focal_plane_y,img_width*52/100,focal_plane_y,255,0,0,0,false);
}

void omni::show_ray_map_above(
	unsigned char* img,
	int img_width,
	int img_height,
	int max_radius_mm)
{
	for (int i = 0; i < img_width*img_height*3; i++)
		img[i]=255;

    for (int y = 0; y < img_height; y+=4) {
		int n = (y * img_width)*6;
		for (int x = 0; x < img_width; x+=4, n+=6*4) {
			float start_x = ray_map[n];
			float start_y = ray_map[n+1];
			float end_x = ray_map[n+3];
			float end_y = ray_map[n+4];

			int x0 = (img_width/2) + (int)(start_x * img_height / max_radius_mm);
			int y0 = (img_height/2) + (int)(start_y * img_height / max_radius_mm);

			int x1 = (img_width/2) + (int)(end_x * img_height / max_radius_mm);
			int y1 = (img_height/2) + (int)(end_y * img_height / max_radius_mm);

			drawing::drawLine(img,img_width,img_height,x0,y0,x1,y1,0,0,0,0,false);
			drawing::drawLine(img,img_width,img_height,x0,y0,img_width/2,img_height/2,0,0,0,0,false);
		}
    }
}

void omni::show_ray_pixels(
	unsigned char* img,
	int img_width,
	int img_height)
{
	const int max_dist = 64;
	const int max_dist2 = 75;
	const int max_dist3 = 100;
	int n = 0,n2 = 0;
    for (int y = 0; y < img_height; y++) {
		for (int x = 0; x < img_width; x++, n+=6, n2+=3) {
			if ((ray_map[n]!=0) || (ray_map[n+1]!=0)) {
				float dist = (float)sqrt(ray_map[n]*ray_map[n] + ray_map[n+1]*ray_map[n+1]);

				if (dist < max_dist) {
					img[n2+1] = (unsigned char)255;
				}
				else {
					if (dist < max_dist2) {
						img[n2+1] = (unsigned char)150;
					}
					else {
						if (dist < max_dist3) {
							img[n2+1] = (unsigned char)100;
						}
					}
				}

			}
		}
    }
}

void omni::show_ray_directions(
	unsigned char* img,
	int img_width,
	int img_height)
{
	int n = 0,n2 = 0;
    for (int y = 0; y < img_height; y++) {
		for (int x = 0; x < img_width; x++, n+=6, n2+=3) {
			if ((ray_map[n]!=0) || (ray_map[n+1]!=0)) {
				int angle = (int)((float)atan2(ray_map[n], ray_map[n+1])*180/3.1415927f)/5;

				if (angle == 8) {
					img[n2+2] = (unsigned char)255;
				}
				if (angle == 9) {
					img[n2+2] = (unsigned char)255;
					img[n2] = (unsigned char)255;
				}

				if (angle == 26) {
					img[n2+1] = (unsigned char)255;
				}
				if (angle == 27) {
					img[n2] = (unsigned char)255;
				}

				if (angle == -8) {
					img[n2+1] = (unsigned char)255;
					img[n2+2] = (unsigned char)255;
				}
				if (angle == -9) {
					img[n2+2] = (unsigned char)255;
				}

				if (angle == -26) {
					img[n2] = (unsigned char)255;
				}
				if (angle == -27) {
					img[n2] = (unsigned char)255;
					img[n2+1] = (unsigned char)255;
				}



/*
				switch(angle % 5) {
			    case 0: {
			    	img[n2] = (unsigned char)255;
			    	break;
			    }
			    case 1: {
			    	img[n2+1] = (unsigned char)255;
			    	break;
			    }
			    case 2: {
			    	img[n2+2] = (unsigned char)255;
			    	break;
			    }
			    case 3: {
			    	img[n2+1] = (unsigned char)255;
			    	img[n2+2] = (unsigned char)255;
			    	break;
			    }
			    case 4: {
			    	img[n2] = (unsigned char)255;
			    	img[n2+2] = (unsigned char)255;
			    	break;
			    }
				}
*/
			}
		}
    }
}

void omni::show_ground_plane(
    unsigned char* img,
    int img_width,
    int img_height,
    int max_radius_mm)
{
	int min_x = -max_radius_mm;
	int max_x = max_radius_mm;
	int min_y = -max_radius_mm*img_height/img_width;
	int max_y = max_radius_mm*img_height/img_width;

	if (img_buffer == NULL) {
		img_buffer = new unsigned char[img_width*img_height*3];
	}
	memset((void*)img_buffer,'\0',img_width*img_height*3);

	int n = 0;
	for (int y = 0; y < img_height; y++) {
		for (int x = 0; x < img_width; x++, n += 6) {
			if ((!((ray_map[n+3] == 0) && (ray_map[n+4] == 0))) &&
				(ray_map[n+3] > min_x) && (ray_map[n+3] < max_x) &&
				(ray_map[n+4] > min_y) && (ray_map[n+4] < max_y) &&
				(ray_map[n+5] == 0)) {
				int xx = (ray_map[n+3] - min_x) * (img_width-1) / (max_x-min_x);
				if ((xx > -1) && (xx < img_width)) {

					int yy = (ray_map[n+4] - min_y) * (img_height-1) / (max_y-min_y);
					if ((yy > -1) && (yy < img_height)) {

						int xx2 = xx+8;
						//if (ray_map[n+3+6] != 0) xx2 = (ray_map[n+3+6] - min_x) * (img_width-1) / (max_x-min_x);
						if ((xx2 > 0) && (xx2 < img_width)) {

							int yy2 = yy+8;
							//if (ray_map[n+4+6+(img_width*6)] != 0) yy2 = (ray_map[n+4+6+(img_width*6)] - min_y) * (img_height-1) / (max_y-min_y);
							if ((yy2 > 0) && (yy2 < img_height)) {

								if ((xx2 == 0) && (yy2 ==0)) {
									xx2 = xx;
									yy2 = yy;
								}

								int n0 = ((y * img_width) + x)*3;
								for (int yyy = yy; yyy <= yy2; yyy++) {
									for (int xxx = xx; xxx <= xx2; xxx++) {
										int n2 = ((yyy * img_width) + xxx)*3;
										if (img_buffer[n2] > 0) break;
										img_buffer[n2] = img[n0];
										img_buffer[n2+1] = img[n0+1];
										img_buffer[n2+2] = img[n0+2];
									}
								}

							}

						}

					}

				}
			}
		}
	}
	memcpy((void*)img, (void*)img_buffer, img_width*img_height*3);
}

void omni::show_ground_plane_features(
    unsigned char* img,
    int img_width,
    int img_height,
    int max_radius_mm,
	int no_of_feats_vertical,
	int no_of_feats_horizontal)
{
	int min_x = -max_radius_mm;
	int max_x = max_radius_mm;
	int min_y = -max_radius_mm*img_height/img_width;
	int max_y = max_radius_mm*img_height/img_width;

	if (img_buffer == NULL) {
		img_buffer = new unsigned char[img_width*img_height*3];
	}
	memset((void*)img_buffer,'\0',img_width*img_height*3);

	/* vertically oriented features */
	int row = 0;
	int feats_remaining = features_per_row[row];

	for (int f = 0; f < no_of_feats_vertical; f++, feats_remaining--) {

		int x = feature_x[f] / OMNI_SUB_PIXEL;
		int y = (int)((4 + (row * OMNI_VERTICAL_SAMPLING)));
		if ((x > -1) && (x < img_width) &&
			(y > -1) && (y < img_height)) {
			int n = ((y*img_width)+x)*6;
			if ((ray_map[n+5] == 0) &&
				(ray_map[n+3] > min_x) && (ray_map[n+3] < max_x) &&
				(ray_map[n+4] > min_y) && (ray_map[n+4] < max_y)) {
				int xx = (ray_map[n+3] - min_x) * img_width / (max_x-min_x);
				int yy = (ray_map[n+4] - min_y) * img_height / (max_y-min_y);
				int n2 = ((yy * img_width) + xx)*3;
				img_buffer[n2] = 255;
				img_buffer[n2+1] = 255;
				img_buffer[n2+2] = 255;
			}
		}

		/* move to the next row */
		if (feats_remaining <= 0) {
			row++;
			feats_remaining = features_per_row[row];
		}
	}

	/* horizontally oriented features */
	int col = 0;
	feats_remaining = features_per_col[col];

	for (int f = 0; f < no_of_feats_horizontal; f++, feats_remaining--) {

		int y = feature_y[f] / OMNI_SUB_PIXEL;
		int x = (int)((4 + (col * OMNI_HORIZONTAL_SAMPLING)));
		if ((x > -1) && (x < img_width) &&
			(y > -1) && (y < img_height)) {
			int n = ((y*img_width)+x)*6;
			if ((ray_map[n+5] == 0) &&
				(ray_map[n+3] > min_x) && (ray_map[n+3] < max_x) &&
				(ray_map[n+4] > min_y) && (ray_map[n+4] < max_y)) {
				int xx = (ray_map[n+3] - min_x) * img_width / (max_x-min_x);
				int yy = (ray_map[n+4] - min_y) * img_height / (max_y-min_y);
				int n2 = ((yy * img_width) + xx)*3;
				img_buffer[n2] = 255;
				img_buffer[n2+1] = 255;
				img_buffer[n2+2] = 255;
			}
		}

	    /* move to the next column */
		if (feats_remaining <= 0) {
			col++;
			feats_remaining = features_per_col[col];
		}
	}

	memcpy((void*)img, (void*)img_buffer, img_width*img_height*3);
}

void omni::show_radial_lines(
    unsigned char* img,
    int img_width,
    int img_height,
    int max_radius_mm)
{
	show_ground_plane(img, img_width, img_height, max_radius_mm);

	for (int i = 0; i < no_of_radial_lines; i++) {
		int x0 = radial_lines[i*5+1];
		int y0 = radial_lines[i*5+2];
		int x1 = radial_lines[i*5+3];
		int y1 = radial_lines[i*5+4];
		drawing::drawLine(img,img_width,img_height,x0,y0,x1,y1,255,0,0,1,false);
	}
}

void omni::detect_radial_lines(
    unsigned char* img,
    int img_width,
    int img_height,
    int max_radius_mm,
	int no_of_feats_vertical,
	int no_of_feats_horizontal,
	int threshold)
{
	if (feature_radius_index == NULL) {
		feature_radius_index = new unsigned char[OMNI_MAX_FEATURES];
	}
	if (radial_lines == NULL) {
		radial_lines = new int[360/2*5];
	}
	int feature_index = 0;
	no_of_radial_lines = 0;

	const int radii = 360/2;
	int* bucket = new int[radii+1];
	int* mean = new int[radii*2+2];
	memset((void*)bucket,'\0',radii*sizeof(int));
	memset((void*)mean,'\0',radii*2*sizeof(int));
	int cx = img_width/2;
	int cy = img_height/2;

	int min_x = -max_radius_mm;
	int max_x = max_radius_mm;
	int min_y = -max_radius_mm*img_height/img_width;
	int max_y = max_radius_mm*img_height/img_width;

	/* vertically oriented features */
	int row = 0;
	int feats_remaining = features_per_row[row];

	for (int f = 0; f < no_of_feats_vertical; f++, feats_remaining--, feature_index++) {

		if (feature_index < OMNI_MAX_FEATURES) {
			feature_radius_index[feature_index] = 0;

			int x = feature_x[f] / OMNI_SUB_PIXEL;
			int y = (int)((4 + (row * OMNI_VERTICAL_SAMPLING)));
			if ((x > -1) && (x < img_width) &&
				(y > -1) && (y < img_height)) {
				int n = ((y*img_width)+x)*6;
				if ((ray_map[n+5] == 0) &&
					(ray_map[n+3] > min_x) && (ray_map[n+3] < max_x) &&
					(ray_map[n+4] > min_y) && (ray_map[n+4] < max_y)) {
					int xx = (ray_map[n+3] - min_x) * img_width / (max_x-min_x);
					int yy = (ray_map[n+4] - min_y) * img_height / (max_y-min_y);
					int dx = xx - cx;
					int dy = yy - cy;
					int b = (int)((float)atan2(dx,dy) * (radii-1) / (3.1415927f*2));
					if (b < 0) b += radii;
					if (b >= radii) b -= radii;
					feature_radius_index[feature_index] = (unsigned char)(1+b);
					bucket[b]++;
					mean[b*2] += xx;
					mean[b*2+1] += yy;
				}
			}
		}

		/* move to the next row */
		if (feats_remaining <= 0) {
			row++;
			feats_remaining = features_per_row[row];
		}
	}

	/* horizontally oriented features */
	int col = 0;
	feats_remaining = features_per_col[col];

	for (int f = 0; f < no_of_feats_horizontal; f++, feats_remaining--, feature_index++) {

		if (feature_index < OMNI_MAX_FEATURES) {
		    feature_radius_index[feature_index] = 0;

			int y = feature_y[f] / OMNI_SUB_PIXEL;
			int x = (int)((4 + (col * OMNI_HORIZONTAL_SAMPLING)));
			if ((x > -1) && (x < img_width) &&
				(y > -1) && (y < img_height)) {
				int n = ((y*img_width)+x)*6;
				if ((ray_map[n+5] == 0) &&
					(ray_map[n+3] > min_x) && (ray_map[n+3] < max_x) &&
					(ray_map[n+4] > min_y) && (ray_map[n+4] < max_y)) {
					int xx = (ray_map[n+3] - min_x) * img_width / (max_x-min_x);
					int yy = (ray_map[n+4] - min_y) * img_height / (max_y-min_y);
					int dx = xx - cx;
					int dy = yy - cy;
					int b = (int)((float)atan2(dx,dy) * (radii-1) / (3.1415927f*2));
					if (b < 0) b += radii;
					if (b >= radii) b -= radii;
					feature_radius_index[feature_index] = (unsigned char)(1+b);
					bucket[b]++;
					mean[b*2] += xx;
					mean[b*2+1] += yy;
				}
			}
		}

	    /* move to the next column */
		if (feats_remaining <= 0) {
			col++;
			feats_remaining = features_per_col[col];
		}
	}

	int average_bucket_hits = 0;
	int ctr = 0;

	// non maximal suppression
	for (int i = 0; i < radii-1; i++) {
		if (bucket[i+1] < bucket[i]) {
			bucket[i+1] = 0;
		}
		else {
			bucket[i] = 0;
		}
	}

	for (int i = 0; i < radii; i++) {
		if (bucket[i] > threshold) {
		    average_bucket_hits += bucket[i];
		    ctr++;
		}
	}
	if (ctr > 0) average_bucket_hits /= ctr;
	for (int i = 0; i < radii; i++) {
		if ((bucket[i] > threshold) &&
			(bucket[i] > average_bucket_hits)) {

			int mid_x = (mean[i*2] / bucket[i]) - cx;
			int mid_y = (mean[i*2+1] / bucket[i]) - cy;
			int mid_r = mid_x*mid_x + mid_y*mid_y;
			int x0 = 0;
			int y0 = 0;
			int hits0 = 0;
			int x1 = 0;
			int y1 = 0;
			int hits1 = 0;

			feature_index = 0;

			/* vertically oriented features */
			row = 0;
			feats_remaining = features_per_row[row];

			for (int f = 0; f < no_of_feats_vertical; f++, feats_remaining--, feature_index++) {

				if (feature_index < OMNI_MAX_FEATURES) {
					int x = feature_x[f] / OMNI_SUB_PIXEL;
					int y = (int)((4 + (row * OMNI_VERTICAL_SAMPLING)));
					if ((x > -1) && (x < img_width) &&
						(y > -1) && (y < img_height)) {
						int n = ((y*img_width)+x)*6;
						if ((ray_map[n+5] == 0) &&
							(ray_map[n+3] > min_x) && (ray_map[n+3] < max_x) &&
							(ray_map[n+4] > min_y) && (ray_map[n+4] < max_y)) {
							int xx = (ray_map[n+3] - min_x) * img_width / (max_x-min_x);
							int yy = (ray_map[n+4] - min_y) * img_height / (max_y-min_y);
							int dx = xx - cx;
							int dy = yy - cy;
							int b = (int)feature_radius_index[feature_index]-1;
							if (b > -1) {
								if ((b >= i-1) && (b <= i+1)) {
									int r = dx*dx + dy*dy;
									if (r < mid_r) {
										x0 += xx;
										y0 += yy;
										hits0++;
									}
									else {
										x1 += xx;
										y1 += yy;
										hits1++;
									}
								}
							}
						}
					}
				}

				/* move to the next row */
				if (feats_remaining <= 0) {
					row++;
					feats_remaining = features_per_row[row];
				}
			}

			/* horizontally oriented features */
			col = 0;
			feats_remaining = features_per_col[col];

			for (int f = 0; f < no_of_feats_horizontal; f++, feats_remaining--, feature_index++) {
				if (feature_index < OMNI_MAX_FEATURES) {
					int y = feature_y[f] / OMNI_SUB_PIXEL;
					int x = (int)((4 + (col * OMNI_HORIZONTAL_SAMPLING)));
					if ((x > -1) && (x < img_width) &&
						(y > -1) && (y < img_height)) {
						int n = ((y*img_width)+x)*6;
						if ((ray_map[n+5] == 0) &&
							(ray_map[n+3] > min_x) && (ray_map[n+3] < max_x) &&
							(ray_map[n+4] > min_y) && (ray_map[n+4] < max_y)) {
							int xx = (ray_map[n+3] - min_x) * img_width / (max_x-min_x);
							int yy = (ray_map[n+4] - min_y) * img_height / (max_y-min_y);
							int dx = xx - cx;
							int dy = yy - cy;

							int b = (int)feature_radius_index[feature_index]-1;
							if (b > -1) {
								if ((b >= i-1) && (b <= i+1)) {
									int r = dx*dx + dy*dy;
									if (r < mid_r) {
										x0 += xx;
										y0 += yy;
										hits0++;
									}
									else {
										x1 += xx;
										y1 += yy;
										hits1++;
									}
								}
							}
						}
					}
				}

			    /* move to the next column */
				if (feats_remaining <= 0) {
					col++;
					feats_remaining = features_per_col[col];
				}
			}


			if ((hits0 > 0) && (hits1 > 0)) {
				x0 /= hits0;
				y0 /= hits0;
				x1 /= hits1;
				y1 /= hits1;
				x1 = x0 + (x1 - x0)* 4;
				y1 = y0 + (y1 - y0)* 4;
				radial_lines[no_of_radial_lines*5] = i * 360 / radii;
				radial_lines[no_of_radial_lines*5+1] = x0;
				radial_lines[no_of_radial_lines*5+2] = y0;
				radial_lines[no_of_radial_lines*5+3] = x1;
				radial_lines[no_of_radial_lines*5+4] = y1;
				no_of_radial_lines++;
			}
		}
	}

	delete[] bucket;
	delete[] mean;
}


void omni::compass(
	int max_variance_degrees)
{
	if (prev_radial_lines == NULL) {
		prev_radial_lines = new int[360/2*5];
        memcpy((void*)prev_radial_lines, radial_lines, no_of_radial_lines*5*sizeof(int));
        prev_no_of_radial_lines = no_of_radial_lines;
	}

	int offset_degrees = 0;
	int best_score = 99999;
    for (int variance = -max_variance_degrees; variance <= max_variance_degrees; variance++) {
    	int score = 0;
        for (int i = 0; i < no_of_radial_lines; i++) {
        	int min = 9999;
        	int angle = radial_lines[i*5] + variance;
        	if (angle < 0) angle += 360;
        	if (angle >= 360) angle -= 360;
        	for (int j = 0; j < prev_no_of_radial_lines; j++) {
        		int diff = prev_radial_lines[j*5] - angle;
        		if (diff < 0) diff = -diff;
        		if (diff < min) min = diff;
        	}
        	score += min;
        }
        if (score < best_score) {
        	best_score = score;
        	offset_degrees = variance;
        }
    }

    if (offset_degrees != 0) {
        //printf("offset_degrees = %d\n", offset_degrees);

        memcpy((void*)prev_radial_lines, radial_lines, no_of_radial_lines*5*sizeof(int));
        prev_no_of_radial_lines = no_of_radial_lines;
    }
}

void omni::unwarp(
	unsigned char* img,
	int img_width,
	int img_height,
	int bytes_per_pixel)
{
	if (unwarp_lookup != NULL) {
		if (img_buffer == NULL) {
			img_buffer = new unsigned char[img_width*img_height*3];
		}

		for (int i = 0; i < img_width*img_height; i++) {
			int j = unwarp_lookup[i] * bytes_per_pixel;
			for (int col = 0; col < bytes_per_pixel; col++) {
			    img_buffer[i*bytes_per_pixel+col] = img[j+col];
			}
		}
		memcpy((void*)img, (void*)img_buffer, img_width*img_height*bytes_per_pixel);
	}
}

void omni::unwarp_features(
	unsigned char* img,
	int img_width,
	int img_height,
	int bytes_per_pixel,
	int no_of_feats_vertical,
	int no_of_feats_horizontal)
{
	if (img_buffer == NULL) {
		img_buffer = new unsigned char[img_width*img_height*3];
	}
	memset((void*)img_buffer,'\0', img_width*img_height*3);

	int max = img_width*(img_height-1)*bytes_per_pixel;

	/* vertically oriented features */
	int row = 0;
	int feats_remaining = features_per_row[row];

	for (int f = 0; f < no_of_feats_vertical; f++, feats_remaining--) {

		int x = feature_x[f] / OMNI_SUB_PIXEL;
		int y = (int)((4 + (row * OMNI_VERTICAL_SAMPLING)));
		if ((x > -1) && (x < img_width) &&
			(y > -1) && (y < img_height)) {
			int n = (y*img_width)+x;
			int n2 = unwarp_lookup_reverse[n] * bytes_per_pixel;
			if ((n2 > 0) && (n2 < max)) {
			    for (int col = 0; col < bytes_per_pixel; col++) {
			    	img_buffer[n2+col] = 255;
			    }
			}
		}

		/* move to the next row */
		if (feats_remaining <= 0) {
			row++;
			feats_remaining = features_per_row[row];
		}
	}

	/* horizontally oriented features */
	int col = 0;
	feats_remaining = features_per_col[col];

	for (int f = 0; f < no_of_feats_horizontal; f++, feats_remaining--) {

		int y = feature_y[f] / OMNI_SUB_PIXEL;
		int x = (int)((4 + (col * OMNI_HORIZONTAL_SAMPLING)));
		if ((x > -1) && (x < img_width) &&
			(y > -1) && (y < img_height)) {
			int n = (y*img_width)+x;
			int n2 = unwarp_lookup_reverse[n] * bytes_per_pixel;
			if ((n2 > 0) && (n2 < max)) {
			    for (int col = 0; col < bytes_per_pixel; col++) {
			    	img_buffer[n2+col] = 255;
			    }
			}
		}

		/* move to the next column */
		if (feats_remaining <= 0) {
			col++;
			feats_remaining = features_per_col[col];
		}
	}

    memcpy((void*)img, (void*)img_buffer, img_width*img_height*bytes_per_pixel);
}

bool omni::save_configuration(
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
	float range)
{
	bool saved = false;
	FILE *file = fopen(filename.c_str(), "w");
	if (file != NULL) {

		fprintf(file,"Number of mirrors %d\n", no_of_mirrors);
		fprintf(file,"Image coordinates as percentages\n");
		for (int mirror = 0; mirror < no_of_mirrors; mirror++) {
			fprintf(file,"%.2f,%.2f\n", mirror_position_pixels[mirror*2], mirror_position_pixels[mirror*2+1]);
		}
		fprintf(file,"\nReal coordinates in millimetres\n");
		for (int mirror = 0; mirror < no_of_mirrors; mirror++) {
			fprintf(file,"%.2f,%.2f\n", mirror_position[mirror*2], mirror_position[mirror*2+1]);
		}

		fprintf(file,"\nFocal length (mm) %.2f\n", focal_length);
		fprintf(file,"Mirror diameter (mm) %.2f\n", mirror_diameter);
		fprintf(file,"Outer radius (percent of image width) %.2f\n", outer_radius_percent);
		fprintf(file,"Inner radius (percent of image width) %.2f\n", inner_radius_percent);
		fprintf(file,"Distance from camera to mirror plane (mm) %.2f\n", dist_to_mirror_centre);
		fprintf(file,"Camera elevation (mm) %.2f\n", camera_height);
		fprintf(file,"Baseline (mm) %.2f\n", baseline);
		fprintf(file,"Mapping range (mm) %.2f\n", range);

		fclose(file);
		saved = true;
	}
	return(saved);
}

bool omni::load_configuration(
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
	float &range)
{
	bool loaded = false;
	FILE *file = fopen(filename.c_str(), "r");
	if (file != NULL) {

		float x=0,y=0;
		fscanf(file,"Number of mirrors %d\n", &no_of_mirrors);
		fscanf(file,"Image coordinates as percentages\n");
		for (int mirror = 0; mirror < no_of_mirrors; mirror++) {
			fscanf(file,"%f,%f\n", &x, &y);
			mirror_position_pixels[mirror*2] = x;
			mirror_position_pixels[mirror*2+1] = y;
		}
		fscanf(file,"\nReal coordinates in millimetres\n");
		for (int mirror = 0; mirror < no_of_mirrors; mirror++) {
			fscanf(file,"%f,%f\n", &x, &y);
			mirror_position[mirror*2] = x;
			mirror_position[mirror*2+1] = y;
		}

		fscanf(file,"\nFocal length (mm) %f\n", &focal_length);
		fscanf(file,"Mirror diameter (mm) %f\n", &mirror_diameter);
		fscanf(file,"Outer radius (percent of image width) %f\n", &outer_radius_percent);
		fscanf(file,"Inner radius (percent of image width) %f\n", &inner_radius_percent);
		fscanf(file,"Distance from camera to mirror plane (mm) %f\n", &dist_to_mirror_centre);
		fscanf(file,"Camera elevation (mm) %f\n", &camera_height);
		fscanf(file,"Baseline (mm) %f\n", &baseline);
		fscanf(file,"Mapping range (mm) %f\n", &range);

		fclose(file);
		loaded = true;
	}
	return(loaded);
}

/*!
 * \brief returns the minimum squared distance between two 3D rays
 */
float omni::min_distance_between_rays(
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
	float &z)
{
	float ux = ray1_x_end - ray1_x_start;
	float uy = ray1_y_end - ray1_y_start;
	float uz = ray1_z_end - ray1_z_start;

	float vx = ray2_x_end - ray2_x_start;
	float vy = ray2_y_end - ray2_y_start;
	float vz = ray2_z_end - ray2_z_start;

	float wx = ray1_x_start - ray2_x_start;
	float wy = ray1_y_start - ray2_y_start;
	float wz = ray1_z_start - ray2_z_start;

    float a = ux*ux + uy*uy + uz*uz;
    float b = ux*vx + uy*vy + uz*vz;
    float c = vx*vx + vy*vy + vz*vz;
    float d = ux*wx + uy*wy + uz*wz;
    float e = vx*wx + vy*wy + vz*wz;
    float D = a*c - b*b;
    float sc, sN, sD = D;
    float tc, tN, tD = D;

    // compute the line parameters of the two closest points
    if (D < 0.00000001f) { // the lines are almost parallel
        sN = 0.0f; // force using point P0 on segment S1
        sD = 1.0f; // to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    }
    else {                // get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if (sN < 0.0f) {       // sc < 0 => the s=0 edge is visible
            sN = 0.0f;
            tN = e;
            tD = c;
        }
        else if (sN > sD) {  // sc > 1 => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0f) {           // tc < 0 => the t=0 edge is visible
        tN = 0.0f;
        // recompute sc for this edge
        if (-d < 0.0f)
            sN = 0.0f;
        else if (-d > a)
            sN = sD;
        else {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD) {      // tc > 1 => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
        if ((-d + b) < 0.0f)
            sN = 0;
        else if ((-d + b) > a)
            sN = sD;
        else {
            sN = (-d + b);
            sD = a;
        }
    }
    // finally do the division to get sc and tc
    sc = (fabs(sN) < 0.00000001f ? 0.0 : sN / sD);
    tc = (fabs(tN) < 0.00000001f ? 0.0 : tN / tD);

    // get the difference of the two closest points
    dx = wx + (sc * ux) - (tc * vx);
    dy = wy + (sc * uy) - (tc * vy);
    dz = wz + (sc * uz) - (tc * vz);
    x = wx + (sc * x);
    y = wy + (sc * y);
    z = wz + (sc * z);
    return (dx*dx + dy*dy + dz*dz);
}
