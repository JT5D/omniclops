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

	// array storing x coordinates of detected features
	feature_x = new short int[OMNI_MAX_FEATURES];
	feature_y = new short int[OMNI_MAX_FEATURES];

	ray_map = NULL;
	mirror_lookup = NULL;
	mirror_map = NULL;
	feature_map = NULL;
	calibration_map = NULL;
	feature_radius_index = NULL;
	unwarp_lookup = NULL;
	unwarp_lookup_reverse = NULL;

	// array storing the number of features detected on each row
	features_per_row = new unsigned short int[OMNI_MAX_IMAGE_HEIGHT
			/ OMNI_VERTICAL_SAMPLING];
	features_per_col = new unsigned short int[OMNI_MAX_IMAGE_WIDTH
			/ OMNI_HORIZONTAL_SAMPLING];

	// buffer which stores sliding sum
	row_sum = new int[OMNI_MAX_IMAGE_WIDTH];

	// buffer used to find peaks in edge space
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
	delete[] feature_y;
	delete[] features_per_row;
	delete[] features_per_col;
	delete[] row_sum;
	delete[] row_peaks;
	delete[] temp_row_peaks;
	if (feature_map != NULL) {
		delete[] feature_map;
	}
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
	if (mirror_lookup != NULL) {
		delete[] mirror_lookup;
	}
	if (img_buffer != NULL)
		delete[] img_buffer;

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

    h = round((hConvert*31)/360);
    s = round(sConvert*255);
    v = round(vConvert*255);
}

void omni::voxel_paint(
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
	vector<short> &occupied_voxels)
{
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

	occupied_voxels.clear();

	int pixels = img_width*img_height;

	vector<unsigned int> voxel_histogram[grid_cells_x*grid_cells_y+1];
	vector<unsigned char> voxel_colours[grid_cells_x*grid_cells_y];
	vector<int> pixel_index[grid_cells_x*grid_cells_y];
	vector<unsigned char> mirror_index[grid_cells_x*grid_cells_y];
	int cam[no_of_mirrors];
	int colour_mean[no_of_mirrors*3];

	//int min_dist_cells = (int)(mirror_diameter_mm / grid_cell_dimension_mm);
	int min_dist_cells = (int)(dist_to_mirror_backing_mm / grid_cell_dimension_mm);

	int offset_x = (grid_cells_x/2) - (grid_centre_x_mm / grid_cell_dimension_mm);
	int offset_y = (grid_cells_y/2) - (grid_centre_y_mm / grid_cell_dimension_mm);
	//int offset_z = (grid_centre_z_mm / grid_cell_dimension_mm);

	memset((void*)img_occlusions,'\0',pixels);

	int layer_cell_index;
	int layer_dist_mm = (int)(grid_cell_dimension_mm * (min_dist_cells + 0.5f));
	//for (int layer = min_dist_cells; layer < grid_cells_z; layer++, layer_dist_mm += grid_cell_dimension_mm) {
	for (int layer = min_dist_cells; layer < min_dist_cells+1; layer++, layer_dist_mm += grid_cell_dimension_mm) {

		// paint colours for this layer
		int n = (pixels - 1) * 6;
		for (int i = pixels-1; i >= 0; i--, n -= 6) {
			if ((mirror_map[i] > 0) &&
				(img_occlusions[i] == 0)) {
				int mirror_idx = mirror_map[i] - 1;

				// calculate the start and end grid cell coordinates of the ray
				int ray_start_x_mm = ray_map[n];
				int ray_start_y_mm = ray_map[n+1];
				int ray_start_z_mm = ray_map[n+2];
				int ray_end_x_mm = ray_map[n+3];
				int ray_end_y_mm = ray_map[n+4];
				int ray_end_z_mm = ray_map[n+5];

				int dz = ray_end_z_mm - ray_start_z_mm;
				if (dz < 0) {
					int dz2 = ray_start_z_mm - (dist_to_mirror_backing_mm - layer_dist_mm);

					int x = ray_start_x_mm + ((ray_end_x_mm - ray_start_x_mm) * dz2 / dz);
					int cell_x = offset_x + (int)(x / grid_cell_dimension_mm);
					if ((cell_x >= 0) && (cell_x < grid_cells_x)) {
						int y = ray_start_y_mm + ((ray_end_y_mm - ray_start_y_mm) * dz2 / dz);
						int cell_y = offset_y + (int)(y / grid_cell_dimension_mm);
						if ((cell_y >= 0) && (cell_y < grid_cells_y)) {

							// colour of the ray
							unsigned char r = img[i*3 + 2];
							unsigned char g = img[i*3 + 1];
							unsigned char b = img[i*3];

							int histogram_index = (r/32)*64 + (g/32)*8 + (b/32);

							layer_cell_index = cell_y*grid_cells_x + cell_x;
							if ((int)voxel_histogram[layer_cell_index].size() == 0) {
								voxel_histogram[layer_cell_index].resize(16*no_of_mirrors,0);
							}
							voxel_histogram[layer_cell_index][(histogram_index/32) + (mirror_idx*16)] |= (unsigned int)pow(2,histogram_index % 32);

							voxel_colours[layer_cell_index].push_back(r);
							voxel_colours[layer_cell_index].push_back(g);
							voxel_colours[layer_cell_index].push_back(b);
							pixel_index[layer_cell_index].push_back(i);
							mirror_index[layer_cell_index].push_back(mirror_idx);
						}
					}
				}
			}
		}

		// determine colour consistency
		layer_cell_index = 0;
		for (int gy = 0; gy < grid_cells_y; gy++) {
			for (int gx = 0; gx < grid_cells_x; gx++, layer_cell_index++) {

				if ((int)pixel_index[layer_cell_index].size() > 0) {

					memset((void*)cam,'\0',no_of_mirrors*sizeof(int));
					for (int sample = (int)pixel_index[layer_cell_index].size()-1; sample >= 0; sample--) {
						int mirror = mirror_index[layer_cell_index][sample];
						cam[mirror]++;
					}

					int correlation = 0;
					int anticorrelation = 0;

					int no_of_mirror_observations = 0;
					for (int mirror0 = 0; mirror0 < no_of_mirrors; mirror0++) {
						if (cam[mirror0] > 0) {
							no_of_mirror_observations++;
							for (int mirror1 = mirror0+1; mirror1 < no_of_mirrors; mirror1++) {
								if (cam[mirror1] > 0) {
                                    for (int idx = 0; idx < 16; idx++) {

                                        int n2 = voxel_histogram[layer_cell_index][idx + (mirror0*16)];
                                        unsigned int anti = 0;
                                        for (int bit = 0; bit < 32; bit++) {
                                            anti <<= 1;
                                            anti |= n2 & 1;
                                            n2 >>= 1;
                                        }

                                    	unsigned int match = voxel_histogram[layer_cell_index][idx+(mirror0*16)] & voxel_histogram[layer_cell_index][idx+(mirror1*16)];
                                    	if (match > 0) {
                                    		correlation +=
                                                BitsSetTable256[match & 0xff] +
                                                BitsSetTable256[(match >> 8) & 0xff] +
                                                BitsSetTable256[(match >> 16) & 0xff] +
                                                BitsSetTable256[match >> 24];
                                    	}
                                    	match = anti & voxel_histogram[layer_cell_index][idx+(mirror1*16)];
                                    	if (match > 0) {
                                    		anticorrelation +=
                                                BitsSetTable256[match & 0xff] +
                                                BitsSetTable256[(match >> 8) & 0xff] +
                                                BitsSetTable256[(match >> 16) & 0xff] +
                                                BitsSetTable256[match >> 24];
                                    	}

                                    }
								}
							}
						}
					}


					int test_r=0;
					int test_g=0;
					int test_b=0;

					memset((void*)colour_mean,'\0',no_of_mirrors*3*sizeof(int));
					for (int sample = (int)pixel_index[layer_cell_index].size()-1; sample >= 0; sample--) {
						int mirror = mirror_index[layer_cell_index][sample];
						unsigned char r = voxel_colours[layer_cell_index][sample*3];
						unsigned char g = voxel_colours[layer_cell_index][sample*3+1];
						unsigned char b = voxel_colours[layer_cell_index][sample*3+2];

						colour_mean[mirror*3] += (int)voxel_colours[layer_cell_index][sample*3];
						colour_mean[mirror*3+1] += (int)voxel_colours[layer_cell_index][sample*3+1];
						colour_mean[mirror*3+2] += (int)voxel_colours[layer_cell_index][sample*3+2];
						test_r = (int)voxel_colours[layer_cell_index][sample*3];
						test_g = (int)voxel_colours[layer_cell_index][sample*3+1];
						test_b = (int)voxel_colours[layer_cell_index][sample*3+2];
					}
					if ((no_of_mirror_observations >= no_of_mirrors-1) &&
						(correlation >= min_correlation)) {
						// we have a winner - add it to the occlusions
						for (int j = (int)pixel_index[layer_cell_index].size()-1; j >= 0; j--) {
							img_occlusions[pixel_index[layer_cell_index][j]] = 1;
						}
						// create a voxel
						occupied_voxels.push_back((short)gx);
						occupied_voxels.push_back((short)gy);
						occupied_voxels.push_back((short)layer);
						//occupied_voxels.push_back((short)mean_r);
						//occupied_voxels.push_back((short)mean_g);
						//occupied_voxels.push_back((short)mean_b);
						occupied_voxels.push_back((short)test_r);
						occupied_voxels.push_back((short)test_g);
						occupied_voxels.push_back((short)test_b);
					}

					voxel_colours[layer_cell_index].clear();
					pixel_index[layer_cell_index].clear();
					mirror_index[layer_cell_index].clear();
				}
			}
		}

	}
}

void omni::show_plane_occupancy(
	unsigned char* img,
	int img_width,
	int img_height,
	int no_of_planes,
	int* plane_occupancy)
{
	memset((void*)img,'\0',img_width*img_height*3);

	int max = 0;
	for (int i = 0; i < no_of_planes; i++) {
		if (plane_occupancy[i] > max) max = plane_occupancy[i];
	}

	if (max > 0) {
		max  = max*120/100;
		int prev_x = 0;
		int prev_y = 0;
		for (int i = 0; i < no_of_planes; i++) {
			int x = i * img_width / no_of_planes;
			int y = img_height-1-(plane_occupancy[i] * img_height / max);
			if (i > 0) {
				drawing::drawLine(img,img_width,img_height,prev_x,prev_y,x,y,0,255,0,1,false);
			}
			prev_x = x;
			prev_y = y;
		}
	}
}


void omni::show_voxels(
	unsigned char* img,
	int img_width,
	int img_height,
	vector<short> &voxels,
	int voxel_radius_pixels,
	int view_type)
{
    int tx = 9999;
    int ty = 9999;
    int tz = 9999;
    int bx = 0;
    int by = 0;
    int bz = 0;

    // get the bounding box
    for (int i = (int)voxels.size()-6; i >= 0; i-=6) {
    	int x = voxels[i];
    	int y = voxels[i+1];
    	int z = voxels[i+2];
    	if (x < tx) tx = x;
    	if (x > bx) bx = x;
    	if (y < ty) ty = y;
    	if (y > by) by = y;
    	if (z < tz) tz = z;
    	if (z > bz) bz = z;
    }

    if (bz == tz) bz = tz+1;

    memset((void*)img, '\0',img_width*img_height*3);
    if ((bx - tx > 0) && (by - ty > 0) && (bz - tz > 0)) {
		for (int i = (int)voxels.size()-6; i >= 0; i-=6) {
			int x = (int)voxels[i];
			int y = (int)voxels[i+1];
			int z = (int)voxels[i+2];
			switch(view_type) {
				case 0: {
					x = (x - tx) * img_height / (bx - tx);
					y = (y - ty) * img_height / (by - ty);
					break;
				}
				case 1: {
					x = (x - tx) * img_height / (bx - tx);
					y = (z - tz) * img_height / (bz - tz);
					break;
				}
				case 2: {
					x = (y - ty) * img_height / (by - ty);
					y = (z - tz) * img_height / (bz - tz);
					break;
				}
				case 3: {
					int x0 = ((x - tx) * img_height / ((bx - tx)*2));
					int y0 = (img_height/4) + ((y - ty) * img_height / ((by - ty)*2));

					for (int yy = y0-voxel_radius_pixels; yy <= y0+voxel_radius_pixels; yy++) {
						for (int xx = x0-voxel_radius_pixels; xx <= x0+voxel_radius_pixels; xx++) {
							if ((xx > -1) && (xx < img_width) &&
								(yy > -1) && (yy < img_height)) {
								int n = ((yy * img_width) + xx) * 3;
								img[n+2] = (unsigned char)voxels[i+3];
								img[n+1] = (unsigned char)voxels[i+4];
								img[n] = (unsigned char)voxels[i+5];
							}
						}
					}

					x0 = (img_height/2) + ((x - tx) * img_height / ((bx - tx)*2));
					y0 = ((z - tz) * img_height / ((bz - tz)*2));

					for (int yy = y0-voxel_radius_pixels; yy <= y0+voxel_radius_pixels; yy++) {
						for (int xx = x0-voxel_radius_pixels; xx <= x0+voxel_radius_pixels; xx++) {
							if ((xx > -1) && (xx < img_width) &&
								(yy > -1) && (yy < img_height)) {
								int n = ((yy * img_width) + xx) * 3;
								img[n+2] = (unsigned char)voxels[i+3];
								img[n+1] = (unsigned char)voxels[i+4];
								img[n] = (unsigned char)voxels[i+5];
							}
						}
					}

					x = (img_height/2) + ((y - ty) * img_height / ((by - ty)*2));
					y = (img_height/2) + ((z - tz) * img_height / ((bz - tz)*2));
					break;
				}
			}
			for (int yy = y-voxel_radius_pixels; yy <= y+voxel_radius_pixels; yy++) {
				for (int xx = x-voxel_radius_pixels; xx <= x+voxel_radius_pixels; xx++) {
					if ((xx > -1) && (xx < img_width) &&
						(yy > -1) && (yy < img_height)) {
						int n = ((yy * img_width) + xx) * 3;
						img[n+2] = (unsigned char)voxels[i+3];
						img[n+1] = (unsigned char)voxels[i+4];
						img[n] = (unsigned char)voxels[i+5];
					}
				}
			}
		}
    }
}

void omni::show_feature_voxels(
	unsigned char* img,
	int img_width,
	int img_height,
	vector<short> &voxels,
	int view_type)
{
    int tx = 99999;
    int ty = 99999;
    int tz = 99999;
    int bx = 0;
    int by = 0;
    int bz = 0;

    // get the bounding box
    for (int i = (int)voxels.size()-8; i >= 0; i-=8) {
    	int x = voxels[i];
    	int y = voxels[i+1];
    	int z = voxels[i+2];
    	if (x < tx) tx = x;
    	if (x > bx) bx = x;
    	if (y < ty) ty = y;
    	if (y > by) by = y;
    	if (z < tz) tz = z;
    	if (z > bz) bz = z;
    }

    if (bz == tz) bz = tz+1;

    memset((void*)img, '\0',img_width*img_height*3);
    if ((bx - tx > 0) && (by - ty > 0) && (bz - tz > 0)) {
		for (int i = (int)voxels.size()-8; i >= 0; i -= 8) {
			int x = (int)voxels[i];
			int y = (int)voxels[i+1];
			int z = (int)voxels[i+2];
			int voxel_radius_pixels = (int)voxels[i+6] * img_width / (bx - tx);
			switch(view_type) {
				case 0: {
					x = (x - tx) * img_height / (bx - tx);
					y = (y - ty) * img_height / (by - ty);
					break;
				}
				case 1: {
					x = (x - tx) * img_height / (bx - tx);
					y = (z - tz) * img_height / (bz - tz);
					break;
				}
				case 2: {
					x = (y - ty) * img_height / (by - ty);
					y = (z - tz) * img_height / (bz - tz);
					break;
				}
				case 3: {
					int x0 = ((x - tx) * img_height / ((bx - tx)*2));
					int y0 = (img_height/4) + ((y - ty) * img_height / ((by - ty)*2));

					for (int yy = y0-voxel_radius_pixels; yy <= y0+voxel_radius_pixels; yy++) {
						for (int xx = x0-voxel_radius_pixels; xx <= x0+voxel_radius_pixels; xx++) {
							if ((xx > -1) && (xx < img_width) &&
								(yy > -1) && (yy < img_height)) {
								int n = ((yy * img_width) + xx) * 3;
								img[n+2] = (unsigned char)voxels[i+3];
								img[n+1] = (unsigned char)voxels[i+4];
								img[n] = (unsigned char)voxels[i+5];
							}
						}
					}

					x0 = (img_height/2) + ((x - tx) * img_height / ((bx - tx)*2));
					y0 = ((z - tz) * img_height / ((bz - tz)*2));

					for (int yy = y0-voxel_radius_pixels; yy <= y0+voxel_radius_pixels; yy++) {
						for (int xx = x0-voxel_radius_pixels; xx <= x0+voxel_radius_pixels; xx++) {
							if ((xx > -1) && (xx < img_width) &&
								(yy > -1) && (yy < img_height)) {
								int n = ((yy * img_width) + xx) * 3;
								img[n+2] = (unsigned char)voxels[i+3];
								img[n+1] = (unsigned char)voxels[i+4];
								img[n] = (unsigned char)voxels[i+5];
							}
						}
					}

					x = (img_height/2) + ((y - ty) * img_height / ((by - ty)*2));
					y = (img_height/2) + ((z - tz) * img_height / ((bz - tz)*2));
					break;
				}
			}
			for (int yy = y-voxel_radius_pixels; yy <= y+voxel_radius_pixels; yy++) {
				for (int xx = x-voxel_radius_pixels; xx <= x+voxel_radius_pixels; xx++) {
					if ((xx > -1) && (xx < img_width) &&
						(yy > -1) && (yy < img_height)) {
						int n = ((yy * img_width) + xx) * 3;
						img[n+2] = (unsigned char)voxels[i+3];
						img[n+1] = (unsigned char)voxels[i+4];
						img[n] = (unsigned char)voxels[i+5];
					}
				}
			}
		}
    }
}

void omni::show_height_field(
	unsigned char* img,
	int img_width,
	int img_height,
	int max_height_mm,
	short* height_field,
	unsigned char* height_field_colour,
	int height_field_width,
	int height_field_height,
	int view_type)
{
    memset((void*)img, '\0',img_width*img_height*3);

    int px=0,py=0;
    for (int y = 0; y < img_height; y++) {
    	int yy = y * height_field_height / img_height;
    	for (int x = 0; x < img_width; x++) {
    		int xx = x * height_field_width / img_width;
    		int n = yy*height_field_width + xx;
    		int height_mm = (int)height_field[n];
			switch(view_type) {
				case 0: {
					px = x;
					py = y;
					int n2 = (py*img_width + px)*3;
					img[n2] = height_field_colour[n*3+2];
					img[n2+1] = height_field_colour[n*3+1];
					img[n2+2] = height_field_colour[n*3];
					break;
				}
				case 1: {
					px = x;
					py = img_height - 1 - (height_mm * img_height / max_height_mm);
					int n2 = (py*img_width + px)*3;
					img[n2] = height_field_colour[n*3+2];
					img[n2+1] = height_field_colour[n*3+1];
					img[n2+2] = height_field_colour[n*3];
					break;
				}
				case 2: {
					px = x;
					py = img_height - 1 - (height_mm * img_height / max_height_mm);
					int n2 = (py*img_width + px)*3;
					img[n2] = height_field_colour[n*3+2];
					img[n2+1] = height_field_colour[n*3+1];
					img[n2+2] = height_field_colour[n*3];
					break;
				}
				case 3: {
					px = x/2;
					py = (img_height/4) + (y/2);
					int n2 = (py*img_width + px)*3;
					if ((n2 > -1) && (n2 < img_width*img_height*3)) {
					    img[n2] = height_field_colour[n*3];
					    img[n2+1] = height_field_colour[n*3+1];
					    img[n2+2] = height_field_colour[n*3+2];
					}

					px = (img_width/2) + (x/2);
					py = img_height - 1 - (height_mm*(img_height/2)/max_height_mm);
					n2 = (py*img_width + px)*3;
					if ((n2 > -1) && (n2 < img_width*img_height*3)) {
					    img[n2] = height_field_colour[n*3];
					    img[n2+1] = height_field_colour[n*3+1];
					    img[n2+2] = height_field_colour[n*3+2];
					}

					px = (img_width/2) + (x/2);
					py = (img_height/2) - 1 - (height_mm*(img_height/2)/max_height_mm);
					n2 = (py*img_width + px)*3;
					if ((n2 > -1) && (n2 < img_width*img_height*3)) {
					    img[n2] = height_field_colour[n*3];
					    img[n2+1] = height_field_colour[n*3+1];
					    img[n2+2] = height_field_colour[n*3+2];
					}

					break;
				}
			}

    	}

    }
}

void omni::reproject(
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
	int ray_map_height)
{
	int min_dist = (bx_mm - tx_mm) / img_width;
	if (min_dist < 2) min_dist = 2;
	min_dist = min_dist*min_dist;
	int ray_map_pixels = ray_map_width*ray_map_height;

	memset((void*)reprojected_img,'\0',ray_map_width*ray_map_height*3);

	int z_mm = (camera_to_backing_dist_mm + focal_length_mm) - (plane_height_mm + focal_length_mm - camera_height_mm);
	float z_fraction = z_mm / ((float)camera_to_backing_dist_mm + focal_length_mm);

	int w = bx_mm - tx_mm;
	int h = by_mm - ty_mm;
	vector<int> reprojected_pixels[img_width*img_height];
	for (int i = ray_map_pixels-1; i >= 0; i--) {
		if (ray_map[i*6 + 2] != 0) {
			int start_x_mm = ray_map[i*6];
			int start_y_mm = ray_map[i*6 + 1];
			int end_x_mm = ray_map[i*6 + 3];
			int end_y_mm = ray_map[i*6 + 4];

			int ray_x_mm = start_x_mm + (int)((end_x_mm - start_x_mm) * z_fraction);
			if ((ray_x_mm > tx_mm) && (ray_x_mm < bx_mm)) {
			    int ray_y_mm = start_y_mm + (int)((end_y_mm - start_y_mm) * z_fraction);
			    if ((ray_y_mm > ty_mm) && (ray_y_mm < by_mm)) {
			    	int x = ((ray_x_mm - tx_mm) * img_width / w);
			    	int y = ((ray_y_mm - ty_mm) * img_height / h);
			    	int n2 = y*img_width + x;
					if (!((ground_img[n2*3] == 0) &&
						  (ground_img[n2*3 + 1] == 0) &&
						  (ground_img[n2*3 + 2] == 0))) {
			    	    reprojected_pixels[n2].push_back(i);
					}
			    }
			}
		}
	}

	int n = 0;
	for (int y = 0; y < img_height; y++) {
		for (int x = 0; x < img_width; x++, n++) {
			if (!((ground_img[n*3] == 0) &&
				  (ground_img[n*3+1] == 0) &&
				  (ground_img[n*3+2] == 0))) {
				int n2 = y*img_width + (img_width-1-x);
				for (int p = (int)reprojected_pixels[n2].size()-1; p >= 0; p--) {
					int n3 = reprojected_pixels[n2][p]*3;
					reprojected_img[n3] = ground_img[n*3];
					reprojected_img[n3 + 1] = ground_img[n*3 + 1];
					reprojected_img[n3 + 2] = ground_img[n*3 + 2];
				}
			}
		}
	}

}


void omni::reconstruct_volume(
	unsigned char* ray_map_img,
	int start_plane_height_mm,
	int end_plane_height_mm,
	int no_of_planes,
	float focal_length_mm,
	int camera_to_backing_dist_mm,
	int camera_height_mm,
	int ray_map_width,
	int ray_map_height,
	int tx_mm,
	int ty_mm,
	int bx_mm,
	int by_mm,
	int camera_base_width_mm,
	int camera_base_height_mm,
	int* ray_map,
	unsigned char* mirror_map,
	float* mirror_lookup,
	unsigned char* projected_img,
	int projected_img_width,
	int projected_img_height,
	int* colour_difference,
	short* height_field,
	unsigned char* height_field_img,
	int patch_size_pixels,
	int min_patch_observations,
	int* plane_occupancy)
{
    int plane_step = abs(end_plane_height_mm - start_plane_height_mm) / no_of_planes;
    if (plane_step < 1) plane_step = 1;
	int p, mirror_index = -1;
	int min_r_mm = 0;
	int max_r_mm = 9999;
	int patch_size_pixels_sqr = patch_size_pixels*patch_size_pixels;
	int w = ray_map_width / patch_size_pixels;
	int h = ray_map_height / patch_size_pixels;
	int total_patches = w*h;
    int average_occupancy_probability = 0;
    int patches = 0;

	memset((void*)height_field,'\0',total_patches*sizeof(short));
	memset((void*)height_field_img,'\0',projected_img_width*projected_img_height*3);

	short cell_occupancy[no_of_planes*total_patches];
	memset((void*)plane_occupancy, '\0', no_of_planes*sizeof(int));
	memset((void*)cell_occupancy, '\0', no_of_planes*total_patches*sizeof(short));

	// pixel positions of the camera base
	int camera_base_width_pixels = (projected_img_width * camera_base_width_mm / (bx_mm - tx_mm));
	int camera_base_height_pixels = (projected_img_height * camera_base_height_mm / (by_mm - ty_mm));
	int camera_base_tx = (projected_img_width/2) - (camera_base_width_pixels/2);
	int camera_base_ty = (projected_img_height/2) - (camera_base_height_pixels/2);
	int camera_base_bx = camera_base_tx + camera_base_width_pixels;
	int camera_base_by = camera_base_ty + camera_base_height_pixels;

	for (int y = camera_base_ty; y < camera_base_by; y += patch_size_pixels, y++) {
		int hy = y / patch_size_pixels;
		for (int x = camera_base_tx; x < camera_base_bx; x += patch_size_pixels, p++) {
			int hx = x / patch_size_pixels;
			int n = hy*w + hx;
			height_field[n] = camera_height_mm;
		}
	}

    for (int plane = no_of_planes-1; plane >= 0; plane--) {

    	int plane_height_mm = start_plane_height_mm + (plane * (end_plane_height_mm - start_plane_height_mm) / no_of_planes);

    	project(
    		ray_map_img,
    		plane_height_mm,
    		focal_length_mm,
    		camera_to_backing_dist_mm,
    		camera_height_mm,
    		ray_map_width,
    		ray_map_height,
    		tx_mm, ty_mm,
    		bx_mm, by_mm,
    		ray_map,
    		mirror_map,
    		mirror_lookup,
    		mirror_index,
    		min_r_mm,
    		max_r_mm,
    		projected_img,
    		projected_img_width,
    		projected_img_height,
    		colour_difference);

    	int tot_colour_difference = 0;
    	int tot_observations = 1;
    	p = 0;
    	for (int y = 0; y < projected_img_height; y++) {
    		for (int x = 0; x < projected_img_width; x++, p++) {
    			if (!((x > camera_base_tx) && (x < camera_base_bx) &&
    				(y > camera_base_ty) && (y < camera_base_by))) {
    				if (colour_difference[p*2 + 1] > 0) {
        		        tot_colour_difference += colour_difference[p*2];
        		        tot_observations++;
    				}
    			}
    		}
    	}
    	tot_colour_difference /= tot_observations;
    	plane_occupancy[plane] = 1000000 / (1 + tot_colour_difference*tot_colour_difference);
    	//printf("%d tot_colour_difference %d\n", plane_height_mm, plane_occupancy[plane]);

    	/*
    	p = 0;
    	for (int y = 0; y < projected_img_height; y += patch_size_pixels, y++) {
    		for (int x = 0; x < projected_img_width; x += patch_size_pixels, p++) {

    			if (!((x > camera_base_tx) && (x < camera_base_bx) &&
    				(y > camera_base_ty) && (y < camera_base_by))) {

					// total colour difference for this patch
					int patch_observations = 0;
					int patch_colour_difference = 0;
					for (int yy = y; yy < y + patch_size_pixels; yy++) {
						for (int xx = x; xx < x + patch_size_pixels; xx++) {
							int n = yy*projected_img_width + xx;
							patch_observations += colour_difference[n*2 + 1];
							patch_colour_difference += colour_difference[n*2];
						}
					}

					if ((patch_observations > min_patch_observations) &&
						(patch_colour_difference > 0)) {

						// normalise for the number of observations
						patch_colour_difference /= patch_size_pixels_sqr;

						// turn the colour difference into a probability (crudely)
						int prob = 10000 / (1 + patch_colour_difference);
						cell_occupancy[plane*total_patches + p] = (short)prob;
						average_occupancy_probability += prob;
						patches++;
					}
    			}
    		}
    	}
    	*/
    }

    /*
    if (patches > 0) {
    	average_occupancy_probability /= patches;

    	// get the average occupancy for each plane
    	int max_plane_occupancy = 0;
    	for (int plane = 0; plane < no_of_planes; plane++) {
    		if (max_plane_occupancy < plane_occupancy[plane]) {
    		    max_plane_occupancy = plane_occupancy[plane];
    		}
    	}

    	// thresholds
    	int plane_occupancy_threshold = max_plane_occupancy * 98/100;
    	short cell_occupancy_threshold = (short)(average_occupancy_probability * 5/100);

    	int camera_base_plane = (start_plane_height_mm - camera_height_mm) * no_of_planes / (end_plane_height_mm - start_plane_height_mm);
    	if (camera_base_plane == no_of_planes) camera_base_plane = no_of_planes-1;
    	//printf("camera_base_plane %d/%d\n", camera_base_plane,no_of_planes);

    	// apply thresholds
    	for (int plane = no_of_planes - 1; plane >= 0; plane--) {
    		if ((plane_occupancy[plane] > plane_occupancy_threshold) ||
    			(plane == camera_base_plane)) {

            	short plane_height_mm = (short)(start_plane_height_mm + (plane * (end_plane_height_mm - start_plane_height_mm) / no_of_planes));

            	project(
            		ray_map_img,
            		plane_height_mm,
            		focal_length_mm,
            		camera_to_backing_dist_mm,
            		camera_height_mm,
            		ray_map_width,
            		ray_map_height,
            		tx_mm, ty_mm,
            		bx_mm, by_mm,
            		ray_map,
            		mirror_map,
            		mirror_lookup,
            		mirror_index,
            		min_r_mm,
            		max_r_mm,
            		projected_img,
            		projected_img_width,
            		projected_img_height,
            		colour_difference);

            	if (plane_occupancy[plane] > plane_occupancy_threshold) {

					for (int y = 0; y < projected_img_height; y++) {
						int y2 = y / patch_size_pixels;
						for (int x = 0; x < projected_img_width; x++) {
							int x2 = x / patch_size_pixels;
							int n = y2*w + x2;
							if (cell_occupancy[plane*total_patches + n] > cell_occupancy_threshold) {

								int n2 = y*projected_img_width + x;
								height_field[n] = plane_height_mm;
								height_field_img[n2*3] = projected_img[n2*3];
								height_field_img[n2*3 + 1] = projected_img[n2*3 + 1];
								height_field_img[n2*3 + 2] = projected_img[n2*3 + 2];

							}
						}
					}
            	}
            	else {

					for (int y = camera_base_ty; y < camera_base_by; y++) {
						int y2 = y / patch_size_pixels;
						for (int x = camera_base_tx; x < camera_base_bx; x++) {
							int x2 = x / patch_size_pixels;

							int n = y2*w + x2;
							int n2 = y*projected_img_width + x;
							height_field[n] = plane_height_mm;
							height_field_img[n2*3] = projected_img[n2*3];
							height_field_img[n2*3 + 1] = projected_img[n2*3 + 1];
							height_field_img[n2*3 + 2] = projected_img[n2*3 + 2];
						}
					}
            	}
    		}
    	}

    }
*/
}


void omni::project(
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
	unsigned char* mirror_map,
	float* mirror_lookup,
	int mirror_index,
	int min_r_mm,
	int max_r_mm,
	unsigned char* projected_img,
	int projected_img_width,
	int projected_img_height,
	int* colour_difference)
{
	int pow2[] = {
		2,4,8,16,32,64,128,256
	};

	int w = bx_mm - tx_mm;
	int h = by_mm - ty_mm;

	memset((void*)projected_img,'\0',projected_img_width*projected_img_height*3);
	memset((void*)colour_difference,'\0',projected_img_width*projected_img_height*2*sizeof(int));

	int z_mm = (int)(camera_to_backing_dist_mm + focal_length_mm - (plane_height_mm + focal_length_mm - camera_height_mm));
	float z_fraction = z_mm / ((float)camera_to_backing_dist_mm + focal_length_mm);

	int n = 0;
	for (int y = 0; y < ray_map_height; y++) {
		for (int x = 0; x < ray_map_width; x++, n++) {
			int mirror = mirror_map[n] - 1;
			if ((mirror > -1) &&
				(!((ray_map_img[n*3] == 0) && (ray_map_img[n*3+1] == 0) && (ray_map_img[n*3+2] == 0))) &&
				((mirror == mirror_index) || (mirror_index == -1))) {
				if ((mirror_lookup[n*2] >= min_r_mm) &&
					(mirror_lookup[n*2] <= max_r_mm)) {
					if ((ray_map[n*6 + 2] != 0) &&
						(ray_map[n*6 + 5] < ray_map[n*6 + 2])) {

						int start_x_mm = ray_map[n*6];
						int start_y_mm = ray_map[n*6 + 1];
						int end_x_mm = ray_map[n*6 + 3];
						int end_y_mm = ray_map[n*6 + 4];

						int ray_x_mm = start_x_mm + (int)((end_x_mm - start_x_mm) * z_fraction);
						int ray_y_mm = start_y_mm + (int)((end_y_mm - start_y_mm) * z_fraction);

						int plane_x = (ray_x_mm - tx_mm) * projected_img_width / w;
						int ray_radius =  ((abs(ray_x_mm) + abs(ray_y_mm))*4/projected_img_width);
						if (ray_radius > 15) ray_radius = 15;
						if ((plane_x > ray_radius) && (plane_x < projected_img_width-ray_radius-1)) {
							int plane_y = (ray_y_mm - ty_mm) * projected_img_height / h;
							if ((plane_y > ray_radius) && (plane_y < projected_img_height-ray_radius-1)) {

								for (int py = plane_y-ray_radius; py <= plane_y+ray_radius; py++) {
									for (int px = plane_x-ray_radius; px <= plane_x+ray_radius; px++) {

										int n2 = (py * projected_img_width) + (ray_map_width-1-px);
										int n3 = n2*3;
										if ((projected_img[n3] == 0) &&
											(projected_img[n3+1] == 0) &&
											(projected_img[n3+2] == 0)) {

											// update plane colour
											projected_img[n3] = ray_map_img[n*3];
											projected_img[n3 + 1] = ray_map_img[n*3 + 1];
											projected_img[n3 + 2] = ray_map_img[n*3 + 2];
										}
										else {

											if (colour_difference != NULL) {
												// colour difference
												int diff =
													abs(ray_map_img[n*3] - projected_img[n3]) +
													abs(ray_map_img[n*3 + 1] - projected_img[n3 + 1]) +
													abs(ray_map_img[n*3 + 2] - projected_img[n3 + 2]);
												colour_difference[n2*2] += diff;
												colour_difference[n2*2 + 1]++;

												// total colour difference
												//colour_difference[0] += diff;
												//colour_difference[1]++;
											}

											// update plane colour
											projected_img[n3] += (ray_map_img[n*3] - projected_img[n3])/2;
											projected_img[n3 + 1] += (ray_map_img[n*3 + 1] - projected_img[n3 + 1])/2;
											projected_img[n3 + 2] += (ray_map_img[n*3 + 2] - projected_img[n3 + 2])/2;

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

	// normalise colour difference for number of observations
	if (colour_difference != NULL) {
		for (int i = projected_img_width*projected_img_height-1; i >= 0; i--) {
			if (colour_difference[i*2 + 1] > 0) {
				colour_difference[i*2] /= colour_difference[i*2 + 1];
			}
		}
	}

}

void omni::voxels_from_features(
	vector<int> &features,
	unsigned char* ray_map_img,
	int ray_map_width,
	int ray_map_height,
	int start_plane_height_mm,
	int end_plane_height_mm,
	int no_of_planes,
	float focal_length_mm,
	int camera_to_backing_dist_mm,
	int camera_height_mm,
	int* ray_map,
	int no_of_mirrors,
	unsigned char* mirror_map,
	int outer_radius_percent,
	int mirror_diameter_mm,
	vector<short> &voxels)
{
	voxels.clear();
	vector<vector<int> > features_on_plane;
	vector<int> vox;
	vector<int> projected_points;
	vector<int> matched_features;
	int ray_radius_mm = 0;
	int mult = 10;
	int minimum_matching_score_percent = 0;
	float pixel_diameter_mm = mirror_diameter_mm / (float)(outer_radius_percent * ray_map_width / 200);
	pixel_diameter_mm *= mult;
	bool remove_matched_pixels = false;

	for (int mirror = 0; mirror < no_of_mirrors; mirror++) {
		vector<int> f;
		features_on_plane.push_back(f);
	}

	for (int p = 0; p < no_of_planes; p++) {
		int plane_height_mm = start_plane_height_mm + ((end_plane_height_mm - start_plane_height_mm) * p / no_of_planes);

		match_features_on_plane(
			features,
			features_on_plane,
			no_of_mirrors,
			ray_map_img,
			plane_height_mm,
			focal_length_mm,
			camera_to_backing_dist_mm,
			camera_height_mm,
			ray_map_width,
			ray_map_height,
			ray_map,
			mirror_map,
			-1,
			pixel_diameter_mm,
			vox,
			projected_points,
			matched_features,
			minimum_matching_score_percent,
			ray_radius_mm,
			remove_matched_pixels);

		ray_radius_mm /= mult;

		for (int i = (int)vox.size()-4; i >= 0; i -= 4) {
			short n = (short)vox[i]*3;
			short x_mm = (short)vox[i+1];
			short y_mm = (short)vox[i+2];
			short score = (short)vox[i+3];

			voxels.push_back(x_mm);
			voxels.push_back(y_mm);
			voxels.push_back((short)plane_height_mm);
			voxels.push_back((short)ray_map_img[n+2]);
			voxels.push_back((short)ray_map_img[n+1]);
			voxels.push_back((short)ray_map_img[n]);
			voxels.push_back((short)ray_radius_mm);
			voxels.push_back(score);

		}
	}
}

void omni::match_features_on_plane(
	vector<int> &features,
	vector<vector<int> > &features_on_plane,
	int no_of_mirrors,
	unsigned char* ray_map_img,
	int plane_height_mm,
	float focal_length_mm,
	int camera_to_backing_dist_mm,
	int camera_height_mm,
	int ray_map_width,
	int ray_map_height,
	int* ray_map,
	unsigned char* mirror_map,
	int mirror_index,
	float pixel_diameter_mm,
	vector<int> &voxels,
	vector<int> &projected_points,
	vector<int> &matched_features,
	int minimum_matching_score_percent,
	int &ray_radius_mm,
	bool remove_matched_pixels)
{
	if ((int)features_on_plane.size() < no_of_mirrors) {
		printf("features_on_plane is too small\n");
	}
	else {

		const int histogram_radius = 5;

		int pow2[32];
		for (int i = 0; i < 32; i++) {
			pow2[i] = (int)pow(2, i);
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

		int minimum_matches = no_of_mirrors-2;

		for (int i = 0; i < no_of_mirrors; i++) {
			features_on_plane[i].clear();
		}
		matched_features.clear();
		voxels.clear();
		projected_points.clear();

		int z_mm = (int)(camera_to_backing_dist_mm + focal_length_mm - (plane_height_mm + focal_length_mm - camera_height_mm));
		float z_fraction = z_mm / ((float)camera_to_backing_dist_mm + focal_length_mm);

		// approximate radius of the ray on the projection plane
		int ray_length_mm = ((camera_to_backing_dist_mm + focal_length_mm)*2) + abs(camera_height_mm - plane_height_mm);
		ray_radius_mm = ray_length_mm * pixel_diameter_mm / (camera_to_backing_dist_mm + focal_length_mm);

		for (int f = (int)features.size()-2; f >= 0; f -= 2) {
			int fx = features[f];
			int fy = features[f+1];

			if ((fx > histogram_radius) && (fx < ray_map_width-histogram_radius) &&
				(fy > histogram_radius) && (fy < ray_map_height-histogram_radius)) {

				int n = fy*ray_map_width + fx;

				int mirror = mirror_map[n] - 1;
				if ((mirror > -1) &&
					((mirror == mirror_index) || (mirror_index == -1))) {
					if ((!((ray_map_img[n*3] == 0) && (ray_map_img[n*3+1] == 0) && (ray_map_img[n*3+2] == 0))) &&
						(ray_map[n*6 + 2] != 0) &&
						(ray_map[n*6 + 5] < ray_map[n*6 + 2])) {

						int start_x_mm = -ray_map[n*6];
						int start_y_mm = ray_map[n*6 + 1];
						int end_x_mm = -ray_map[n*6 + 3];
						int end_y_mm = ray_map[n*6 + 4];

						int ray_x_mm = start_x_mm + (int)((end_x_mm - start_x_mm) * z_fraction);
						int ray_y_mm = start_y_mm + (int)((end_y_mm - start_y_mm) * z_fraction);

						projected_points.push_back(n);
						projected_points.push_back(ray_x_mm);
						projected_points.push_back(ray_y_mm);
						projected_points.push_back(0);

						features_on_plane[mirror].push_back(f);
						features_on_plane[mirror].push_back(ray_x_mm);
						features_on_plane[mirror].push_back(ray_y_mm);
						features_on_plane[mirror].push_back(0);
						features_on_plane[mirror].push_back(0);
						features_on_plane[mirror].push_back(0);

					}

				}
			}

		}

		for (int f0 = (int)features_on_plane[no_of_mirrors-1].size()-6; f0 >= 0; f0 -= 6) {
			int fx0 = features_on_plane[no_of_mirrors-1][f0 + 1];
			int fy0 = features_on_plane[no_of_mirrors-1][f0 + 2];
			vector<int> matches;

			// do any rays from other mirror match this one?
			for (int mirror = 0; mirror < no_of_mirrors-1; mirror++) {
				int min_separation = ray_radius_mm*ray_radius_mm;
				int best_f1 = -1;

				for (int f1 = (int)features_on_plane[mirror].size()-6; f1 >= 0; f1 -= 6) {

					// is this feature within range?
					int fx1 = features_on_plane[mirror][f1 + 1];
					int dx = fx1 - fx0;
					if ((dx > -ray_radius_mm) && (dx < ray_radius_mm)) {
						int fy1 = features_on_plane[mirror][f1 + 2];
						int dy = fy1 - fy0;
						if ((dy > -ray_radius_mm) && (dy < ray_radius_mm)) {
							int separation = dx*dx + dy*dy;
							if (separation < min_separation) {
								// closest feature
								min_separation = separation;
								best_f1 = f1;
								//printf("min_separation %d\n", min_separation);
							}
						}
					}
				}

				// add the best match to the list
				if (best_f1 > -1) {
					matches.push_back(mirror);
					matches.push_back(best_f1);
				}
			}

			if ((int)matches.size() >= minimum_matches*2) {
				// are these matches photo consistent?

				bool matched = true;
				int matching_score = 0;
				int matching_score_hits = 0;

				unsigned int histogram_r0 = 0;
				unsigned int histogram_g0 = 0;
				unsigned int histogram_b0 = 0;
				for (int y = fy0-histogram_radius; y <= fy0+histogram_radius; y++) {
					int n2 = (y*ray_map_width + fx0 - histogram_radius)*3;
					for (int x = fx0-histogram_radius; x <= fx0+histogram_radius; x++, n2 += 3) {
						histogram_r0 |= pow2[ray_map_img[n2 + 2] / 8];
						histogram_g0 |= pow2[ray_map_img[n2 + 1] / 8];
						histogram_b0 |= pow2[ray_map_img[n2] / 8];
					}
				}

				int no_of_bits_set =
					BitsSetTable256[histogram_r0 & 0xff] +
					BitsSetTable256[(histogram_r0 >> 8) & 0xff] +
					BitsSetTable256[(histogram_r0 >> 16) & 0xff] +
					BitsSetTable256[histogram_r0 >> 24] +

					BitsSetTable256[histogram_g0 & 0xff] +
					BitsSetTable256[(histogram_g0 >> 8) & 0xff] +
					BitsSetTable256[(histogram_g0 >> 16) & 0xff] +
					BitsSetTable256[histogram_g0 >> 24] +

					BitsSetTable256[histogram_b0 & 0xff] +
					BitsSetTable256[(histogram_b0 >> 8) & 0xff] +
					BitsSetTable256[(histogram_b0 >> 16) & 0xff] +
					BitsSetTable256[histogram_b0 >> 24];

				if (no_of_bits_set > 0) {

					for (int i = (int)matches.size()-2; i >= 0; i -= 2) {
						int mirror = matches[i];
						int f1 = matches[i + 1];

						unsigned int histogram_r1 = features_on_plane[mirror][f1 + 3];
						unsigned int histogram_g1 = features_on_plane[mirror][f1 + 4];
						unsigned int histogram_b1 = features_on_plane[mirror][f1 + 5];
						if ((histogram_r1 == 0) &&
							(histogram_g1 == 0) &&
							(histogram_b1 == 0)) {
							int fx1 = features_on_plane[mirror][f1 + 1];
							int fy1 = features_on_plane[mirror][f1 + 2];

							for (int y = fy1-histogram_radius; y <= fy1+histogram_radius; y++) {
								int n2 = (y*ray_map_width + fx1 - histogram_radius)*3;
								for (int x = fx1-histogram_radius; x <= fx1+histogram_radius; x++, n2 += 3) {
									histogram_r1 |= pow2[ray_map_img[n2 + 2] / 8];
									histogram_g1 |= pow2[ray_map_img[n2 + 1] / 8];
									histogram_b1 |= pow2[ray_map_img[n2] / 8];
								}
							}
							features_on_plane[mirror][f1 + 3] = histogram_r1;
							features_on_plane[mirror][f1 + 4] = histogram_g1;
							features_on_plane[mirror][f1 + 5] = histogram_b1;
						}

						unsigned int dr = histogram_r0 & histogram_r1;
						unsigned int dg = histogram_g0 & histogram_g1;
						unsigned int db = histogram_b0 & histogram_b1;

						if ((dr > 0) || (dg > 0) || (db > 0)) {
							int correlation =
								BitsSetTable256[dr & 0xff] +
								BitsSetTable256[(dr >> 8) & 0xff] +
								BitsSetTable256[(dr >> 16) & 0xff] +
								BitsSetTable256[dr >> 24] +

								BitsSetTable256[dg & 0xff] +
								BitsSetTable256[(dg >> 8) & 0xff] +
								BitsSetTable256[(dg >> 16) & 0xff] +
								BitsSetTable256[dg >> 24] +

								BitsSetTable256[db & 0xff] +
								BitsSetTable256[(db >> 8) & 0xff] +
								BitsSetTable256[(db >> 16) & 0xff] +
								BitsSetTable256[db >> 24];

							int correlation_percent = correlation * 100 / no_of_bits_set;

							if ((correlation_percent < 20) || (correlation_percent > 180)) {
								// the histograms do not match
								matched = false;
								break;
							}
							else {
								matching_score += 100 - abs(correlation_percent - 100);
								matching_score_hits++;
							}
						}
					}

					if ((matched) &&
						(matching_score > minimum_matching_score_percent*matching_score_hits)) {

						int f = features_on_plane[no_of_mirrors-1][f0];
						int px = features[f];
						int py = features[f + 1];

						voxels.push_back(py*ray_map_width + px);
						voxels.push_back(fx0);
						voxels.push_back(fy0);
						voxels.push_back(matching_score);

						matched_features.push_back(f);
						for (int i = (int)matches.size()-2; i >= 0; i -= 2) {
							int mirror = matches[i];
							int f1 = matches[i + 1];
							f = features_on_plane[mirror][f1];
							matched_features.push_back(f);
						}

					}
				}
			}
		}

		if (remove_matched_pixels) {
			sort (matched_features.begin(),matched_features.end());

			int prev_f = 99999;
			for (int i = (int)matched_features.size()-1; i >= 0; i--) {
				int f = matched_features[i];
				if (prev_f > f) {
					features.erase(features.begin()+f);
					features.erase(features.begin()+f);
					prev_f = f;
				}
			}
		}
	}
}

/*!
 * \brief project the given image features onto the given height plane
 * \param features list of features in image coordinates
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
 * \param projected_features returned projected features (x,y)
 */
void omni::project_features(
	vector<int> &features,
	int mirror_index,
	int plane_height_mm,
	float focal_length_mm,
	int camera_to_backing_dist_mm,
	int camera_height_mm,
	int ray_map_width,
	int ray_map_height,
	int* ray_map,
	unsigned char* mirror_map,
	int max_range_mm,
    vector<int> &projected_features)
{
	projected_features.clear();

	int z_mm = (int)(camera_to_backing_dist_mm + focal_length_mm - (plane_height_mm + focal_length_mm - camera_height_mm));
	float z_fraction = z_mm / ((float)camera_to_backing_dist_mm + focal_length_mm);

	for (int f = 0; f < (int)features.size(); f += 2) {
		int fx = features[f];
		int fy = features[f+1];

		int n = fy*ray_map_width + fx;

		int mirror = mirror_map[n] - 1;
		if ((mirror > -1) &&
			((mirror == mirror_index) || (mirror_index == -1))) {
			int n2 = n*6;
			if ((ray_map[n2 + 2] != 0) &&
				(ray_map[n2 + 5] < ray_map[n2 + 2])) {

				int start_x_mm = ray_map[n2];
				int start_y_mm = ray_map[n2 + 1];
				int end_x_mm = ray_map[n2 + 3];
				int end_y_mm = ray_map[n2 + 4];

				int ray_x_mm = start_x_mm + (int)((end_x_mm - start_x_mm) * z_fraction);
				int ray_y_mm = start_y_mm + (int)((end_y_mm - start_y_mm) * z_fraction);

				if ((ray_x_mm > -max_range_mm) && (ray_x_mm < max_range_mm) &&
					(ray_y_mm > -max_range_mm) && (ray_y_mm < max_range_mm)) {
				    projected_features.push_back(ray_x_mm);
				    projected_features.push_back(ray_y_mm);
				    projected_features.push_back(n);
				}

			}
		}
	}
}

/*!
 * \brief reproject features from a plane (typically the ground) into image coordinates
 * \param plane_features features on the plane
 * \param mirror_index index number of the mirror
 * \param plane_height_mm height of the plane above the ground
 * \param plane_tollerance_mm tolerance of the intercept point above and below the plane
 * \param focal_length_mm focal length of the camera
 * \param camera_to_backing_dist_mm distance between the camera and the mirror backing plane
 * \param camera_height_mm height of the camera above the ground
 * \param ray_map lookup table containing ray vectors
 * \param ray_map_width width of the image
 * \param ray_map_height height of the image
 * \param mirror_map lookup table containing mirror indexes
 * \param ground_plane_tollerance_mm tollerance for matching rays on the plane
 * \param max_range_mm maximum range in millimetres
 * \param reprojected_features returned reprojected features in image coordinates
 */
void omni::reproject_features(
	vector<int> &plane_features,
	int mirror_index,
	int plane_height_mm,
	int plane_tollerance_mm,
	float focal_length_mm,
	int camera_to_backing_dist_mm,
	int camera_height_mm,
	int* ray_map,
	int ray_map_width,
	int ray_map_height,
    unsigned char* mirror_map,
    int ground_plane_tollerance_mm,
    int max_range_mm,
    vector<int> &reprojected_features)
{
	reprojected_features.clear();

	int ground_plane_tollerance_mm_sqr = ground_plane_tollerance_mm*ground_plane_tollerance_mm/4;

	int z_mm = (camera_to_backing_dist_mm + focal_length_mm) - (plane_height_mm + focal_length_mm - camera_height_mm);
	float z_fraction = z_mm / ((float)camera_to_backing_dist_mm + focal_length_mm);

	int bounding_box_tx = 99999;
	int bounding_box_ty = 99999;
	int bounding_box_bx = -99999;
	int bounding_box_by = -99999;
	for (int f = (int)plane_features.size()-3; f >= 0; f -= 3) {
		int x = plane_features[f];
		int y = plane_features[f + 1];
		if (x < bounding_box_tx) bounding_box_tx = x;
		if (y < bounding_box_ty) bounding_box_ty = y;
		if (x > bounding_box_bx) bounding_box_bx = x;
		if (y > bounding_box_by) bounding_box_by = y;
	}
	bounding_box_tx -= ground_plane_tollerance_mm;
	bounding_box_ty -= ground_plane_tollerance_mm;
	bounding_box_bx += ground_plane_tollerance_mm;
	bounding_box_by += ground_plane_tollerance_mm;

	if (bounding_box_tx < -max_range_mm) bounding_box_tx = -max_range_mm;
	if (bounding_box_ty < -max_range_mm) bounding_box_ty = -max_range_mm;
	if (bounding_box_bx > max_range_mm) bounding_box_bx = max_range_mm;
	if (bounding_box_by > max_range_mm) bounding_box_by = max_range_mm;

	int bounding_box_width = bounding_box_bx - bounding_box_tx;
	int bounding_box_height = bounding_box_by - bounding_box_ty;

	if ((bounding_box_width > 0) && (bounding_box_height > 0)) {
		int w = bounding_box_width / ground_plane_tollerance_mm;
		int h = bounding_box_height / ground_plane_tollerance_mm;

		//printf("wh %d %d\n",w,h);
		unsigned short ground_features[(w+1)*(h+1)*10];
		memset((void*)ground_features,'\0',(w+1)*(h+1)*10*sizeof(unsigned short));
		for (int f = (int)plane_features.size()-3; f >= 0; f -= 3) {
			int x = (plane_features[f] - bounding_box_tx) * w / bounding_box_width;
			int y = (plane_features[f+1] - bounding_box_ty) * h / bounding_box_height;
			int n = (y*w + x)*10;
			if (ground_features[n] < 9) {
				ground_features[n+ground_features[n]+1] = (unsigned short)(f+1);
				ground_features[n]++;
			}
		}

		int n = 0;
		int i = 0;
		for (int y = 0; y < ray_map_height; y++) {
			for (int x = 0; x < ray_map_width; x++, i += 6, n++) {

				if ((ray_map[i + 2] != 0) &&
					(ray_map[i + 5] < ray_map[i + 2]) &&
					((mirror_index == -1) || (mirror_map[n]-1 == mirror_index))) {

					int start_x_mm = ray_map[i];
					int start_y_mm = ray_map[i + 1];
					int end_x_mm = ray_map[i + 3];
					int end_y_mm = ray_map[i + 4];

					int ray_x_mm = start_x_mm + (int)((end_x_mm - start_x_mm) * z_fraction);
					if ((ray_x_mm > bounding_box_tx) && (ray_x_mm < bounding_box_bx)) {
						int ray_y_mm = start_y_mm + (int)((end_y_mm - start_y_mm) * z_fraction);
						if ((ray_y_mm > bounding_box_ty) && (ray_y_mm < bounding_box_by)) {

							int xx = ((ray_x_mm - bounding_box_tx) * w) / bounding_box_width;
							int yy = ((ray_y_mm - bounding_box_ty) * h) / bounding_box_height;
							int n2 = yy*w + xx;
							if (ground_features[n2*10] > 0) {

								int start_z_mm = ray_map[i + 2] + camera_height_mm;
								int end_z_mm = ray_map[i + 5] + camera_height_mm;

								int max_gf = ground_features[n2*10];
								for (int gf = max_gf-1; gf >= 0; gf--) {

									// find vertical intercept point between the two rays
									int f = (int)ground_features[n2*10+gf+1]-1;
									int n3 = plane_features[f+2]*6;
									int plane_start_x_mm = ray_map[n3];
									int plane_start_y_mm = ray_map[n3+1];
									int plane_start_z_mm = ray_map[n3+2] + camera_height_mm;
									int plane_end_x_mm = ray_map[n3+3];
									int plane_end_y_mm = ray_map[n3+4];
									int plane_end_z_mm = ray_map[n3+5] + camera_height_mm;

									float ix=0, iy=0, iz=0;
									//float dx=0, dy=0, dz=0;

									//min_distance_between_rays(
									rays_intercept(
										start_x_mm,start_y_mm,start_z_mm,
										end_x_mm,end_y_mm,end_z_mm,
										plane_start_x_mm,plane_start_y_mm,plane_start_z_mm,
										plane_end_x_mm,plane_end_y_mm,plane_end_z_mm,
										//dx,dy,dz,
										ix,iy,iz);

									int deviation_from_plane_mm = (int)(iz - plane_height_mm);

									//if ((iz > 0) && (plane_height_mm > 0)) {
									//	printf("iz %f/%d %d\n", iz,plane_height_mm, plane_tollerance_mm);
									//}

									if ((deviation_from_plane_mm >= -plane_tollerance_mm) &&
										(deviation_from_plane_mm <= plane_tollerance_mm)) {
										//int dist = (int)sqrt(dx*dx + dy*dy + dz*dz);
										//printf("dist = %d/%d\n",dist,ground_plane_tollerance_mm);
										//int dx2 = (int)ix - ray_x_mm;
										//if ((dx2 >= -ground_plane_tollerance_mm) && (dx2 <= ground_plane_tollerance_mm)) {
											//int dy2 = (int)iy - ray_y_mm;
											//if ((dy2 >= -ground_plane_tollerance_mm) && (dy2 <= ground_plane_tollerance_mm)) {
												//printf("%d %d / %d\n",dx2,dy2,ground_plane_tollerance_mm);
											//int n4= yy*w + xx;
											//if (ground_features[n4] > 0) {
												reprojected_features.push_back(x);
												reprojected_features.push_back(y);
												gf = -1;
												//printf("iz %f/%d\n", iz,plane_height_mm);
											//}
											//}
										//}
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


void omni::show_voxels(
	unsigned char* ray_map_img,
	int tx_mm,
	int ty_mm,
	int bx_mm,
	int by_mm,
	vector<int> &voxels,
	int img_width,
	int img_height,
	unsigned char* result)
{
	memset((void*)result,'\0',img_width*img_height*3);

	for (int i = (int)voxels.size()-4; i >= 0; i -= 4) {
		int n = voxels[i]*3;
		int x_mm = voxels[i + 1];
		int y_mm = voxels[i + 2];
		int x = (x_mm - tx_mm) * img_width / (bx_mm - tx_mm);
		int y = (y_mm - ty_mm) * img_height / (by_mm - ty_mm);
		int n2 = (y*img_width + x)*3;
		result[n2] = ray_map_img[n];
		result[n2+1] = ray_map_img[n+1];
		result[n2+2] = ray_map_img[n+2];
	}
}


void omni::show_rays(
	unsigned char* mirror_map,
	int* ray_map,
	unsigned char* img,
	int img_width,
	int img_height,
	int radius_mm,
	int point_radius_pixels)
{
    memset((void*)img,'\0',img_width*img_height*3);

    int point_radius_sqr = point_radius_pixels*point_radius_pixels;
    int n = 0;
    for (int y = 0; y < img_height; y++) {
    	for (int x = 0; x < img_width; x++, n++) {
            if (mirror_map[n] > 0) {
            	int mirror = mirror_map[n] - 1;
				int ray_start_z_mm = ray_map[n*6 + 2];
				int ray_end_x_mm = ray_map[n*6 + 3];
				int ray_end_y_mm = ray_map[n*6 + 4];
				int ray_end_z_mm = ray_map[n*6 + 5];
				if (ray_end_z_mm < ray_start_z_mm) {
					if ((ray_end_x_mm > -radius_mm) &&
						(ray_end_x_mm < radius_mm) &&
						(ray_end_y_mm > -radius_mm) &&
						(ray_end_y_mm < radius_mm)) {
						int xx = (ray_end_x_mm + radius_mm) * img_height / (radius_mm*2);
						int yy = (ray_end_y_mm + radius_mm) * img_height / (radius_mm*2);
						if ((xx > 0) && (xx < img_width-1) &&
						    (yy > 0) && (yy < img_height-1)) {

						    for (int yy2 = yy-point_radius_pixels; yy2 <= yy+point_radius_pixels; yy2++) {
						    	int dy = yy2 - yy;
							    for (int xx2 = xx-point_radius_pixels; xx2 <= xx+point_radius_pixels; xx2++) {
							    	int dx = xx2 - xx;

							    	if (dx*dx + dy*dy < point_radius_sqr) {

										int n2 = ((yy2*img_width) + xx2)*3;
										switch(mirror) {
											case 0: {
												img[n2] = 255;
												break;
											}
											case 1: {
												img[n2+1] = 255;
												break;
											}
											case 2: {
												img[n2+2] = 255;
												break;
											}
											case 3: {
												img[n2+1] = 255;
												img[n2+2] = 255;
												break;
											}
											case 4: {
												img[n2] = 255;
												img[n2+2] = 255;
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
    }

    n = 0;
    for (int y = 0; y < img_height; y++) {
    	for (int x = 0; x < img_width; x++, n++) {
            if (mirror_map[n] > 0) {
            	int mirror = mirror_map[n] - 1;
				int ray_start_x_mm = ray_map[n*6];
				int ray_start_y_mm = ray_map[n*6 + 1];
				int ray_start_z_mm = ray_map[n*6 + 2];
				int ray_end_x_mm = ray_map[n*6 + 3];
				int ray_end_y_mm = ray_map[n*6 + 4];
				int ray_end_z_mm = ray_map[n*6 + 5];
				if (ray_end_z_mm < ray_start_z_mm) {
					if ((ray_end_x_mm > -radius_mm) &&
						(ray_end_x_mm < radius_mm) &&
						(ray_end_y_mm > -radius_mm) &&
						(ray_end_y_mm < radius_mm)) {
						int xx = (ray_start_x_mm + radius_mm) * img_height / (radius_mm*2);
						int yy = (ray_start_y_mm + radius_mm) * img_height / (radius_mm*2);
						if ((xx > 0) && (xx < img_width-1) &&
						    (yy > 0) && (yy < img_height-1)) {

						    for (int yy2 = yy-point_radius_pixels; yy2 <= yy+point_radius_pixels; yy2++) {
						    	int dy = yy2 - yy;
							    for (int xx2 = xx-point_radius_pixels; xx2 <= xx+point_radius_pixels; xx2++) {
							    	int dx = xx2 - xx;

							    	if (dx*dx + dy*dy < point_radius_sqr) {

										int n2 = ((yy2*img_width) + xx2)*3;
										switch(mirror) {
											case 0: {
												img[n2] = 255;
												img[n2+1] = 0;
												img[n2+2] = 0;
												break;
											}
											case 1: {
												img[n2] = 0;
												img[n2+1] = 255;
												img[n2+2] = 0;
												break;
											}
											case 2: {
												img[n2] = 0;
												img[n2+1] = 0;
												img[n2+2] = 255;
												break;
											}
											case 3: {
												img[n2] = 0;
												img[n2+1] = 255;
												img[n2+2] = 255;
												break;
											}
											case 4: {
												img[n2+1] = 0;
												img[n2] = 255;
												img[n2+2] = 255;
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
    }

}

void omni::create_ray_map(
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
    unsigned char* mirror_map,
    float* mirror_lookup)
{
	int mirror_radius_pixels = img_width * outer_radius_percent / 200;

	memset((void*)ray_map,'\0',img_width*img_height*6*sizeof(int));
	memset((void*)mirror_lookup,'\0',img_width*img_height*2*sizeof(float));

	for (int mirror = 0; mirror < no_of_mirrors; mirror++) {
		// mirror distance from the centre of the backing plane (xy)
		float mirror_dist_from_backing_centre =
			(float)sqrt(mirror_position[mirror*2]*mirror_position[mirror*2] +
					mirror_position[mirror*2+1]*mirror_position[mirror*2+1]);

		// angle of the mirror on the xz plane
		float tilt_radians = (float)atan2(mirror_dist_from_backing_centre, dist_to_mirror_backing_mm + focal_length);

		// rotation of the mirror on the backing plane (xy)
		float rotate_radians = (float)atan2(mirror_position[mirror*2], mirror_position[mirror*2+1]);

		// distance to the centre of the mirror
		float dist_to_mirror_centre_mm = (float)sqrt(mirror_dist_from_backing_centre*mirror_dist_from_backing_centre + (dist_to_mirror_backing_mm+focal_length)*(dist_to_mirror_backing_mm+focal_length));

		create_ray_map_mirror(
			mirror,
			mirror_position_pixels[mirror*2]*img_width/100,
			mirror_position_pixels[mirror*2 + 1]*img_height/100,
			mirror_position[mirror*2],
			mirror_position[mirror*2 + 1],
			mirror_radius_pixels,
			ray_map,
			mirror_map,
			mirror_lookup,
			img_width, img_height,
			tilt_radians,
			rotate_radians,
			mirror_diameter_mm,
			dist_to_mirror_centre_mm,
			camera_height_mm);
	}
}

void omni::create_ray_map_mirror_inner(
	int mirror,
	int centre_x_pixels,
	int centre_y_pixels,
	int centre_x_mm,
	int centre_y_mm,
	int radius_pixels,
	int* ray_map,
	unsigned char* mirror_map,
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
	bool negative)
{
	float z0 = (float)sqrt(radius_pixels*radius_pixels - r*r);
	if (negative) z0 = -z0;
	float sphere_ang = (float)asin(r / (float)radius_pixels);

	//float sphere_x = (float)fabs(mirror_diameter_mm*0.5*sin(sphere_ang));
	//float sphere_z = dist_to_mirror_centre_mm - (float)fabs(mirror_diameter_mm*0.5f*cos(sphere_ang));

	float sphere_x = r * mirror_diameter_mm * 0.5f / radius_pixels;
	float sphere_y = 0;
	float sphere_z = dist_to_mirror_centre_mm - (z0 * mirror_diameter_mm * 0.5f / radius_pixels);
	float viewing_angle = (float)fabs(atan2(sphere_x, sphere_z));
	float ang2 = viewing_angle + (sphere_ang * 2);
	float ray_end_y = 0;
	float ray_end_z = 0;
	float ray_end_x = sphere_x + (sphere_z * (float)tan(ang2));
	bool ground = true;
	if (ray_end_x < 0) {
		ray_end_x = sphere_x - (sphere_z * (float)tan(ang2));
		ray_end_z = sphere_z * 2;
		ground = false;
	}

	for (float ang = 0; ang < 360; ang += 0.2f) {

		float ang_radians = (ang / 180 * 3.1415927f);

		// rotate
		float sphere_x2 = (float)((cos(ang_radians+3.1415927f) * sphere_x) - (sin(ang_radians+3.1415927f) * sphere_y));
		float sphere_y2 = (float)((sin(ang_radians) * sphere_x) + (cos(ang_radians) * sphere_y));
		float sphere_z2 = sphere_z;
		float ray_end_x2 = (float)((cos(ang_radians+3.1415927f) * ray_end_x) - (sin(ang_radians+3.1415927f) * ray_end_y));
		float ray_end_y2 = (float)((sin(ang_radians) * ray_end_x) + (cos(ang_radians) * ray_end_y));
		float ray_end_z2 = ray_end_z;

		// tilt
		float sphere_x3 = (float)((cos(tilt_radians+3.1415927f) * sphere_x2) - (sin(tilt_radians+3.1415927f) * sphere_z2));
		float sphere_y3 = sphere_y2;
		float sphere_z3 = (float)((sin(tilt_radians) * sphere_x2) + (cos(tilt_radians) * sphere_z2));;
		float ray_end_x3 = (float)((cos(tilt_radians+3.1415927f) * ray_end_x2) - (sin(tilt_radians+3.1415927f) * ray_end_z2));
		float ray_end_y3 = ray_end_y2;
		float ray_end_z3 = (float)((sin(tilt_radians) * ray_end_x2) + (cos(tilt_radians) * ray_end_z2));

		// rotate
		float sphere_x4 = (float)((cos(rotate_radians+3.1415927f) * sphere_x3) - (sin(rotate_radians+3.1415927f) * sphere_y3));
		float sphere_y4 = (float)((sin(rotate_radians) * sphere_x3) + (cos(rotate_radians) * sphere_y3));
		float sphere_z4 = sphere_z3 + camera_height_mm;
		float ray_end_x4 = (float)((cos(rotate_radians+3.1415927f) * ray_end_x3) - (sin(rotate_radians+3.1415927f) * ray_end_y3));
		float ray_end_y4 = (float)((sin(rotate_radians) * ray_end_x3) + (cos(rotate_radians) * ray_end_y3));
		float ray_end_z4 = ray_end_z3 + camera_height_mm;

		// rotate
		float sphere_x5 = (float)((cos(-rotate_radians+3.1415927f) * sphere_x2) - (sin(-rotate_radians+3.1415927f) * sphere_y2));
		float sphere_y5 = (float)((sin(-rotate_radians) * sphere_x2) + (cos(-rotate_radians) * sphere_y2));

		//int px = centre_x_pixels + (int)((sphere_x4 - centre_x) * (radius_pixels+2) / (mirror_diameter_mm * 0.5f));
		//int py = centre_y_pixels + (int)((sphere_y4 - centre_y) * (radius_pixels+2) / (mirror_diameter_mm * 0.5f));
		int px = centre_x_pixels + (int)(sphere_x5 * (radius_pixels+2) / (mirror_diameter_mm * 0.5f));
		int py = centre_y_pixels + (int)(sphere_y5 * (radius_pixels+2) / (mirror_diameter_mm * 0.5f));
		//int px = centre_x_pixels + (int)(r * sin(ang_radians+rotate_radians));
		//int py = centre_y_pixels + (int)(r * cos(ang_radians+rotate_radians));
		if ((px > -1) && (px < ray_map_width) &&
			(py > -1) && (py < ray_map_height)) {

			if (ground) {
				float ix=9999, iy=9999,iz=0;

				intersection(
					0,0, 100,0,
					sphere_x4, sphere_z4, ray_end_x4, ray_end_z4,
					ix, iz);
				if (ix != 9999) {
					intersection(
						0,0, 100,0,
						sphere_y4, sphere_z4, ray_end_y4, ray_end_z4,
						iy, iz);
					if (iy != 9999) {
						ray_end_x4 = ix;
						ray_end_y4 = iy;
						ray_end_z4 = 0;
					}
				}
			}

			int n = py*ray_map_width + px;
			ray_map[n*6] = (int)sphere_x4;
			ray_map[n*6+1] = (int)sphere_y4;
			ray_map[n*6+2] = (int)sphere_z4;
			ray_map[n*6+3] = (int)ray_end_x4;
			ray_map[n*6+4] = (int)ray_end_y4;
			ray_map[n*6+5] = (int)ray_end_z4;

			mirror_lookup[n*2] = r;
			mirror_lookup[n*2+1] = ang_radians;

			mirror_map[n] = mirror+1;
		}
	}
}

void omni::create_ray_map_mirror(
	int mirror,
	int centre_x_pixels,
	int centre_y_pixels,
	int centre_x_mm,
	int centre_y_mm,
	int radius_pixels,
	int* ray_map,
	unsigned char* mirror_map,
	float* mirror_lookup,
	int ray_map_width,
	int ray_map_height,
	float tilt_radians,
	float rotate_radians,
	float mirror_diameter_mm,
	float dist_to_mirror_centre_mm,
	float camera_height_mm)
{
	float centre_x = 0;
	float centre_y = 0;
	float centre_z = dist_to_mirror_centre_mm;

	// tilt
	float centre_x2 = (float)((cos(tilt_radians+3.1415927f) * centre_x) - (sin(tilt_radians+3.1415927f) * centre_z));
	float centre_y2 = centre_y;
	float centre_z2 = (float)((sin(tilt_radians) * centre_x) + (cos(tilt_radians) * centre_z));;

	// rotate
	centre_x = (float)((cos(rotate_radians+3.1415927f) * centre_x2) - (sin(rotate_radians+3.1415927f) * centre_y2));
	centre_y = (float)((sin(rotate_radians) * centre_x2) + (cos(rotate_radians) * centre_y2));
	centre_z = centre_z2 + camera_height_mm;

	for (float r = radius_pixels; r >= 0; r -= 0.2f) {
		create_ray_map_mirror_inner(
			mirror,
			centre_x_pixels, centre_y_pixels,
			centre_x_mm, centre_y_mm,
			radius_pixels,
			ray_map,
			mirror_map,
			mirror_lookup,
			ray_map_width,
			ray_map_height,
			tilt_radians,
			rotate_radians,
			mirror_diameter_mm,
			dist_to_mirror_centre_mm,
			camera_height_mm,
			centre_x, centre_y, centre_z,
			r, false);
	}
}


void omni::show_mirror_lookup(
	unsigned char* img,
	int img_width,
	int img_height,
	float* lookup,
	bool show_radius,
	float outer_radius_percent)
{
	memset((void*)img, '\0', img_width*img_height*3);
	int radius_pixels = img_width * outer_radius_percent / 200;

	int n = img_width*img_height*2-2;
	for (int i = img_width*img_height*3-3; i >= 0; i -= 3, n-=2) {
        if ((lookup[n] != 0) && (lookup[n+1] != 0)) {
            int v = 0;
            if (show_radius) {
                v = 255 - (int)(lookup[n] * 255 / radius_pixels);
            }
            else {
            	v = (int)(lookup[n+1] * 255 / (2*3.1415927f));
            }
            img[i] = 255 - v;
            img[i+1] = v;
            img[i+2] = 0;
        }
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
		mirror_lookup = new float[img_width*img_height*2];
		feature_map = new unsigned char[img_width*img_height];
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
		float mirror_rotation = -(float)atan2(mirror_position[mirror*2], mirror_position[mirror*2+1]);
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

	}

    delete[] calibration_radius;
    delete[] lookup_pixel_x;
    delete[] lookup_ray;
    delete[] lookup_angle;

	create_ray_map(
	  	mirror_diameter,
	  	dist_to_mirror_backing,
	  	focal_length,
	  	outer_radius_percent,
	  	camera_height_mm,
	  	no_of_mirrors,
	  	mirror_position,
	  	mirror_position_pixels,
	    img_width,img_height,
	    ray_map,
	    mirror_map,
	    mirror_lookup);
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
 * \brief determines the closest point between two 3D rays.  Based on an algorithm written by John Burkardt
 */
void omni::rays_intercept(
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
	float& ix, float& iy, float& iz)
{
	float a;
	float b;
	float c;
	float d;
	float det;
	float e;
	int i;
	float sn;
	float tn;
	float pn[3];
	float qn[3];
	float u[3];
	float v[3];
	float w0[3];
	float p1[3];
	float p2[3];
	float q1[3];
	float q2[3];

	p1[0] = ray1_x_start;
	p1[1] = ray1_y_start;
	p1[2] = ray1_z_start;
	p2[0] = ray1_x_end;
	p2[1] = ray1_y_end;
	p2[2] = ray1_z_end;
	q1[0] = ray2_x_start;
	q1[1] = ray2_y_start;
	q1[2] = ray2_z_start;
	q2[0] = ray2_x_end;
	q2[1] = ray2_y_end;
	q2[2] = ray2_z_end;

	//
	//  The lines are identical.
	//  THIS CASE NOT SET UP YET
	//
	// if ( lines_exp_equal_3d ( p1, p2, q1, q2 ) ) then
	// end if
	//
	//  The lines are not identical, but parallel
	//  THIS CASE NOT SET UP YET.
	//
	// if ( lines_exp_parallel_3d ( p1, p2, q1, q2 ) ) then
	// end if
	//
	//  C: The lines are not identical, not parallel.
	//

	//
	//  Let U = (P2-P1) and V = (Q2-Q1) be the direction vectors on
	//  the two lines.
	//
	for (i = 0; i < 3; i++) {
		u[i] = p2[i] - p1[i];
	}
	for (i = 0; i < 3; i++) {
		v[i] = q2[i] - q1[i];
	}
	//
	//  Let SN be the unknown coordinate of the nearest point PN on line 1,
	//  so that PN = P(SN) = P1 + SN * (P2-P1).
	//
	//  Let TN be the unknown coordinate of the nearest point QN on line 2,
	//  so that QN = Q(TN) = Q1 + TN * (Q2-Q1).
	//
	//  Let W0 = (P1-Q1).
	//
	for (i = 0; i < 3; i++) {
		w0[i] = p1[i] - q1[i];
	}
	//
	//  The vector direction WC = P(SN) - Q(TC) is unique (among directions)
	//  perpendicular to both U and V, so
	//
	//    U dot WC = 0
	//    V dot WC = 0
	//
	//  or, equivalently:
	//
	//    U dot ( P1 + SN * (P2 - P1) - Q1 - TN * (Q2 - Q1) ) = 0
	//    V dot ( P1 + SN * (P2 - P1) - Q1 - TN * (Q2 - Q1) ) = 0
	//
	//  or, equivalently:
	//
	//    (u dot u ) * sn - (u dot v ) tc = -u * w0
	//    (v dot u ) * sn - (v dot v ) tc = -v * w0
	//
	//  or, equivalently:
	//
	//   ( a  -b ) * ( sn ) = ( -d )
	//   ( b  -c )   ( tc )   ( -e )
	//

	a = 0.0f;
	for (i = 0; i < 3; i++) {
		a = a + u[i] * u[i];
	}

	b = 0.0f;
	for (i = 0; i < 3; i++) {
		b = b + u[i] * v[i];
	}

	c = 0.0f;
	for (i = 0; i < 3; i++) {
		c = c + v[i] * v[i];
	}

	d = 0.0f;
	for (i = 0; i < 3; i++) {
		d = d + u[i] * w0[i];
	}

	e = 0.0f;
	for (i = 0; i < 3; i++) {
		e = e + v[i] * w0[i];
	}
	//
	//  Check the determinant.
	//
	det = -a * c + b * b;

	if (det == 0.0) {
		sn = 0.0;
		if (fabs(b) < fabs(c)) {
			tn = e / c;
		} else {
			tn = d / b;
		}
	} else {
		sn = (c * d - b * e) / det;
		tn = (b * d - a * e) / det;
	}

	for (i = 0; i < 3; i++) {
		pn[i] = p1[i] + sn * (p2[i] - p1[i]);
	}
	for (i = 0; i < 3; i++) {
		qn[i] = q1[i] + tn * (q2[i] - q1[i]);
	}

	ix = (pn[0] + qn[0])*0.5f;
	iy = (pn[1] + qn[1])*0.5f;
	iz = (pn[2] + qn[2])*0.5f;
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
    x = wx + (sc * ux); // + dx*0.5f;
    y = wy + (sc * uy);// + dy*0.5f;
    z = wz + (sc * uz);// + dz*0.5f;
    return (dx*dx + dy*dy + dz*dz);
}

void omni::create_ground_grid(
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
	int r, int g, int b)
{
	for (int x = tx_mm; x < bx_mm; x+= grid_dimension_mm) {
		create_obstacle(
			ground_height_mm,
			ground_img,
			ground_width,
			ground_height,
			tx_mm, ty_mm,
			bx_mm, by_mm,
			x, grid_ty_mm,
			x, grid_by_mm,
			height_mm,
			line_width_mm,
			r, g, b);
	}

	for (int y = ty_mm; y < by_mm; y += grid_dimension_mm) {
		create_obstacle(
			ground_height_mm,
			ground_img,
			ground_width,
			ground_height,
			tx_mm, ty_mm,
			bx_mm, by_mm,
			grid_tx_mm, y,
			grid_bx_mm, y,
			height_mm,
			line_width_mm,
			r, g, b);
	}
}

void omni::create_obstacle(
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
	int r, int g, int b)
{
	int dx = obstacle_bx_mm - obstacle_tx_mm;
	int dy = obstacle_by_mm - obstacle_ty_mm;
	int dist_mm = (int)sqrt(dx*dx + dy*dy);
	for (int i = 0; i < dist_mm; i += 4) {
		int x = obstacle_tx_mm + (i * dx / dist_mm);
		int y = obstacle_ty_mm + (i * dy / dist_mm);
		for (int xx = x - width_mm; xx <= x + width_mm; xx += 4) {
			int gx = (xx - tx_mm) * ground_width / (bx_mm - tx_mm);
			if ((gx > -1) && (gx < ground_width)) {
				for (int yy = y - width_mm; yy <= y + width_mm; yy += 4) {
					int gy = (yy - ty_mm) * ground_height / (by_mm - ty_mm);
					if ((gy > -1) && (gy < ground_height)) {
						int n = (gy * ground_width) + gx;
						ground_height_mm[n] = height_mm;
						ground_img[n*3] = b;
						ground_img[n*3 + 1] = g;
						ground_img[n*3 + 2] = r;
					}
				}
			}
		}
	}
}
