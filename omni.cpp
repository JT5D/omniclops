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
	calibration_map = NULL;
	feature_radius_index = NULL;
	unwarp_lookup = NULL;
	unwarp_lookup_reverse = NULL;

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
	if (ray_map != NULL) {
		delete[] ray_map;
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
	int centre_x,
	int centre_y,
	float outer_radius_percent,
	float inner_radius_percent,
	float inner_aspect,
	int no_of_struts,
	float * strut_angles,
	float strut_width)
{
	int i,x,y,dx,dy,r,n,col,tx=0,ty=0,bx=0,by=0;
	int max = (int)(img_width * outer_radius_percent / 200);
	int min = (int)(img_width * inner_radius_percent / 200);
	if (inner_aspect > 0) {
		int w = img_width * inner_radius_percent / 100;
		int h = (int)(w * inner_aspect);
		tx = centre_x - (w/2);
		bx = tx + w;
		ty = centre_y - (h/2);
		by = ty + h; 
	}
	for (y=centre_y-5;y<=centre_y+5;y++) {
		n = ((y * img_width) + centre_x)*bytes_per_pixel;
		img[n] = 255;
		img[n+1] = 255;
		img[n+2] = 255;
	}
	for (x=centre_x-5;x<=centre_x+5;x++) {
		n = ((centre_y * img_width) + x)*bytes_per_pixel;
		img[n] = 255;
		img[n+1] = 255;
		img[n+2] = 255;
	}
	max *= max;
	min *= min;
	for (y = 0; y < img_height; y++) {
		dy = y - centre_y;
		for (x = 0; x < img_width; x++) {
			dx = x - centre_x;
			r = dx*dx + dy*dy;
			if ((r > max) || ((r < min) && (inner_aspect==0)) ||
				((x>tx) && (x<bx) && (y>ty) && (y<by))) {
				n = ((y * img_width) + x)*bytes_per_pixel;
				for (col = 0; col < bytes_per_pixel; col++) {
					img[n+col]=0;
				}
			}
		}
	}
	float angle_width = strut_width*3.1415927f/360;
	for (i = 0; i < no_of_struts; i++) {
		float angle = (strut_angles[i]+180)*3.1415927f/180;
		for (r = 0; r < img_width/2; r++) {
			int x0 = centre_x + (int)(r*sin(angle-angle_width));
			int y0 = centre_y + (int)(r*cos(angle-angle_width));
			int x1 = centre_x + (int)(r*sin(angle+angle_width));
			int y1 = centre_y + (int)(r*cos(angle+angle_width));
			dx = x1 - x0;
			dy = y1 - y0;
			int length = (int)sqrt(dx*dx+dy*dy);
			for (int j = 0; j < length; j++) {
				x = x0 + (j*dx/length);
				y = y0 + (j*dy/length);
				if ((x>=0) && (x<img_width) && (y>=0) && (y<img_height)) {
					n = ((y * img_width) + x)*bytes_per_pixel;
					for (col = 0; col < bytes_per_pixel; col++) {
						img[n+col]=0;
					}
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

		fwrite(m, sizeof(MatchData), no_of_radial_lines, file);
		delete[] m;

		fclose(file);
	}
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
	int squares_across = 10;
	int squares_down = 7;
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

void omni::create_ray_map(
	float mirror_diameter_mm,
	float dist_to_mirror_mm,
	float focal_length_mm,
	float inner_radius_percent,
	float outer_radius_percent,
	float camera_height_mm,
	int img_width,
	int img_height)
{
	create_ray_map(
		mirror_diameter_mm,
		dist_to_mirror_mm,
		focal_length_mm,
		inner_radius_percent,
		outer_radius_percent,
		camera_height_mm,
		img_width,
		img_height,
		true, true);
}


void omni::create_ray_map(
	float mirror_diameter_mm,
	float dist_to_mirror_mm,
	float focal_length_mm,
	float inner_radius_percent,
	float outer_radius_percent,
	float camera_height_mm,
	int img_width,
	int img_height,
	bool clear_map,
	bool update_unwarp)
{
	if (ray_map == NULL) {
		ray_map = new int[img_width*img_height*6];
		memset((void*)ray_map,'\0',img_width*img_height*6*sizeof(int));

		unwarp_lookup = new int[img_width*img_height];
		unwarp_lookup_reverse = new int[img_width*img_height];
		memset((void*)unwarp_lookup, '\0', img_width*img_height*sizeof(int));
		memset((void*)unwarp_lookup_reverse, '\0', img_width*img_height*sizeof(int));
	}

	int pixels = img_width*img_height;
	float half_pi = 3.1415927f/2;
	float dist_to_mirror_centre = focal_length_mm + dist_to_mirror_mm + (mirror_diameter_mm * 0.5f);
	float ang_incr = 3.1415927f / (180*100);
	float inner_radius_pixels = inner_radius_percent * img_width / 200;
	float outer_radius_pixels = outer_radius_percent * img_width / 200;
	float sphere_x = (float)fabs(mirror_diameter_mm*0.5f*sin(half_pi));
	float sphere_y = dist_to_mirror_centre - (float)(mirror_diameter_mm*0.5f*cos(half_pi));
	float max_viewing_angle = (float)atan(sphere_x / sphere_y);
	float pixel_x = 0;
	epipole = 0;
	int max_index = (int)(half_pi / ang_incr);
	float lookup_pixel_x[max_index];
	float lookup_ray[max_index*4];
	float lookup_angle[max_index];
	memset((void*)lookup_pixel_x,'\0',max_index*sizeof(float));
	memset((void*)lookup_ray,'\0',max_index*4*sizeof(float));
	memset((void*)lookup_angle,'\0',max_index*sizeof(float));
	int i = 0;
	float min_angle = 9999;
	float max_angle = -9999;
	for (float ang = ang_incr; ang < half_pi; ang += ang_incr, i++) {
		sphere_x = (float)fabs(mirror_diameter_mm*0.5f*sin(ang));
		sphere_y = camera_height_mm + dist_to_mirror_centre - (float)fabs(mirror_diameter_mm*0.5f*cos(ang));
		float viewing_angle = (float)atan(sphere_x / (sphere_y - camera_height_mm));
		pixel_x = viewing_angle * outer_radius_pixels / max_viewing_angle;
		float ang2 = viewing_angle + (ang*2);
		if (ang2 >= half_pi) {
			if (epipole == 0) {
				epipole = (int)pixel_x;
			}
		}
		float ray_end_y = 0;
		float ray_end_x = sphere_x + (sphere_y * (float)tan(ang2));
		if (ray_end_x < 0) {
			ray_end_x = sphere_x - (sphere_y * (float)tan(ang2));
			ray_end_y = sphere_y * 2;
		}
		lookup_pixel_x[i] = pixel_x;
		lookup_ray[i*4] = sphere_x;
		lookup_ray[i*4+1] = sphere_y;
		lookup_ray[i*4+2] = ray_end_x;
		lookup_ray[i*4+3] = ray_end_y;
		lookup_angle[i] = (float)atan2(ray_end_x, dist_to_mirror_centre);
		if (lookup_angle[i] < min_angle) min_angle = lookup_angle[i];
		if (lookup_angle[i] > max_angle) max_angle = lookup_angle[i];
	}

	float calibration_radius[img_width*4];

	// create unwarp lookup
	for (int j = 0; j < img_height; j++) {
		float target_ang = min_angle + (j * (max_angle-min_angle) / (img_width/2));
		int k = 0;
		for (k = 0; k < i; k++) {
			if (lookup_angle[k] >= target_ang) break;
		}
		calibration_radius[j] = lookup_pixel_x[k];
	}

	int n = 0;
	float angle;
	if (update_unwarp) {
		for (int x = 0; x < img_width; x++) {
			angle = x * 3.1415927f*2 / img_width;
			float sin_ang = (float)sin(angle);
			float cos_ang = (float)cos(angle);
			for (int y = 0; y < img_height; y++) {
				float raw_radius = calibration_radius[y];
				int raw_x = (img_width/2) + (int)(raw_radius * sin_ang);
				int raw_y = (img_height/2) + (int)(raw_radius * cos_ang);
				n = ((img_height-1-y) * img_width)+x;
				int n2 = (raw_y * img_width)+raw_x;
				if ((n2 > -1) && (n2 < pixels)) {
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
	if (clear_map) memset((void*)ray_map,'\0', pixels*6*sizeof(int));
	int cx = img_width/2;
	int cy = img_height/2;
	n = 0;
	for (int y = 0; y < img_height; y++) {
		int dy = y - cy;
		for (int x = 0; x < img_width; x++, n++) {
			int dx = x - cx;
			int r = (int)sqrt(dx*dx + dy*dy);
			if ((r >= inner_radius_pixels) && (r <= outer_radius_pixels)) {
				angle = (float)acos(dy/(float)r);
				if (dx < 0) angle = (3.1415927f*2) - angle;
				ray_map[n*6] = (int)(calibration_radius[r*4]*sin(angle));
				ray_map[n*6+1] = (int)(calibration_radius[r*4]*cos(angle));
				ray_map[n*6+2] = (int)(calibration_radius[r*4+1]);

				ray_map[n*6+3] = (int)(calibration_radius[r*4+2]*sin(angle));
				ray_map[n*6+4] = (int)(calibration_radius[r*4+2]*cos(angle));
				ray_map[n*6+5] = (int)(calibration_radius[r*4+3]);
			}
		}
	}
}

void omni::show_ray_map_side(
	unsigned char* img,
	int img_width,
	int img_height,
	int max_height_mm,
	int focal_length_mm,
	int camera_height_mm)
{
	for (int i = 0; i < img_width*img_height*3; i++)
		img[i]=255;

    int camera_height_y = camera_height_mm * img_height / max_height_mm;

	int n = (img_height/2) * img_width * 6;
	const int step = 1;
    for (int x = 0; x < img_width; x+=step, n+=6*step) {
    	float start_x = ray_map[n];
    	float start_z = ray_map[n+2];
    	float end_x = ray_map[n+3];
    	float end_z = ray_map[n+5];

    	int x0 = (img_width/2) + (int)(start_x * img_width / max_height_mm);
    	int y0 = img_height-1-(int)(start_z * img_height / max_height_mm);

    	int x1 = (img_width/2) + (int)(end_x * img_width / max_height_mm);
    	int y1 = img_height-1-(int)(end_z * img_height / max_height_mm);

    	drawing::drawLine(img,img_width,img_height,x0,y0,x1,y1,0,0,0,0,false);
    	drawing::drawLine(img,img_width,img_height,x0,y0,img_width/2,img_height-1-camera_height_y,0,0,0,0,false);
    }
    int focal_plane_y = img_height-1-camera_height_y-(focal_length_mm * img_height / max_height_mm);
    drawing::drawLine(img,img_width,img_height,img_width*48/100,focal_plane_y,img_width*52/100,focal_plane_y,255,0,0,0,false);
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
			if ((ray_map[n+3] > min_x) && (ray_map[n+3] < max_x) &&
				(ray_map[n+4] > min_y) && (ray_map[n+4] < max_y)) {
				int xx = (ray_map[n+3] - min_x) * (img_width-1) / (max_x-min_x);
				int yy = (ray_map[n+4] - min_y) * (img_height-1) / (max_y-min_y);
				int xx2 = (ray_map[n+3+6] - min_x) * (img_width-1) / (max_x-min_x);
				int yy2 = (ray_map[n+4+6+(img_width*6)] - min_y) * (img_height-1) / (max_y-min_y);
				int n0 = ((y * img_width) + x)*3;
				for (int yyy = yy; yyy <= yy2; yyy++) {
					if ((yyy > -1) && (yyy < img_height)) {
						for (int xxx = xx; xxx <= xx2; xxx++) {
							if ((xxx > -1) && (xxx < img_width)) {
								int n2 = ((yyy * img_width) + xxx)*3;
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
