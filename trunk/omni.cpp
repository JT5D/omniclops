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

	calibration_map = NULL;

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
}

omni::~omni() {
	delete[] feature_x;
	delete[] features_per_row;
	delete[] row_sum;
	delete[] row_peaks;
	if (calibration_map != NULL)
		delete[] calibration_map;
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

		features_per_col[col_idx++] = no_of_feats;
	}

#ifdef OMNI_VERBOSE
	printf("%d horizontally oriented edge features located\n", no_of_features);
#endif

	return (no_of_features);
}

/* creates a calibration map */
void omni::make_map(float centre_of_distortion_x, /* centre of distortion x coordinate in pixels */
float centre_of_distortion_y, /* centre of distortion y coordinate in pixels */
float coeff_0, /* lens distortion polynomial coefficient 0 */
float coeff_1, /* lens distortion polynomial coefficient 1 */
float coeff_2, /* lens distortion polynomial coefficient 2 */
float rotation, /* camera rotation (roll angle) in radians */
float scale) { /* scaling applied */

	/* free existing map */
	if (calibration_map != NULL)
		free(calibration_map);

	polynomial* distortion_curve = new polynomial();
	distortion_curve->SetDegree(3);
	distortion_curve->SetCoeff(0, 0);
	distortion_curve->SetCoeff(1, coeff_0);
	distortion_curve->SetCoeff(2, coeff_1);
	distortion_curve->SetCoeff(3, coeff_2);

	int half_width = imgWidth / 2;
	int half_height = imgHeight / 2;
	calibration_map = new int[imgWidth * imgHeight];
	for (int x = 0; x < (int) imgWidth; x++) {

		float dx = x - centre_of_distortion_x;

		for (int y = 0; y < (int) imgHeight; y++) {

			float dy = y - centre_of_distortion_y;

			float radial_dist_rectified = (float) sqrt(dx * dx + dy * dy);
			if (radial_dist_rectified >= 0.01f) {

				double radial_dist_original = distortion_curve->RegVal(
						radial_dist_rectified);
				if (radial_dist_original > 0) {

					double ratio = radial_dist_original / radial_dist_rectified;
					float x2 = (float) round(centre_of_distortion_x + (dx
							* ratio));
					x2 = (x2 - (imgWidth / 2)) * scale;
					float y2 = (float) round(centre_of_distortion_y + (dy
							* ratio));
					y2 = (y2 - (imgHeight / 2)) * scale;

					// apply rotation
					double x3 = x2, y3 = y2;
					double hyp;
					if (rotation != 0) {
						hyp = sqrt(x2 * x2 + y2 * y2);
						if (hyp > 0) {
							double rot_angle = acos(y2 / hyp);
							if (x2 < 0)
								rot_angle = (3.1415927 * 2) - rot_angle;
							double new_angle = rotation + rot_angle;
							x3 = hyp * sin(new_angle);
							y3 = hyp * cos(new_angle);
						}
					}

					x3 += half_width;
					y3 += half_height;

					if (((int) x3 > -1) && ((int) x3 < (int) imgWidth)
							&& ((int) y3 > -1) && ((int) y3 < (int) imgHeight)) {

						int n = (y * imgWidth) + x;
						int n2 = ((int) y3 * imgWidth) + (int) x3;

						calibration_map[n] = n2;
					}
				}
			}
		}
	}
	delete distortion_curve;
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

/* saves stereo matches to file for use by other programs */
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

		for (int f = 0; f < no_of_feats_vertical; f++, feats_remaining--, index++) {

			m[index].x = feature_x[f];
			m[index].y = (unsigned short)((4 + (row * OMNI_VERTICAL_SAMPLING)) * OMNI_SUB_PIXEL);

			/* move to the next row */
			if (feats_remaining <= 0) {
				row++;
				feats_remaining = features_per_row[row];
			}
		}

		/* horizontally oriented features */
		int col = 0;
		feats_remaining = features_per_col[col];

		for (int f = 0; f < no_of_feats_horizontal; f++, feats_remaining--, index++) {

			m[index].y = feature_y[f];
			m[index].x = (unsigned short)((4 + (col * OMNI_HORIZONTAL_SAMPLING)) * OMNI_SUB_PIXEL);

		    /* move to the next column */
			if (feats_remaining <= 0) {
				col++;
				feats_remaining = features_per_col[col];
			}
		}

		fwrite(m, sizeof(MatchData), no_of_feats_horizontal+no_of_feats_vertical, file);
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
    int outer_radius_percent)
{
	const int no_of_radii = img_width;
	int* response = new int[no_of_radii];

	int max_radius = outer_radius_percent * img_width / 200;
	int min_radius = inner_radius_percent * img_width / 200;

	int x,y,dx,dy,r,dist,n=0,col;
	int cx = img_width/2;
	int cy = img_height/2;
	for (y = 0; y < img_height; y++) {
		dy = y - cy;
		for (x = 0; x < img_width; x++,n+=bytes_per_pixel) {
			dx = x - cx;
			r = dx*dx + dy*dy;

			/* integer square root */
			for (dist = 0; r >= (2* dist ) + 1; r -= (2 * dist++) + 1);

			if ((dist > min_radius) && (dist < max_radius)) {
                for (col = 0; col < bytes_per_pixel; col++) {
                	response[dist] += img[n+col];
                }
			}
		}
	}
	delete[] response;
}

void omni::get_calibration_image(
	unsigned char* img,
	int img_width,
	int img_height)
{
	int i,r;
	int line_width = 2;
    int initial_radius = img_width/20;
    int radius_increment = 30;
    int cx = img_width/2;
    int cy = -img_height/5;

    for (i = 0; i < img_width*img_height*3; i++) {
    	img[i] = 255;
    }

    r = initial_radius;
    while (r < img_width*2) {
    	drawing::drawCircle(img, img_width,img_height,cx,cy,r,0,0,0,line_width);
        r += radius_increment;
    }
}