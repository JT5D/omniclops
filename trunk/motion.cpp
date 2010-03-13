/*
    Motion detection using OpenCV
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

#include "motion.h"

motion::motion() {
	first_update = true;
	prev = new unsigned char[MOTION_IMAGE_WIDTH*MOTION_IMAGE_HEIGHT];
	curr = new unsigned char[MOTION_IMAGE_WIDTH*MOTION_IMAGE_HEIGHT];
	movement = new unsigned char[MOTION_IMAGE_WIDTH*MOTION_IMAGE_HEIGHT];
}

motion::~motion() {
	delete[] prev;
	delete[] curr;
	delete[] movement;
}

void motion::grab(
	unsigned char* img,
	int img_width,
	int img_height)
{
	// remember the previous image
	memcpy((void*)prev, (void*)curr, MOTION_IMAGE_WIDTH*MOTION_IMAGE_HEIGHT);

	// store a new image
	int n = 0;
	for (int y = 0; y < MOTION_IMAGE_HEIGHT; y++) {
		for (int x = 0; x < MOTION_IMAGE_WIDTH; x++, n++) {
			int n2 = (((y * (img_height-1) / MOTION_IMAGE_HEIGHT)*img_width) +
					   (x * (img_width-1) / MOTION_IMAGE_WIDTH))*3;
			curr[n] = (img[n2] + img[n2+1] + img[n2+2])/3;
		}
	}
}

int motion::get_orientation_offset()
{
	int min_diff = 9999999;
	int offset = 0;
	for (int i = -MOTION_IMAGE_WIDTH/2; i < MOTION_IMAGE_WIDTH/2; i++) {
		int diff = 0;
		for (int x = 0; x < MOTION_IMAGE_WIDTH; x++) {
			int x2 = x + i;
			if (x2 >= MOTION_IMAGE_WIDTH) x2 -= MOTION_IMAGE_WIDTH;
			if (x2 < 0) x2 += MOTION_IMAGE_WIDTH;
			for (int y = 0; y < MOTION_IMAGE_HEIGHT; y++) {
				int n0 = y*MOTION_IMAGE_WIDTH + x2;
				int n1 = y*MOTION_IMAGE_WIDTH + x;
				diff += abs(prev[n1] - curr[n0]);
			}
		}
		if (diff < min_diff) {
			offset = i;
			min_diff = diff;
		}
	}
	return(offset);
}

void motion::update(
	unsigned char* img,
	int img_width,
	int img_height,
	int motion_threshold)
{
	memset((void*)movement, '\0', MOTION_IMAGE_WIDTH*MOTION_IMAGE_HEIGHT);

	grab(img, img_width, img_height);
	if (!first_update) {

		int offset = get_orientation_offset();

		for (int x = 0; x < MOTION_IMAGE_WIDTH; x++) {
			int x2 = x + offset;
			if (x2 >= MOTION_IMAGE_WIDTH) x2 -= MOTION_IMAGE_WIDTH;
			if (x2 < 0) x2 += MOTION_IMAGE_WIDTH;
			for (int y = 0; y < MOTION_IMAGE_HEIGHT; y++) {
				int n0 = y*MOTION_IMAGE_WIDTH + x2;
				int n1 = y*MOTION_IMAGE_WIDTH + x;
				if (abs(prev[n1] - curr[n0]) > motion_threshold) {
					movement[n1] = 255;
				}
			}
		}

	}
	first_update = false;
}

void motion::show(
	unsigned char* img,
	int img_width,
	int img_height)
{
	int n = 0;
	for (int y = 0; y < img_height; y++) {
		int yy = y * (MOTION_IMAGE_HEIGHT-1) / img_height;
		for (int x = 0; x < img_width; x++, n += 3) {
			int xx = x * (MOTION_IMAGE_WIDTH-1) / img_width;
			int n2 = yy* MOTION_IMAGE_WIDTH + xx;
			if (movement[n2] == 0) {
				img[n] = 0;
				img[n+1] = 0;
				img[n+2] = 0;
			}
		}
	}
}

void motion::save(string filename)
{
	FILE *file = fopen(filename.c_str(), "wb");
	if (file != NULL) {
		fwrite(prev, sizeof(unsigned char), MOTION_IMAGE_WIDTH*MOTION_IMAGE_HEIGHT, file);
		fclose(file);
	}
	else {
		printf("Couldn't save motion file\n");
	}
}

void motion::load(string filename)
{
	FILE *file = fopen(filename.c_str(), "rb");
	if (file != NULL) {
		fread(prev, sizeof(unsigned char), MOTION_IMAGE_WIDTH*MOTION_IMAGE_HEIGHT, file);
		fclose(file);
		first_update = false;
	}
	else {
		printf("Couldn't load motion file\n");
	}
}
