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
	horizontal = NULL;
	no_of_attention_boxes = 0;
	attention_box = NULL;
}

motion::~motion() {
	delete[] prev;
	delete[] curr;
	delete[] movement;
	if (horizontal != NULL) {
		delete[] horizontal;
		delete[] attention_box;
	}
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

void motion::attention_boxes(
	unsigned char* img,
	int img_width,
	int img_height,
	int ty, int by,
	bool clear,
	bool show)
{
	if (clear) no_of_attention_boxes = 0;

	int ty2 = ty * (MOTION_IMAGE_HEIGHT-1) / img_height;
	int by2 = by * (MOTION_IMAGE_HEIGHT-1) / img_height;

	if (horizontal == NULL) {
	    horizontal = new int[MOTION_IMAGE_WIDTH*2];
	    attention_box = new int[MOTION_MAX_ATTENTION_BOXES*4];
	}
	for (int x = 0; x < MOTION_IMAGE_WIDTH; x++) {
		int vertical = 0;
		int vertical2 = 0;
		for (int y = ty2; y < by2; y++) {
			int n = y*MOTION_IMAGE_WIDTH + x;
			if (movement[n] > 0) {
				if (vertical == 0) vertical = y;
				vertical2 = y;
			}
		}
		horizontal[x*2] = vertical;
		horizontal[x*2+1] = vertical2;
	}

	int radius = 6;
	for (int x = 0; x < MOTION_IMAGE_WIDTH; x++) {
		if (horizontal[x*2] > 0) {
			for (int r = -radius; r <= 0; r++) {
				int x2 = x+r;
				if (x2 < 0) x2 += MOTION_IMAGE_WIDTH;
				if (x2 >= MOTION_IMAGE_WIDTH) x2 -= MOTION_IMAGE_WIDTH;
				if (horizontal[x2*2] == 0) {
					horizontal[x2*2] = horizontal[x*2];
					horizontal[x2*2+1] = horizontal[x*2+1];
				}
			}
		}
	}

	for (int x = 0; x < MOTION_IMAGE_WIDTH-1; x++) {
		if ((horizontal[x*2] > 0) && (horizontal[(x+1)*2] == 0)) {
			int length = 0;
			int x2 = x;
			int min = horizontal[x*2];
			int max = horizontal[x*2+1];
			while ((horizontal[x2*2] > 0) && (length < MOTION_IMAGE_WIDTH/2)) {
				if (horizontal[x2*2] < min) min = horizontal[x2*2];
				if (horizontal[x2*2+1] > max) max = horizontal[x2*2+1];
				length++;
				x2--;
				if (x2 < 0) x2 += MOTION_IMAGE_WIDTH;
			}
			if (no_of_attention_boxes < MOTION_MAX_ATTENTION_BOXES) {
			    attention_box[no_of_attention_boxes*4] = x2;
			    attention_box[no_of_attention_boxes*4+1] = min;
			    attention_box[no_of_attention_boxes*4+2] = x;
			    attention_box[no_of_attention_boxes*4+3] = max;
			    no_of_attention_boxes++;
			}
		}
	}

	if (show) {
		int r = 0;
		int g = 255;
		int b = 0;
		for (int i = 0; i < no_of_attention_boxes; i++) {
			int tx = (attention_box[i*4]+1) * 2 * img_width / (MOTION_IMAGE_WIDTH*2);
			int ty = (attention_box[i*4+1]+1) * 2 * img_height / (MOTION_IMAGE_HEIGHT*2);
			int bx = (attention_box[i*4+2]+1) * 2 * img_width / (MOTION_IMAGE_WIDTH*2);
			int by = (attention_box[i*4+3]+1) * 2 * img_height / (MOTION_IMAGE_HEIGHT*2);
			if (bx > tx) {
			    drawing::drawLine(img, img_width, img_height, tx,ty,bx,ty, r,g,b, 0, false);
			    drawing::drawLine(img, img_width, img_height, tx,by,bx,by, r,g,b, 0, false);
			    drawing::drawLine(img, img_width, img_height, tx,ty,tx,by, r,g,b, 0, false);
			    drawing::drawLine(img, img_width, img_height, bx,ty,bx,by, r,g,b, 0, false);
			}
		}
	}
}
