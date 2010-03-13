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

#ifndef MOTION_H_
#define MOTION_H_

#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <sstream>

#define MOTION_IMAGE_WIDTH   128
#define MOTION_IMAGE_HEIGHT  64

using namespace std;

class motion {
protected:
	unsigned char* prev;
	unsigned char* curr;
	unsigned char* movement;
	bool first_update;

	int get_orientation_offset();
	void grab(
		unsigned char* img,
		int img_width,
		int img_height);
public:

	void update(
		unsigned char* img,
		int img_width,
		int img_height,
		int motion_threshold);
	void save(string filename);
	void load(string filename);
	void show(
		unsigned char* img,
		int img_width,
		int img_height);

	motion();
	virtual ~motion();
};

#endif /* MOTION_H_ */
