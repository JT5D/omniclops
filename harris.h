/*
    Harris corner detection using OpenCV
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

#ifndef HARRIS_H_
#define HARRIS_H_

#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <sstream>

using namespace std;


class harris {
public:
	static void get_features(
		IplImage* frame1,
	    IplImage *&frame1_1C,
		IplImage *&eig_image,
		IplImage *&temp_image,
		IplImage *&pyramid1,
		int minimum_separation,
		int centre_x,
		int centre_y,
		int max_radius_pixels,
		std::string harris_filename,
		vector<int> &features);

	static void proximal_to_edges(
		vector<int> &features,
		vector<int> &edge_heights,
		int search_radius,
		vector<int> &proximal);
};

#endif /* HARRIS_H_ */
