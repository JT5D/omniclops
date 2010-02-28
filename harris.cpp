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

#include "harris.h"

harris::harris() {
	// TODO Auto-generated constructor stub

}

harris::~harris() {
	// TODO Auto-generated destructor stub
}


void harris::get_features(
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
	vector<int> &features)
{
	features.clear();

	if (frame1_1C == NULL) frame1_1C = cvCreateImage(cvSize(frame1->width,frame1->height), 8, 1);
	if (eig_image == NULL) eig_image = cvCreateImage(cvSize(frame1->width,frame1->height), IPL_DEPTH_32F, 1);
	if (temp_image == NULL) temp_image = cvCreateImage(cvSize(frame1->width,frame1->height), IPL_DEPTH_32F, 1);
	if (pyramid1 == NULL) pyramid1 = cvCreateImage(cvSize(frame1->width,frame1->height), IPL_DEPTH_8U, 1);

	FILE *file = NULL;
	if (harris_filename != "") {
		file = fopen(harris_filename.c_str(), "wb");
	}

    int number_of_features = 2000;
	cvConvertImage(frame1, frame1_1C, CV_BGR2GRAY);
    CvPoint2D32f frame1_features[number_of_features];

    cvGoodFeaturesToTrack(frame1_1C, eig_image, temp_image, frame1_features, &
number_of_features, .01, minimum_separation, NULL, 16, 1, 0.001);

    for(int i = 0; i < number_of_features; i++) {
        features.push_back((int)(frame1_features[i].x));
        features.push_back((int)(frame1_features[i].y));
    }

	//remove features outside of the outer perimeter
	int outer_radius_pixels_sqr = max_radius_pixels*max_radius_pixels;
	for (int i = (int)features.size()-2; i >= 0; i-=2) {
		int dx = features[i] - centre_x;
		int dy = features[i+1] - centre_y;
		int r = dx*dx + dy*dy;
		if (r > outer_radius_pixels_sqr) {
			features.erase(features.begin()+i);
			features.erase(features.begin()+i);
		}
	}

	// save features to file
	if (harris_filename != "") {
		int feats = (int)features.size()/2;
		fprintf(file, "%d",feats);
		for(int i = 0; i < feats; i++) {
			fprintf(file,"%d", features[i*2]);
			fprintf(file,"%d", features[i*2 + 1]);
		}

		fclose(file);
	}
}
