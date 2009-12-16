/*
    omniclops
    A command line utility for omnidirectional vision
    Copyright (C) 2009 Bob Mottram
    fuzzgun@gmail.com

    Requires packages:
		libgstreamer-plugins-base0.10-dev
		libgst-dev

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

#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappbuffer.h>
#include <sstream>

#include "anyoption.h"
#include "drawing.h"
#include "omni.h"
//#include "motionmodel.h"
#include "fast.h"
#include "libcam.h"

#define VERSION 0.2

using namespace std;

void compute_optical_flow(
	IplImage* frame1,
	IplImage* frame2,
	IplImage* output,
	int number_of_features,
	int max_magnitude,
    IplImage *&frame1_1C,
    IplImage *&frame2_1C,
	IplImage *&eig_image,
	IplImage *&temp_image,
	IplImage *&pyramid1,
	IplImage *&pyramid2,
	std::string flow_filename)
{
	if (frame1_1C == NULL) {
		frame1_1C = cvCreateImage(cvSize(frame1->width,frame1->height), 8, 1);
		frame2_1C = cvCreateImage(cvSize(frame1->width,frame1->height), 8, 1);
		eig_image = cvCreateImage(cvSize(frame1->width,frame1->height), IPL_DEPTH_32F, 1);
		temp_image = cvCreateImage(cvSize(frame1->width,frame1->height), IPL_DEPTH_32F, 1);
		pyramid1 = cvCreateImage(cvSize(frame1->width,frame1->height), IPL_DEPTH_8U, 1);
		pyramid2 = cvCreateImage(cvSize(frame1->width,frame1->height), IPL_DEPTH_8U, 1);
	}

	struct MatchData {
		unsigned short int x0;
		unsigned short int y0;
		unsigned short int x1;
		unsigned short int y1;
	};

	FILE *file = NULL;
	MatchData *m = NULL;
	if (flow_filename != "") {
		file = fopen(flow_filename.c_str(), "wb");
		m = new MatchData[number_of_features*4];
		memset((void*)m,'\0',number_of_features*4*sizeof(unsigned short int));
	}

	cvConvertImage(frame1, frame1_1C, CV_BGR2GRAY);
	cvConvertImage(frame2, frame2_1C, CV_BGR2GRAY);

    CvPoint2D32f frame1_features[number_of_features];

    cvGoodFeaturesToTrack(frame1_1C, eig_image, temp_image, frame1_features, &
number_of_features, .01, .01, NULL);

    CvPoint2D32f frame2_features[number_of_features];

    char optical_flow_found_feature[number_of_features];
    float optical_flow_feature_error[number_of_features];
    CvSize optical_flow_window = cvSize(3,3);
    CvTermCriteria optical_flow_termination_criteria
        = cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 );

    cvCalcOpticalFlowPyrLK(frame1_1C, frame2_1C, pyramid1, pyramid2, frame1_features,
frame2_features, number_of_features, optical_flow_window, 5,
optical_flow_found_feature, optical_flow_feature_error,
optical_flow_termination_criteria, 0);

    /* let's paint and make blended drinks */
    for(int i = 0; i < number_of_features; i++)
    {
         if (optical_flow_found_feature[i] == 0) continue;
         int line_thickness = 1;
         CvScalar line_color = CV_RGB(255,0,0);
         CvPoint p,q;
         p.x = (int) frame1_features[i].x;
         p.y = (int) frame1_features[i].y;
         q.x = (int) frame2_features[i].x;
         q.y = (int) frame2_features[i].y;
         double hypotenuse = sqrt( (p.y - q.y)*(p.y - q.y) + (p.x - q.x)*(p.x - q.x) );
         if ((hypotenuse > 3) && (hypotenuse < max_magnitude)) {
			 if (flow_filename != "") {
				 m[i].x0 = (unsigned short int)p.x;
				 m[i].y0 = (unsigned short int)p.y;
				 m[i].x1 = (unsigned short int)q.x;
				 m[i].y1 = (unsigned short int)q.y;
			 }
			 double angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
			 q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
			 q.y = (int) (p.y - 3 * hypotenuse * sin(angle));
			 cvLine( output, p, q, line_color, line_thickness, CV_AA, 0 );
			 p.x = (int) (q.x + 5 * cos(angle + 3.1415927 / 4));
			 p.y = (int) (q.y + 5 * sin(angle + 3.1415927 / 4));
			 cvLine( output, p, q, line_color, line_thickness, CV_AA, 0 );
			 p.x = (int) (q.x + 5 * cos(angle - 3.1415927 / 4));
			 p.y = (int) (q.y + 5 * sin(angle - 3.1415927 / 4));
			 cvLine( output, p, q, line_color, line_thickness, CV_AA, 0 );
         }
    }

	if (flow_filename != "") {
		fprintf(file, "%d",number_of_features);
		fwrite(m, sizeof(MatchData), number_of_features, file);
		delete[] m;

		fclose(file);
	}
}


int main(int argc, char* argv[]) {
  int ww = 640;
  int hh = 480;
  int skip_frames = 1;
  bool show_FAST = false;
  bool show_features = false;
  bool show_ground_features = false;
  bool show_ground = false;
  bool show_lines = false;
  bool unwarp_features = false;
  float outer_radius = 115;
  //int FOV_degrees = 50;

  // Port to start streaming from - second video will be on this + 1
  int start_port = 5000;

  AnyOption *opt = new AnyOption();
  assert(opt != NULL);

  // help
  opt->addUsage( "Example: " );
  opt->addUsage( "  omniclops --dev /dev/video1 -w 320 -h 240 --features" );
  opt->addUsage( " " );
  opt->addUsage( "Usage: " );
  opt->addUsage( "" );
  opt->addUsage( "     --dev                  Video device to be used");
  opt->addUsage( " -w  --width                Image width in pixels");
  opt->addUsage( " -h  --height               Image height in pixels");
  opt->addUsage( "     --diam                 Diameter of the spherical mirror in millimetres");
  opt->addUsage( "     --dist                 Distance from camera lens to nearest point on the mirror surface in millimetres");
  opt->addUsage( "     --focal                Focal length in millimetres");
  opt->addUsage( "     --elevation            Height of the camera above the ground in millimetres");
  opt->addUsage( "     --radius               Outer radius as a percentage of the image width");
  opt->addUsage( "     --inner                Inner radius as a percentage of the image width");
  opt->addUsage( "     --features             Show edge features");
  opt->addUsage( "     --lines                Show radial lines");
  opt->addUsage( "     --groundfeatures       Show edge features on the ground plane");
  opt->addUsage( "     --ground               Show ground plane");
  opt->addUsage( "     --range                Maximum range when displaying ground plane");
  opt->addUsage( "     --fast                 Show FAST corners");
  opt->addUsage( "     --descriptors          Saves feature descriptor for each FAST corner");
  opt->addUsage( "     --raypaths             Saves image showing ray paths");
  opt->addUsage( "     --rays                 Saves file containing rays for each observed edge feature");
  opt->addUsage( "     --fov                  Field of view in degrees");
  opt->addUsage( " -f  --fps                  Frames per second");
  opt->addUsage( " -s  --skip                 Skip this number of frames");
  opt->addUsage( " -V  --version              Show version number");
  opt->addUsage( "     --save                 Save raw image");
  opt->addUsage( "     --savecalib            Save calibration image");
  opt->addUsage( "     --saveedges            Save edges to file");
  opt->addUsage( "     --saveflow             Save optical flow vectors");
  opt->addUsage( "     --saveradial           Save radial lines to file");
  opt->addUsage( "     --flip                 Flip the image");
  opt->addUsage( "     --unwarp               Unwarp the image");
  opt->addUsage( "     --unwarpfeatures       Unwarp edge features");
  opt->addUsage( "     --flow                 Compute optical flow");
  opt->addUsage( "     --stream               Stream output using gstreamer");
  opt->addUsage( "     --headless             Disable video output (for use with --stream)");
  opt->addUsage( "     --help                 Show help");
  opt->addUsage( "" );

  opt->setOption(  "fast" );
  opt->setOption(  "diam" );
  opt->setOption(  "dist" );
  opt->setOption(  "focal" );
  opt->setOption(  "elevation" );
  opt->setOption(  "range" );
  opt->setOption(  "raypaths" );
  opt->setOption(  "rays" );
  opt->setOption(  "radius" );
  opt->setOption(  "inner" );
  opt->setOption(  "descriptors" );
  opt->setOption(  "save" );
  opt->setOption(  "savecalib" );
  opt->setOption(  "saveedges" );
  opt->setOption(  "saveradial" );
  opt->setOption(  "saveflow" );
  opt->setOption(  "fps", 'f' );
  opt->setOption(  "dev" );
  opt->setOption(  "width", 'w' );
  opt->setOption(  "height", 'h' );
  opt->setOption(  "skip", 's' );
  opt->setOption(  "fov" );
  opt->setFlag(  "help" );
  opt->setFlag(  "flip" );
  opt->setFlag(  "unwarp" );
  opt->setFlag(  "unwarpfeatures" );
  opt->setFlag(  "flow" );
  opt->setFlag(  "version", 'V' );
  opt->setFlag(  "stream"  );
  opt->setFlag(  "headless"  );
  opt->setFlag(  "features" );
  opt->setFlag(  "lines" );
  opt->setFlag(  "groundfeatures" );
  opt->setFlag(  "ground" );

  opt->processCommandArgs(argc, argv);

  if( ! opt->hasOptions())
  {
      // print usage if no options
      opt->printUsage();
      delete opt;
      return(0);
  }

  if( opt->getFlag( "version" ) || opt->getFlag( 'V' ) )
  {
      printf("Version %f\n", VERSION);
      delete opt;
      return(0);
  }

  bool stream = false;
  if( opt->getFlag( "stream" ) ) {
	  stream = true;
  }

  bool headless = false;
  if( opt->getFlag( "headless" ) ) {
      headless = true;
  }

  bool flip_image = false;
  if( opt->getFlag( "flip" ) )
  {
	  flip_image = true;
  }

  bool unwarp_image = false;
  if( opt->getFlag( "unwarp" ) )
  {
	  unwarp_image = true;
  }

  bool save_image = false;
  std::string save_filename = "";
  if( opt->getValue( "save" ) != NULL  ) {
  	  save_filename = opt->getValue("save");
  	  if (save_filename == "") save_filename = "image_";
  	  save_image = true;
  }

  std::string calibration_image_filename = "";
  if( opt->getValue( "savecalib" ) != NULL  ) {
	  calibration_image_filename = opt->getValue("savecalib");
  	  if (calibration_image_filename == "") calibration_image_filename = "calibration.jpg";
  }

  std::string edges_filename = "";
  if( opt->getValue( "saveedges" ) != NULL  ) {
	  edges_filename = opt->getValue("saveedges");
  	  if (edges_filename == "") edges_filename = "edges.dat";
  }

  std::string radial_lines_filename = "";
  if( opt->getValue( "saveradial" ) != NULL  ) {
	  radial_lines_filename = opt->getValue("saveradial");
  	  if (radial_lines_filename == "") radial_lines_filename = "radial.dat";
  }

  std::string flow_filename = "";
  if( opt->getValue( "saveflow" ) != NULL  ) {
	  flow_filename = opt->getValue("saveflow");
  	  if (flow_filename == "") flow_filename = "flow.dat";
  }

  if( opt->getFlag( "help" ) ) {
      opt->printUsage();
      delete opt;
      return(0);
  }

  if( opt->getFlag( "ground" ) ) {
	  show_lines = false;
	  show_ground = true;
	  show_ground_features = false;
	  show_features = false;
	  show_FAST = false;
	  unwarp_features = false;
  }

  bool optical_flow = false;
  if( opt->getFlag( "flow" ) ) {
      optical_flow = true;
  }

  if( opt->getFlag( "features" ) ) {
	  show_lines = false;
	  show_ground = false;
	  show_ground_features = false;
	  show_features = true;
	  show_FAST = false;
	  unwarp_features = false;
  }

  if( opt->getFlag( "unwarpfeatures" ) ) {
	  show_lines = false;
	  show_ground = false;
	  show_ground_features = false;
	  show_features = false;
	  unwarp_features = true;
	  show_FAST = false;
  }

  if( opt->getFlag( "lines" ) ) {
	  show_lines = true;
	  show_ground = false;
	  show_ground_features = false;
	  show_features = false;
	  show_FAST = false;
	  unwarp_features = false;
  }

  if( opt->getFlag( "groundfeatures" ) ) {
	  show_lines = false;
	  show_ground = false;
	  show_ground_features = true;
	  show_features = false;
	  show_FAST = false;
	  unwarp_features = false;
  }

  float focal_length = 3.6f;
  if( opt->getValue( "focal" ) != NULL  ) {
	  focal_length = atof(opt->getValue("focal"));
  }

  float camera_height = 150;
  if( opt->getValue( "elevation" ) != NULL  ) {
	  camera_height = atof(opt->getValue("elevation"));
  }

  std::string save_ray_paths_image = "";
  if( opt->getValue( "raypaths" ) != NULL  ) {
	  save_ray_paths_image = opt->getValue("raypaths");
	  if (save_ray_paths_image == "") save_ray_paths_image = "ray_paths.jpg";
  }

  std::string save_rays = "";
  if( opt->getValue( "rays" ) != NULL  ) {
	  save_rays = opt->getValue("rays");
	  if (save_rays == "") save_rays = "rays.dat";
  }

  float mirror_diameter = 60;
  if( opt->getValue( "diam" ) != NULL  ) {
	  mirror_diameter = atof(opt->getValue("diam"));
  }

  float dist_to_mirror = 50;
  if( opt->getValue( "dist" ) != NULL  ) {
	  dist_to_mirror = atof(opt->getValue("dist"));
  }

  if( opt->getValue( "radius" ) != NULL  ) {
	  outer_radius = atoi(opt->getValue("radius"));
  }

  float inner_radius = 30;
  if( opt->getValue( "inner" ) != NULL  ) {
	  inner_radius = atoi(opt->getValue("inner"));
  }

  int range_mm = 500;
  if( opt->getValue( "range" ) != NULL  ) {
	  range_mm = atoi(opt->getValue("range"));
  }

  int desired_corner_features = 70;
  if( opt->getValue( "fast" ) != NULL  ) {
	  show_lines = false;
	  show_ground = false;
	  show_ground_features = false;
	  show_FAST = true;
	  show_features = false;
	  unwarp_features = false;
	  desired_corner_features = atoi(opt->getValue("fast"));
	  if (desired_corner_features > 150) desired_corner_features=150;
	  if (desired_corner_features < 50) desired_corner_features=50;
  }

  //if( opt->getValue( "fov" ) != NULL  ) {
  	  //FOV_degrees = atoi(opt->getValue("fov"));
  //}

  std::string dev = "/dev/video1";
  if( opt->getValue( "dev" ) != NULL  ) {
  	  dev = opt->getValue("dev");
  }

  if( opt->getValue( 'w' ) != NULL  || opt->getValue( "width" ) != NULL  ) {
  	  ww = atoi(opt->getValue("width"));
  }

  if( opt->getValue( 'h' ) != NULL  || opt->getValue( "height" ) != NULL  ) {
  	  hh = atoi(opt->getValue("height"));
  }

  int fps = 30;
  if( opt->getValue( 'f' ) != NULL  || opt->getValue( "fps" ) != NULL  ) {
	  fps = atoi(opt->getValue("fps"));
  }

  std::string descriptors_filename = "";
  if( opt->getValue( "descriptors" ) != NULL  ) {
	  descriptors_filename = opt->getValue("descriptors");
  }

  if( opt->getValue( 's' ) != NULL  || opt->getValue( "skip" ) != NULL  ) {
	  skip_frames = atoi(opt->getValue("skip"));
  }

  delete opt;

  Camera c(dev.c_str(), ww, hh, fps);

  std::string image_title = "Image";
  if (unwarp_image) image_title = "Unwarped";
  if (show_FAST) image_title = "FAST corners";
  if (show_features) image_title = "Image features";

//cout<<c.setSharpness(3)<<"   "<<c.minSharpness()<<"  "<<c.maxSharpness()<<" "<<c.defaultSharpness()<<endl;

  if ((!save_image) &&
	  (!headless)) {

      cvNamedWindow(image_title.c_str(), CV_WINDOW_AUTOSIZE);
      if (!show_FAST) {
          cvNamedWindow(image_title.c_str(), CV_WINDOW_AUTOSIZE);
      }
  }

  IplImage *l=cvCreateImage(cvSize(ww, hh), 8, 3);
  IplImage *prev=cvCreateImage(cvSize(ww, hh), 8, 3);
  IplImage *flow=cvCreateImage(cvSize(ww, hh), 8, 3);
  IplImage *frame1_1C = NULL;
  IplImage *frame2_1C = NULL;
  IplImage *eig_image = NULL;
  IplImage *temp_image = NULL;
  IplImage *pyramid1 = NULL;
  IplImage *pyramid2 = NULL;
  unsigned char *l_=(unsigned char *)l->imageData;
  unsigned char *prev_=(unsigned char *)prev->imageData;
  unsigned char *flow_=(unsigned char *)flow->imageData;

  /* feature detection params */
  int inhibition_radius = 6;
  unsigned int minimum_response = 250;

  omni* lcam = new omni(ww, hh);
  //motionmodel* motion = new motionmodel();
  fast* corners = new fast();

  /*
   * Send the video over a network for use in embedded applications
   * using the gstreamer library.
   */
  GstElement* l_source = NULL;
  GstBuffer* l_app_buffer = NULL;
  GstFlowReturn ret;

  // Yuck
  std::stringstream lp_str;
  lp_str << start_port;

  std::string caps;

  if( stream ) {
	// Initialise gstreamer and glib
      	gst_init( NULL, NULL );
	GError* l_error = 0;
	GstElement* l_pipeline = 0;

        caps = "image/jpeg";

	// Can replace this pipeline with anything you like (udpsink, videowriters etc)
	std::string l_pipetext = "appsrc name=appsource caps="+ caps +
	    " ! jpegdec ! ffmpegcolorspace ! queue ! jpegenc ! multipartmux ! tcpserversink port=" + lp_str.str();

	// Create the image pipeline
	l_pipeline = gst_parse_launch( l_pipetext.c_str(), &l_error );

	// Seperate errors in case of port clash
	if( l_error == NULL ) {
	    l_source = gst_bin_get_by_name( GST_BIN( l_pipeline ), "appsource" );
	    gst_app_src_set_caps( (GstAppSrc*) l_source, gst_caps_from_string( caps.c_str() ) );
	    gst_element_set_state( l_pipeline, GST_STATE_PLAYING );
	    cout << "Streaming started on port " << start_port << endl;
	    cout << "Watch stream with the command:" << endl;
	    cout << "gst-launch tcpclientsrc host=[ip] port=" << start_port << " ! multipartdemux ! jpegdec ! autovideosink" << endl;
	} else {
	    cout << "A gstreamer error occurred: " << l_error->message << endl;
	}
  }

  lcam->create_ray_map(
      	mirror_diameter,
      	dist_to_mirror,
      	focal_length,
      	outer_radius,
      	camera_height,
        ww, hh);

  while(1){

    while(c.Get()==0) usleep(100);

    c.toIplImage(l);

    if (flip_image) {
    	lcam->flip(l_, NULL);
    }

    lcam->remove(l_, ww, hh, 3, outer_radius, inner_radius);

	int no_of_feats = 0;
	int no_of_feats_horizontal = 0;

	/* display the features */
	if ((show_features) ||
		(unwarp_features) ||
		(edges_filename != "") ||
		(radial_lines_filename != "") ||
		(save_rays != "") ||
		(show_ground_features) ||
		(show_lines)) {

		int inner = (int)inner_radius;
		int outer = (int)outer_radius;
		if (unwarp_image) {
			inner = 0;
			outer = 9999;
		}

		no_of_feats = lcam->get_features_vertical(
			l_,
			inhibition_radius,
			minimum_response,0,0, outer, inner);

		no_of_feats_horizontal = lcam->get_features_horizontal(
			l_,
			inhibition_radius,
			minimum_response,0,0, outer, inner);

		if ((!show_lines) &&
			(!unwarp_features)) {

			/* vertically oriented features */
			int row = 0;
			int feats_remaining = lcam->features_per_row[row];

			for (int f = 0; f < no_of_feats; f++, feats_remaining--) {

				int x = (int)lcam->feature_x[f] / OMNI_SUB_PIXEL;
				int y = 4 + (row * OMNI_VERTICAL_SAMPLING);

				drawing::drawCross(l_, ww, hh, x, y, 2, 0, 255, 0, 0);

				/* move to the next row */
				if (feats_remaining <= 0) {
					row++;
					feats_remaining = lcam->features_per_row[row];
				}
			}

			/* horizontally oriented features */
			int col = 0;
			feats_remaining = lcam->features_per_col[col];

			for (int f = 0; f < no_of_feats_horizontal; f++, feats_remaining--) {

				int y = (int)lcam->feature_y[f] / OMNI_SUB_PIXEL;
				int x = 4 + (col * OMNI_HORIZONTAL_SAMPLING);

				drawing::drawCross(l_, ww, hh, x, y, 2, 0, 255, 0, 0);

				/* move to the next column */
				if (feats_remaining <= 0) {
					col++;
					feats_remaining = lcam->features_per_col[col];
				}
			}
		}

	}

    if (unwarp_image) {
    	lcam->unwarp(l_,ww,hh,3);
    }

    if (unwarp_features) {
    	lcam->unwarp_features(l_,ww,hh,3,no_of_feats,no_of_feats_horizontal);
    }

	if (calibration_image_filename != "") {
		const int calibration_image_width = 640*2;
		const int calibration_image_height = 480*2;
	    IplImage *calib = cvCreateImage(cvSize(calibration_image_width, calibration_image_height), 8, 3);
		unsigned char *calib_=(unsigned char *)calib->imageData;

		lcam->get_calibration_image(calib_, calibration_image_width, calibration_image_height);
		cvSaveImage(calibration_image_filename.c_str(), calib);
	    cvReleaseImage(&calib);
		printf("Camera calibration image saved to %s\n", calibration_image_filename.c_str());
		if ((!save_image) && (radial_lines_filename == "") && (save_rays == "") && (edges_filename == "") && (save_ray_paths_image == "")) break;
	}

	if (edges_filename != "") {
		lcam->save_edges(edges_filename, no_of_feats, no_of_feats_horizontal);
		printf("Edges saved to %s\n", edges_filename.c_str());
		if ((!save_image) && (radial_lines_filename == "") && (save_rays == "") && (save_ray_paths_image == "")) break;
	}

	if (radial_lines_filename != "") {
	    lcam->detect_radial_lines(l_, ww, hh, range_mm, no_of_feats, no_of_feats_horizontal,5);
		lcam->save_radial_lines(radial_lines_filename);
		printf("Radial lines saved to %s\n", radial_lines_filename.c_str());
		if ((!save_image) && (save_rays == "") && (save_ray_paths_image == "")) break;
	}

	if (save_rays != "") {
		lcam->save_rays(save_rays, no_of_feats, no_of_feats_horizontal);
		printf("Rays saved to %s\n", save_rays.c_str());
		if ((!save_image) && (save_ray_paths_image == "")) break;
	}

	if (show_FAST) {
		/* locate corner features in the image */
		corners->update(l_,ww,hh, desired_corner_features,1);
		corners->show(l_,ww,hh,1);
	}

	if (show_ground) {
	    lcam->show_ground_plane(l_,ww,hh,range_mm);
	}

	if (show_ground_features) {
	    lcam->show_ground_plane_features(l_,ww,hh,range_mm,no_of_feats,no_of_feats_horizontal);
	}

	if (show_lines) {
	    lcam->detect_radial_lines(
	        l_, ww, hh, range_mm,
	    	no_of_feats, no_of_feats_horizontal,5);

	    lcam->show_radial_lines(l_,ww,hh,range_mm);
	}

	if (optical_flow) {
    	memcpy((void*)flow_,l_,ww*hh*3);
    	compute_optical_flow(
    		prev, l, flow, 150, 20,
            frame1_1C, frame2_1C, eig_image, temp_image, pyramid1, pyramid2,
            flow_filename);
        memcpy((void*)prev_,l_,ww*hh*3);
        memcpy((void*)l_,flow_,ww*hh*3);
    }

	if (skip_frames == 0) {

		/* save image to file, then quit */
		if (save_image) {
			std::string filename = save_filename + ".jpg";
			cvSaveImage(filename.c_str(), l);
			if (save_ray_paths_image == "") break;
		}
	}

	if (save_ray_paths_image != "") {
	    lcam->show_ray_map_side(
		    l_, ww,hh,(int)((camera_height+focal_length+dist_to_mirror+(mirror_diameter*0.5f))*1.1f),(int)focal_length, (int)camera_height);
	    cvSaveImage(save_ray_paths_image.c_str(), l);
		printf("Ray paths saved to %s\n", save_ray_paths_image.c_str());
	    break;
	}

    /*
     * The streaming bit - seems a bit hacky, someone else can try
     * and convert an IPLImage directly to something GStreamer can handle.
     * My bitbanging abilities just aren't up to the task.
     */
    if (stream) {
	    CvMat* l_buf;
	    l_buf = cvEncodeImage(".jpg", l);

	    l_app_buffer = gst_app_buffer_new( l_buf->data.ptr, l_buf->step, NULL, l_buf->data.ptr );
	    g_signal_emit_by_name( l_source, "push-buffer", l_app_buffer, &ret );
    }

	/* display the image */
	if ((!save_image) && (!headless)) {
	    cvShowImage(image_title.c_str(), l);
	}

    skip_frames--;
    if (skip_frames < 0) skip_frames = 0;

    int wait = cvWaitKey(10) & 255;
    if( wait == 27 ) break;
  }

  /* destroy the image */
  if (!save_image) {
	  cvDestroyWindow(image_title.c_str());
  }
  cvReleaseImage(&l);
  cvReleaseImage(&prev);
  cvReleaseImage(&flow);

  if (frame1_1C != NULL) {
	  cvReleaseImage(&frame1_1C);
	  cvReleaseImage(&frame2_1C);
	  cvReleaseImage(&eig_image);
	  cvReleaseImage(&temp_image);
	  cvReleaseImage(&pyramid1);
	  cvReleaseImage(&pyramid2);
  }

  delete lcam;
  //delete motion;
  delete corners;

  return 0;
}



