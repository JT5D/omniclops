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
#include "unwarp.h"
//#include "motionmodel.h"
#include "fast.h"
#include "libcam.h"

#define VERSION 0.1

using namespace std;

int main(int argc, char* argv[]) {
  int ww = 640;
  int hh = 480;
  int skip_frames = 1;
  bool show_FAST = false;
  bool show_features = false;
  float outer_radius = 115;
  int FOV_degrees = 50;

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
  opt->addUsage( "     --radius               Outer radius as a percentage of the image width");
  opt->addUsage( "     --inner                Inner radius as a percentage of the image width");
  opt->addUsage( "     --calibrate            Calibrate offsets");
  opt->addUsage( "     --features             Show stereo features");
  opt->addUsage( "     --fast                 Show FAST corners");
  opt->addUsage( "     --descriptors          Saves feature descriptor for each FAST corner");
  opt->addUsage( "     --fov                  Field of view in degrees");
  opt->addUsage( " -f  --fps                  Frames per second");
  opt->addUsage( " -s  --skip                 Skip this number of frames");
  opt->addUsage( " -V  --version              Show version number");
  opt->addUsage( "     --save                 Save raw image");
  opt->addUsage( "     --savecalib            Save calibration image");
  opt->addUsage( "     --flip                 Flip the image");
  opt->addUsage( "     --unwarp               Unwarp the image");
  opt->addUsage( "     --stream               Stream output using gstreamer");
  opt->addUsage( "     --headless             Disable video output (for use with --stream)");
  opt->addUsage( "     --help                 Show help");
  opt->addUsage( "" );

  opt->setOption(  "fast" );
  opt->setOption(  "radius" );
  opt->setOption(  "inner" );
  opt->setOption(  "descriptors" );
  opt->setOption(  "save" );
  opt->setOption(  "savecalib" );
  opt->setOption(  "fps", 'f' );
  opt->setOption(  "dev" );
  opt->setOption(  "width", 'w' );
  opt->setOption(  "height", 'h' );
  opt->setOption(  "skip", 's' );
  opt->setOption(  "fov" );
  opt->setFlag(  "help" );
  opt->setFlag(  "flip" );
  opt->setFlag(  "unwarp" );
  opt->setFlag(  "calibrate" );
  opt->setFlag(  "version", 'V' );
  opt->setFlag(  "stream"  );
  opt->setFlag(  "headless"  );
  opt->setFlag(  "features" );

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

  if( opt->getFlag( "help" ) ) {
      opt->printUsage();
      delete opt;
      return(0);
  }

  bool calibrate = false;
  if( opt->getFlag( "calibrate" ) ) {
	  show_FAST = false;
	  calibrate = true;
  }

  if( opt->getFlag( "features" ) ) {
	  show_features = true;
	  show_FAST = false;
  }

  if( opt->getValue( "radius" ) != NULL  ) {
	  outer_radius = atoi(opt->getValue("radius"));
  }

  float inner_radius = 30;
  if( opt->getValue( "inner" ) != NULL  ) {
	  inner_radius = atoi(opt->getValue("inner"));
  }

  int desired_corner_features = 70;
  if( opt->getValue( "fast" ) != NULL  ) {
	  show_FAST = true;
	  show_features = false;
	  desired_corner_features = atoi(opt->getValue("fast"));
	  if (desired_corner_features > 150) desired_corner_features=150;
	  if (desired_corner_features < 50) desired_corner_features=50;
  }

  if( opt->getValue( "fov" ) != NULL  ) {
  	  FOV_degrees = atoi(opt->getValue("fov"));
  }

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
  IplImage *unwarped=cvCreateImage(cvSize(ww, hh), 8, 3);
  unsigned char *l_=(unsigned char *)l->imageData;
  unsigned char *unwarped_=(unsigned char *)unwarped->imageData;

  /* feature detection params */
  int inhibition_radius = 6;
  unsigned int minimum_response = 250;

  omni* lcam = new omni(ww, hh);
  //motionmodel* motion = new motionmodel();
  fast* corners = new fast();

  linefit *lines = new linefit();

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

  float mirror_diameter = 60;
  float dist_to_mirror = 50;
  float focal_length = 3.6f;
  lcam->create_calibration_map(
      	mirror_diameter,
      	dist_to_mirror,
      	focal_length,
      	outer_radius,
        ww, hh);

  while(1){

    while(c.Get()==0) usleep(100);

    c.toIplImage(l);

    if (flip_image) {
    	lcam->flip(l_, NULL);
    }

    lcam->remove(l_, ww, hh, 3, outer_radius, inner_radius);

    if (unwarp_image) {
        unwarp::update(
			ww/2, hh/2,
			(int)(inner_radius*ww/200),
			(int)(outer_radius*ww/200),
			false,
			false,
			false,
			0.28,
			1.5,
			l,
			unwarped);
		memcpy((void*)l_,(void*)unwarped_,ww*hh*3);
    }

	int no_of_feats = 0;
	int no_of_feats_horizontal = 0;

	/* display the features */
	if (show_features) {

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

	if (calibration_image_filename != "") {
		const int calibration_image_width = 640*2;
		const int calibration_image_height = 480*2;
	    IplImage *calib = cvCreateImage(cvSize(calibration_image_width, calibration_image_height), 8, 3);
		unsigned char *calib_=(unsigned char *)calib->imageData;

		lcam->get_calibration_image(calib_, calibration_image_width, calibration_image_height);
		cvSaveImage(calibration_image_filename.c_str(), calib);
	    cvReleaseImage(&calib);
		printf("Camera calibration image saved to %s\n", calibration_image_filename.c_str());
		break;
	}


	if (skip_frames == 0) {

		/* save image to file, then quit */
		if (save_image) {
			std::string filename = save_filename + ".jpg";
			cvSaveImage(filename.c_str(), l);
			break;
		}
	}

	//motion->update(l_,ww,hh);
	//motion->show(l_,ww,hh);

	if (show_FAST) {
		/* locate corner features in the image */
		corners->update(l_,ww,hh, desired_corner_features,1);
		corners->show(l_,ww,hh,1);
	}

	if (calibrate) {
		lcam->calibrate(l_, ww, hh, 3, (int)inner_radius, (int)outer_radius, "E");
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
  cvReleaseImage(&unwarped);

  delete lcam;
  //delete motion;
  delete corners;
  delete lines;

  return 0;
}



