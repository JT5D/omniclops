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

    --grid --mirrors 5 --loadconfig /home/motters/develop/omniclops/docs/mirrors5.txt --dist 150 --elevation 200 --range 500
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
#include "fast.h"
#include "libcam.h"

#include "cppunitlite/TestHarness.h"
#include "cppunitlite/TestResultStdErr.h"
#include "unittests/tests_ray_map.h"
#include "unittests/tests_mirror_lookup.h"
#include "unittests/tests_volumetric.h"
#include "unittests/tests_feature_matching.h"

#define VERSION 0.2
#define MAX_FLOW_MATCHES 150

using namespace std;

struct Match2D {
	unsigned short int x0;
	unsigned short int y0;
	unsigned short int x1;
	unsigned short int y1;
};

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
	std::string flow_filename,
	int& no_of_matches,
	Match2D* matches)
{
	no_of_matches = 0;

	if (frame1_1C == NULL) {
		frame1_1C = cvCreateImage(cvSize(frame1->width,frame1->height), 8, 1);
		frame2_1C = cvCreateImage(cvSize(frame1->width,frame1->height), 8, 1);
		eig_image = cvCreateImage(cvSize(frame1->width,frame1->height), IPL_DEPTH_32F, 1);
		temp_image = cvCreateImage(cvSize(frame1->width,frame1->height), IPL_DEPTH_32F, 1);
		pyramid1 = cvCreateImage(cvSize(frame1->width,frame1->height), IPL_DEPTH_8U, 1);
		pyramid2 = cvCreateImage(cvSize(frame1->width,frame1->height), IPL_DEPTH_8U, 1);
	}

	FILE *file = NULL;
	if (flow_filename != "") {
		file = fopen(flow_filename.c_str(), "wb");
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
         if (hypotenuse < max_magnitude) {  /* ignore big magnitudes which are probably bogus */

        	 /* store the match for later use */
        	 matches[no_of_matches].x0 = (unsigned short)p.x;
        	 matches[no_of_matches].y0 = (unsigned short)p.y;
        	 matches[no_of_matches].x1 = (unsigned short)q.x;
        	 matches[no_of_matches].y1 = (unsigned short)q.y;
        	 no_of_matches++;

        	 if (hypotenuse > 3) { /* don't bother to show features which havn't moved */
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
    }

	if (flow_filename != "") {
		fprintf(file, "%d",no_of_matches);
		fwrite(matches, sizeof(Match2D), no_of_matches, file);

		fclose(file);
	}
}

/*!
 * \brief run all unit tests
 */
void RunUnitTests()
{
    TestResultStdErr result;
    TestRegistry::runAllTests(result);
}

int main(int argc, char* argv[]) {

  RunUnitTests();
  return(0);

  int ww = 640;
  int hh = 480;
  int skip_frames = 1;
  bool show_FAST = false;
  bool show_features = false;
  bool show_ground_features = false;
  bool show_ground = false;
  bool show_lines = false;
  bool unwarp_features = false;
  float outer_radius = 37; //115;
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
  opt->addUsage( "     --mirrors              Number of mirrors");
  opt->addUsage( "     --baseline             Baseline distance between mirrors in mm");
  opt->addUsage( "     --diam                 Diameter of the spherical mirror in millimetres");
  opt->addUsage( "     --dist                 Distance from camera lens to the centre of the mirror in millimetres");
  opt->addUsage( "     --focal                Focal length in millimetres");
  opt->addUsage( "     --elevation            Height of the camera above the ground in millimetres");
  opt->addUsage( "     --radius               Outer radius as a percentage of the image width");
  opt->addUsage( "     --inner                Inner radius as a percentage of the image width");
  opt->addUsage( "     --gridcell             Occupancy grid cell size in millimetres");
  opt->addUsage( "     --griddim              Occupancy grid dimension in cells");
  opt->addUsage( "     --grid                 Show occupancy grid");
  opt->addUsage( "     --features             Show edge features");
  opt->addUsage( "     --lines                Show radial lines");
  opt->addUsage( "     --groundfeatures       Show edge features on the ground plane");
  opt->addUsage( "     --ground               Show ground plane");
  opt->addUsage( "     --overlay              Show overlaid ray intersections with the image plane");
  opt->addUsage( "     --overlayangles        Show overlaid ray angles with the image plane");
  opt->addUsage( "     --range                Maximum range when displaying ground plane");
  opt->addUsage( "     --fast                 Show FAST corners");
  opt->addUsage( "     --descriptors          Saves feature descriptor for each FAST corner");
  opt->addUsage( "     --raypaths             Saves image showing ray paths");
  opt->addUsage( "     --rays                 Saves file containing rays for each observed edge feature");
  opt->addUsage( "     --fov                  Field of view in degrees");
  opt->addUsage( "     --basewidth            Width of the camera base in mm");
  opt->addUsage( "     --baseheight           Height of the camera base in mm");
  opt->addUsage( " -f  --fps                  Frames per second");
  opt->addUsage( " -s  --skip                 Skip this number of frames");
  opt->addUsage( " -V  --version              Show version number");
  opt->addUsage( "     --load                 Load raw image");
  opt->addUsage( "     --save                 Save raw image");
  opt->addUsage( "     --savecalib            Save calibration image");
  opt->addUsage( "     --saveedges            Save edges to file");
  opt->addUsage( "     --saveflow             Save optical flow vectors");
  opt->addUsage( "     --saveradial           Save radial lines to file");
  opt->addUsage( "     --loadconfig           Load a configuration file");
  opt->addUsage( "     --saveconfig           Save a configuration file");
  opt->addUsage( "     --flip                 Flip the image");
  opt->addUsage( "     --unwarp               Unwarp the image");
  opt->addUsage( "     --unwarpfeatures       Unwarp edge features");
  opt->addUsage( "     --flow                 Compute optical flow");
  opt->addUsage( "     --stream               Stream output using gstreamer");
  opt->addUsage( "     --headless             Disable video output (for use with --stream)");
  opt->addUsage( "     --mirrorx0             Relative x coordinate of the first mirror as a percent of image width");
  opt->addUsage( "     --mirrory0             Relative y coordinate of the first mirror as a percent of image height");
  opt->addUsage( "     --mirrorx1             Relative x coordinate of the second mirror as a percent of image width");
  opt->addUsage( "     --mirrory1             Relative y coordinate of the second mirror as a percent of image height");
  opt->addUsage( "     --mirrorx2             Relative x coordinate of the third mirror as a percent of image width");
  opt->addUsage( "     --mirrory2             Relative y coordinate of the third mirror as a percent of image height");
  opt->addUsage( "     --mirrorx3             Relative x coordinate of the fourth mirror as a percent of image width");
  opt->addUsage( "     --mirrory3             Relative y coordinate of the fourth mirror as a percent of image height");
  opt->addUsage( "     --mirrorx4             Relative x coordinate of the fifth mirror as a percent of image width");
  opt->addUsage( "     --mirrory4             Relative y coordinate of the fifth mirror as a percent of image height");
  opt->addUsage( "     --tests                Run unit tests");
  opt->addUsage( "     --help                 Show help");
  opt->addUsage( "" );

  opt->setOption(  "fast" );
  opt->setOption(  "diam" );
  opt->setOption(  "dist" );
  opt->setOption(  "focal" );
  opt->setOption(  "mirrors" );
  opt->setOption(  "baseline" );
  opt->setOption(  "elevation" );
  opt->setOption(  "range" );
  opt->setOption(  "raypaths" );
  opt->setOption(  "rays" );
  opt->setOption(  "radius" );
  opt->setOption(  "inner" );
  opt->setOption(  "descriptors" );
  opt->setOption(  "load" );
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
  opt->setOption(  "mirrorx0" );
  opt->setOption(  "mirrory0" );
  opt->setOption(  "mirrorx1" );
  opt->setOption(  "mirrory1" );
  opt->setOption(  "mirrorx2" );
  opt->setOption(  "mirrory2" );
  opt->setOption(  "mirrorx3" );
  opt->setOption(  "mirrory3" );
  opt->setOption(  "mirrorx4" );
  opt->setOption(  "mirrory4" );
  opt->setOption(  "saveconfig" );
  opt->setOption(  "loadconfig" );
  opt->setOption(  "gridcell" );
  opt->setOption(  "griddim" );
  opt->setOption(  "basewidth" );
  opt->setOption(  "baseheight" );
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
  opt->setFlag(  "overlay" );
  opt->setFlag(  "overlayangles" );
  opt->setFlag(  "grid" );
  opt->setFlag(  "tests" );

  opt->processCommandArgs(argc, argv);

  if( ! opt->hasOptions())
  {
      // print usage if no options
      opt->printUsage();
      delete opt;
      return(0);
  }

  if ( opt->getFlag("tests") ) {
	  RunUnitTests();
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

  std::string load_filename = "";
  if( opt->getValue( "load" ) != NULL  ) {
  	  load_filename = opt->getValue("load");
  	  if (load_filename == "") load_filename = "image_";
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

  bool show_overlay_angles = false;
  bool show_overlay = false;
  bool show_occupancy_grid = false;
  if( opt->getFlag( "grid" ) ) {
	  show_occupancy_grid = true;
	  show_overlay_angles = false;
	  show_overlay = false;
	  show_lines = false;
	  show_ground = false;
	  show_ground_features = false;
	  show_features = false;
	  show_FAST = false;
	  unwarp_features = false;
  }

  if( opt->getFlag( "overlay" ) ) {
	  show_occupancy_grid = false;
	  show_overlay_angles = false;
	  show_overlay = true;
	  show_lines = false;
	  show_ground = false;
	  show_ground_features = false;
	  show_features = false;
	  show_FAST = false;
	  unwarp_features = false;
  }

  if( opt->getFlag( "overlayangles" ) ) {
	  show_occupancy_grid = false;
	  show_overlay_angles = true;
	  show_overlay = false;
	  show_lines = false;
	  show_ground = false;
	  show_ground_features = false;
	  show_features = false;
	  show_FAST = false;
	  unwarp_features = false;
  }

  if( opt->getFlag( "ground" ) ) {
	  show_occupancy_grid = false;
	  show_overlay_angles = false;
	  show_overlay = false;
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
	  show_occupancy_grid = false;
	  show_overlay_angles = false;
	  show_overlay = false;
	  show_lines = false;
	  show_ground = false;
	  show_ground_features = false;
	  show_features = true;
	  show_FAST = false;
	  unwarp_features = false;
  }

  if( opt->getFlag( "unwarpfeatures" ) ) {
	  show_occupancy_grid = false;
	  show_overlay_angles = false;
	  show_overlay = false;
	  show_lines = false;
	  show_ground = false;
	  show_ground_features = false;
	  show_features = false;
	  unwarp_features = true;
	  show_FAST = false;
  }

  if( opt->getFlag( "lines" ) ) {
	  show_occupancy_grid = false;
	  show_overlay_angles = false;
	  show_overlay = false;
	  show_lines = true;
	  show_ground = false;
	  show_ground_features = false;
	  show_features = false;
	  show_FAST = false;
	  unwarp_features = false;
  }

  if( opt->getFlag( "groundfeatures" ) ) {
	  show_occupancy_grid = false;
	  show_overlay_angles = false;
	  show_overlay = false;
	  show_lines = false;
	  show_ground = false;
	  show_ground_features = true;
	  show_features = false;
	  show_FAST = false;
	  unwarp_features = false;
  }

  float baseline = 100;
  if( opt->getValue( "baseline" ) != NULL  ) {
	  baseline = atof(opt->getValue("baseline"));
  }

  int no_of_mirrors = 1;
  float mirror_position[5*2];
  float mirror_position_pixels[5*2];
  mirror_position[0] = 0;
  mirror_position[1] = 0;
  mirror_position_pixels[0] = 50;
  mirror_position_pixels[1] = 50;
  if( opt->getValue( "mirrors" ) != NULL  ) {
	  no_of_mirrors = atoi(opt->getValue("mirrors"));
	  switch(no_of_mirrors)
	  {
		  case 1: {
			  mirror_position[0] = 0;
			  mirror_position[1] = 0;
			  mirror_position_pixels[0] = 50;
			  mirror_position_pixels[1] = 50;
			  break;
		  }
		  case 2: {
			  mirror_position[0] = -baseline/2;
			  mirror_position[1] = 0;
			  mirror_position_pixels[0] = 20;
			  mirror_position_pixels[1] = 50;
			  mirror_position[2] = baseline/2;
			  mirror_position[3] = 0;
			  mirror_position_pixels[2] = 80;
			  mirror_position_pixels[3] = 50;
			  break;
		  }
		  case 3: {
			  mirror_position[0] = -baseline/2;
			  mirror_position[1] = -baseline/2;
			  mirror_position[2] = baseline/2;
			  mirror_position[3] = -baseline/2;
			  mirror_position[4] = 0;
			  mirror_position[5] = baseline/2;

			  mirror_position_pixels[0] = 20;
			  mirror_position_pixels[1] = 20;
			  mirror_position_pixels[2] = 80;
			  mirror_position_pixels[3] = 20;
			  mirror_position_pixels[4] = 50;
			  mirror_position_pixels[5] = 80;
			  break;
		  }
		  case 4: {
			  mirror_position[0] = -baseline/2;
			  mirror_position[1] = -baseline*hh/ww;
			  mirror_position[2] = baseline/2;
			  mirror_position[3] = -baseline*hh/ww;
			  mirror_position[4] = baseline/2;
			  mirror_position[5] = baseline*hh/ww;
			  mirror_position[6] = -baseline/2;
			  mirror_position[7] = baseline*hh/ww;

			  mirror_position_pixels[0] = 20;
			  mirror_position_pixels[1] = 20;

			  mirror_position_pixels[2] = 20;
			  mirror_position_pixels[3] = 80;

			  mirror_position_pixels[4] = 80;
			  mirror_position_pixels[5] = 80;

			  mirror_position_pixels[6] = 80;
			  mirror_position_pixels[7] = 20;
			  break;
		  }
		  case 5: {
			  mirror_position[0] = -baseline/2;
			  mirror_position[1] = -baseline*hh/ww;
			  mirror_position[2] = baseline/2;
			  mirror_position[3] = -baseline*hh/ww;
			  mirror_position[4] = baseline/2;
			  mirror_position[5] = baseline*hh/ww;
			  mirror_position[6] = -baseline/2;
			  mirror_position[7] = baseline*hh/ww;
			  mirror_position[8] = 0;
			  mirror_position[9] = 0;

			  mirror_position_pixels[0] = 20;
			  mirror_position_pixels[1] = 20;

			  mirror_position_pixels[2] = 20;
			  mirror_position_pixels[3] = 80;

			  mirror_position_pixels[4] = 80;
			  mirror_position_pixels[5] = 80;

			  mirror_position_pixels[6] = 80;
			  mirror_position_pixels[7] = 20;

			  mirror_position_pixels[8] = 50;
			  mirror_position_pixels[9] = 50;
			  break;
		  }
	  }
  }

  if( opt->getValue( "mirrorx0" ) != NULL  ) {
	  mirror_position_pixels[0] = atof(opt->getValue("mirrorx0"));
  }
  if( opt->getValue( "mirrory0" ) != NULL  ) {
	  mirror_position_pixels[1] = atof(opt->getValue("mirrory0"));
  }
  if( opt->getValue( "mirrorx1" ) != NULL  ) {
	  mirror_position_pixels[2] = atof(opt->getValue("mirrorx1"));
  }
  if( opt->getValue( "mirrory1" ) != NULL  ) {
	  mirror_position_pixels[3] = atof(opt->getValue("mirrory1"));
  }
  if( opt->getValue( "mirrorx2" ) != NULL  ) {
	  mirror_position_pixels[4] = atof(opt->getValue("mirrorx2"));
  }
  if( opt->getValue( "mirrory2" ) != NULL  ) {
	  mirror_position_pixels[5] = atof(opt->getValue("mirrory2"));
  }
  if( opt->getValue( "mirrorx3" ) != NULL  ) {
	  mirror_position_pixels[6] = atof(opt->getValue("mirrorx3"));
  }
  if( opt->getValue( "mirrory3" ) != NULL  ) {
	  mirror_position_pixels[7] = atof(opt->getValue("mirrory3"));
  }
  if( opt->getValue( "mirrorx4" ) != NULL  ) {
	  mirror_position_pixels[8] = atof(opt->getValue("mirrorx4"));
  }
  if( opt->getValue( "mirrory4" ) != NULL  ) {
	  mirror_position_pixels[9] = atof(opt->getValue("mirrory4"));
  }

  float focal_length = 3.6f;
  if( opt->getValue( "focal" ) != NULL  ) {
	  focal_length = atof(opt->getValue("focal"));
  }

  float camera_height = 0;
  if( opt->getValue( "elevation" ) != NULL  ) {
	  camera_height = atof(opt->getValue("elevation"));
	  //printf("elevation: %f\n", camera_height);
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

  int camera_base_width_mm = 0;
  int camera_base_height_mm = 0;
  if( opt->getValue( "basewidth" ) != NULL  ) {
	  camera_base_width_mm = atoi(opt->getValue("basewidth"));
  }
  if( opt->getValue( "baseheight" ) != NULL  ) {
	  camera_base_height_mm = atoi(opt->getValue("baseheight"));
  }

  int grid_cell_size = 4;
  if( opt->getValue( "gridcell" ) != NULL  ) {
	  grid_cell_size = atoi(opt->getValue("gridcell"));
  }

  int grid_dimension = 128;
  if( opt->getValue( "griddim" ) != NULL  ) {
	  grid_cell_size = atoi(opt->getValue("griddim"));
  }

  float mirror_diameter = 60;
  if( opt->getValue( "diam" ) != NULL  ) {
	  mirror_diameter = atof(opt->getValue("diam"));
  }

  float dist_to_mirror_centre = 50+(mirror_diameter/2);
  if( opt->getValue( "dist" ) != NULL  ) {
	  dist_to_mirror_centre = atof(opt->getValue("dist"));
  }

  if( opt->getValue( "radius" ) != NULL  ) {
	  outer_radius = atoi(opt->getValue("radius"));
  }

  float inner_radius = 30;
  if( opt->getValue( "inner" ) != NULL  ) {
	  inner_radius = atoi(opt->getValue("inner"));
  }

  float range_mm = 200;
  if( opt->getValue( "range" ) != NULL  ) {
	  range_mm = atof(opt->getValue("range"));
  }

  int desired_corner_features = 70;
  if( opt->getValue( "fast" ) != NULL  ) {
	  show_occupancy_grid = false;
	  show_overlay_angles = false;
	  show_overlay = false;
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

  if( opt->getValue( "loadconfig" ) != NULL  ) {
	  string config_filename = opt->getValue("loadconfig");
	  if (omni::load_configuration(
			  config_filename,
			  no_of_mirrors,
			  mirror_position_pixels,
			  mirror_position,
			  focal_length,
			  mirror_diameter,
			  outer_radius,
			  inner_radius,
			  dist_to_mirror_centre,
			  camera_height,
			  baseline,
			  range_mm)) {
		  printf("Configuration loaded from %s\n", config_filename.c_str());
	  }
	  else {
		  printf("Warning: Configuration could not be loaded from %s\n", config_filename.c_str());
	  }
  }

  if( opt->getValue( "saveconfig" ) != NULL  ) {
	  string config_filename = opt->getValue("saveconfig");
	  if (omni::save_configuration(
			  config_filename,
			  no_of_mirrors,
			  mirror_position_pixels,
			  mirror_position,
			  focal_length,
			  mirror_diameter,
			  outer_radius,
			  inner_radius,
			  dist_to_mirror_centre,
			  camera_height,
			  baseline,
			  range_mm)) {
		  printf("Configuration saved to %s\n", config_filename.c_str());
	  }
	  else {
		  printf("Warning: Configuration could not be saved to %s\n", config_filename.c_str());
	  }
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
  IplImage *buf=cvCreateImage(cvSize(ww, hh), 8, 3);
  IplImage *prev=cvCreateImage(cvSize(ww, hh), 8, 3);
  IplImage *flow=cvCreateImage(cvSize(ww, hh), 8, 3);
  IplImage *frame1_1C = NULL;
  IplImage *frame2_1C = NULL;
  IplImage *eig_image = NULL;
  IplImage *temp_image = NULL;
  IplImage *pyramid1 = NULL;
  IplImage *pyramid2 = NULL;
  unsigned char *l_=(unsigned char *)l->imageData;
  unsigned char *buf_=(unsigned char *)buf->imageData;
  unsigned char *prev_=(unsigned char *)prev->imageData;
  unsigned char *flow_=(unsigned char *)flow->imageData;
  int* colour_difference = NULL;
  short* height_field = NULL;
  int* plane_occupancy = NULL;
  unsigned char* height_field_img = NULL;

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


  float cam_height = camera_height;
  if (show_occupancy_grid) cam_height = 0;

  /* create lookup table which maps pixels to 3D rays */
  lcam->create_ray_map(
  	mirror_diameter,
  	dist_to_mirror_centre,
  	focal_length,
  	outer_radius,
  	cam_height,
  	no_of_mirrors,
  	mirror_position,
  	mirror_position_pixels,
    ww,hh);

  Match2D flow_matches[MAX_FLOW_MATCHES];
  unsigned char* img_occlusions = NULL;

  /* load image from file */
  if (load_filename != "") {
	  cvReleaseImage(&prev);
	  prev = cvLoadImage( load_filename.c_str(), CV_LOAD_IMAGE_COLOR );
	  if (prev == NULL) {
		  printf("Couldn't load %s\n", load_filename.c_str());
		  return 0;
	  }
      prev_ = (unsigned char *)prev->imageData;
  }

  while(1){

    while(c.Get()==0) usleep(100);

    c.toIplImage(l);

    if (flip_image) {
    	lcam->flip(l_, NULL);
    }

    if (no_of_mirrors == 1)
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

	if (show_overlay) {
		lcam->show_ray_pixels(l_,ww,hh);
	}

	if (show_overlay_angles) {
		lcam->show_ray_directions(l_,ww,hh);
	}

	if (show_occupancy_grid) {

		int start_plane_height_mm = (int)camera_height;
		int end_plane_height_mm = 0;
		int no_of_planes = 20;
		int patch_size_pixels = 8;
		int min_patch_observations = 3;

		int tx_mm = -range_mm;
		int ty_mm = -range_mm;
		int bx_mm = range_mm;
		int by_mm = range_mm;

		int mirror_index = -1;
		int min_r_mm = 0;
		int max_r_mm = 60;

		if (colour_difference == NULL) {
			colour_difference = new int[ww*hh*2];
			height_field = new short[ww*hh];
			height_field_img = new unsigned char[ww*hh*3];
			plane_occupancy = new int[no_of_planes];
		}
		//memcpy((void*)buf_,(void*)l_,ww*hh*3);

		/*
		omni::project(
			buf_,
			0,
			focal_length,
			dist_to_mirror_centre,
			camera_height,
		  	ww,hh,
			tx_mm, ty_mm,
			bx_mm, by_mm,
			lcam->ray_map,
			lcam->mirror_map,
			lcam->mirror_lookup,
			mirror_index,
			min_r_mm,
			max_r_mm,
			l_,
			ww, hh,
			colour_difference);


		int grid_centre_x_mm = 0;
		int grid_centre_y_mm = 0;
		int grid_centre_z_mm = 0;
		vector<short> occupied_voxels;
		int min_correlation = 0;
		if (img_occlusions == NULL) {
			img_occlusions = new unsigned char[ww*hh];
		}

		omni::voxel_paint(
		    lcam->ray_map,
		    dist_to_mirror_centre,
			mirror_diameter,
			no_of_mirrors,
			lcam->mirror_map,
			l_,
			img_occlusions,
			ww,hh,
			grid_dimension,
			grid_dimension,
			grid_dimension,
			grid_cell_size,
			grid_centre_x_mm,
			grid_centre_y_mm,
			grid_centre_z_mm,
			min_correlation,
			occupied_voxels);

    	int voxel_radius_pixels = 2;
    	int view_type = 3;

		omni::show_voxels(l_,ww,hh, occupied_voxels, voxel_radius_pixels, view_type);
		*/

		omni::reconstruct_volume(
			l_,
			start_plane_height_mm,
			end_plane_height_mm,
			no_of_planes,
			focal_length,
			(int)dist_to_mirror_centre,
			(int)camera_height,
			ww, hh,
			tx_mm, ty_mm,
			bx_mm, by_mm,
			camera_base_width_mm,
			camera_base_height_mm,
			lcam->ray_map,
			lcam->mirror_map,
			lcam->mirror_lookup,
			buf_,
			ww, hh,
			colour_difference,
			height_field,
			height_field_img,
			patch_size_pixels,
			min_patch_observations,
			plane_occupancy);
/*
	    omni::show_height_field(
	    	l_,
	    	ww,hh,
	    	(int)dist_to_mirror_centre,
	    	height_field,
	    	height_field_img,
	    	ww,hh,3);
*/
	    omni::show_plane_occupancy(l_,ww,hh,no_of_planes,plane_occupancy);

	}

	if (optical_flow) {
	    int no_of_flow_matches = 0;

    	memcpy((void*)flow_,l_,ww*hh*3);
    	compute_optical_flow(
    		prev, l, flow, MAX_FLOW_MATCHES, 20,
            frame1_1C, frame2_1C, eig_image, temp_image, pyramid1, pyramid2,
            flow_filename,no_of_flow_matches,flow_matches);
        memcpy((void*)prev_,l_,ww*hh*3);
        memcpy((void*)l_,flow_,ww*hh*3);
    }

	if (skip_frames == 0) {

		/* save image to file, then quit */
		if (save_image) {
			std::string filename = save_filename + ".jpg";
			if (!optical_flow) {
			    cvSaveImage(filename.c_str(), l);
			}
			else {
				cvSaveImage(filename.c_str(), prev);
			}
			if (save_ray_paths_image == "") break;
		}
	}

	if (save_ray_paths_image != "") {
	    lcam->show_ray_map_side(
		    l_, ww,hh,(int)((camera_height+focal_length+dist_to_mirror_centre)*1.1f),(int)focal_length, (int)camera_height, true);

	    //float max_radius_mm = 130;
	    //lcam->show_ray_map_above(l_, ww, hh, max_radius_mm);
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
  cvReleaseImage(&buf);
  cvReleaseImage(&prev);
  cvReleaseImage(&flow);

  if (colour_difference != NULL) {
	  delete[] colour_difference;
  }
  if (height_field != NULL) {
	  delete[] height_field;
	  delete[] height_field_img;
  }
  if (plane_occupancy != NULL) {
	  delete[] plane_occupancy;
  }
  if (img_occlusions == NULL) {
	  delete[] img_occlusions;
  }
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



