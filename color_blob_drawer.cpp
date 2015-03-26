/*!
  \file        color_blob_drawer.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/9/30

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

A camera skeleton, based on:
http://docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html
 */
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "timer.h"
#include "disjoint_sets2.h"

namespace image_utils {
inline void HSVfilter(const cv::Mat3b & srcHSV,
                      const int outValue, cv::Mat1b & result,
                      const int Hmin, const int Hmax,
                      const int Smin, const int Smax,
                      const int Vmin, const int Vmax,
                      cv::Mat1b & Hbuffer, cv::Mat1b & Sbuffer, cv::Mat1b & Vbuffer) {
  // init the images if needed
  Hbuffer.create(srcHSV.size());
  Sbuffer.create(srcHSV.size());
  Vbuffer.create(srcHSV.size());
  result.create(srcHSV.size());

  // split
  std::vector< cv::Mat1b > layers;
  layers.push_back(Hbuffer);
  layers.push_back(Sbuffer);
  layers.push_back(Vbuffer);
  cv::split(srcHSV, layers);

  // filter each channel
  cv::threshold(Hbuffer, Hbuffer, Hmax, 255, cv::THRESH_TOZERO_INV);
  cv::threshold(Hbuffer, Hbuffer, Hmin, outValue, cv::THRESH_BINARY);
  cv::threshold(Sbuffer, Sbuffer, Smax, 255, cv::THRESH_TOZERO_INV);
  cv::threshold(Sbuffer, Sbuffer, Smin, outValue, cv::THRESH_BINARY);
  cv::threshold(Vbuffer, Vbuffer, Vmax, 255, cv::THRESH_TOZERO_INV);
  cv::threshold(Vbuffer, Vbuffer, Vmin, outValue, cv::THRESH_BINARY);

  // combine
  cv::min( (cv::Mat) Hbuffer, Sbuffer, result);
  cv::min( (cv::Mat) result, Vbuffer, result);
}
} // end namespace image_utils

namespace geometry_utils {
/*!
 * \brief   computes the barycenter of a std::vector of points
     *
 * \param   src the std::vector
 * \return  the barycenter
 */
template<class Point2>
static inline Point2 barycenter(const std::vector<Point2> & src) {
  if (src.empty())
    return Point2();
  double x = 0, y = 0;
  unsigned int npts = src.size();
  for (unsigned int pt_idx = 0; pt_idx < npts; ++pt_idx) {
    x += src[pt_idx].x;
    y += src[pt_idx].y;
  } // end loop pt_idx
  return Point2(x / npts, y / npts);
}
} // end namespace geometry_utils

class CPaintRecognizer {
public:
  //! the width of the tracking window around the center, in percents
  static const int TRACKING_WINDOW_WIDTH_PERCENT = 40;
  //! the height of the tracking window around the center, in percents
  static const int TRACKING_WINDOW_HEIGHT_PERCENT = 40;
  //! the minimum biggest size of the biggest component for declaring the search OK, in pixels
  static const unsigned int MIN_BIGGEST_SIZE = 250;
  //! the time to stop a drawing, in seconds
  static const int TIME_THRES_DRAWING_STOP = 2000;
  //! the maximum error allowed to call the function
  static const int RECOGNITION_ERROR_THRES = 3;

  static const int DRAWING_IMAGE_LITTLE_WIDTH = 640;
  static const int DRAWING_IMAGE_LITTLE_HEIGHT = 480;


  /* constructor */
  CPaintRecognizer() {
    PENCIL_DIAMETER = 5;
    track_window_.width = -1; // marker to know we haven't init
    is_color_blob_found_last_frame = is_color_blob_found = false;
    is_drawing = false;
    // create the images
    drawing_image.create(DRAWING_IMAGE_LITTLE_HEIGHT,
                         DRAWING_IMAGE_LITTLE_WIDTH);
    WIN1NAME = "filter";
    cv::namedWindow(WIN1NAME);
    hue_min = 60;
    hue_max = 100;
    saturation_min = 50;
    saturation_max = 255;
    value_min = 140;
    value_max= 255;
    cv::createTrackbar("HUE_MIN", WIN1NAME, &hue_min, 255);
    cv::createTrackbar("HUE_MAX", WIN1NAME, &hue_max, 255);
    cv::createTrackbar("SATURATION_MIN", WIN1NAME, &saturation_min, 255);
    cv::createTrackbar("SATURATION_MAX", WIN1NAME, &saturation_max, 255);
    cv::createTrackbar("VALUE_MIN", WIN1NAME, &value_min, 255);
    cv::createTrackbar("VALUE_MAX", WIN1NAME, &value_max, 255);
  }

  /* destructor */
  ~CPaintRecognizer() {}

  ////////////////////////////////////////////////////////////////////////////////

  void process_rgb(const cv::Mat3b & rgb) {
    printf("process_rgb(const cv::Mat3b & rgb)\n");
    timer.reset();

    if (track_window_.width == -1)
      track_color_blob_init(rgb);
    find_color_blob(rgb);

    drawer();
    display(rgb);
    timer.printTime("after find_color_blob()");
  }

private:
  ////////////////////////////////////////////////////////////////////////////////

  void display(const cv::Mat3b & rgb) {
    cv::cvtColor(final_mask_, frameOut, CV_GRAY2RGB);

    // draw the pencil
    if (is_color_blob_found == true) {
      cv::circle(frameOut, color_blob_center, 5, CV_RGB(0, 0, 0), -1);
      cv::circle(frameOut, color_blob_center, 2, CV_RGB(0, 255, 0), -1);
    }
    // pencil not found => make a red cross
    else {
      cv::line(frameOut, cv::Point(0, 0), cv::Point(rgb.cols, rgb.rows),
               CV_RGB(255, 0, 0), 15);
      cv::line(frameOut, cv::Point(0, rgb.rows), cv::Point(rgb.cols, 0),
               CV_RGB(255, 0, 0), 15);
    }

    // draw the search box
    cv::rectangle(frameOut, track_window_, CV_RGB(0, 0, 255), 2);
    // search box too close from the border => make a red rectangle
    if (is_track_window_close_from_borders)
      cv::rectangle(frameOut,
                    cv::Point(0, 0),
                    cv::Point(rgb.cols, rgb.rows),
                    CV_RGB(255, 0, 0), 15);

    /* display in the 2 windows */
    cv::imshow("rgb", rgb);
    cv::imshow("drawing_image", drawing_image);
    cv::imshow(WIN1NAME, frameOut);

    char c = (cv::waitKey(5) & 255);
    if (c == ' ') {
      //startStopDrawer();
    }
    if (c == 27)
      exit(0); // exit key pressed - DON'T DELETE THIS LINE OR THERE WILL BE NO WINDOW DISPLAYED
  }

  ////////////////////////////////////////////////////////////////////////////////

  void find_color_blob(const cv::Mat3b & rgb) {
    printf("find_color_blob()\n");

    /* get the monochrome version of frame */
    thres_image(rgb);

    is_color_blob_found_last_frame = is_color_blob_found;
    if (is_color_blob_found_last_frame == true) // found in the last image
      track_color_blob(rgb);

    if (is_color_blob_found_last_frame == false)
      find_color_blob_no_tracking(rgb);
  }

  ////////////////////////////////////////////////////////////////////////////////

  void thres_image(const cv::Mat3b & rgb) {
    printf( "thres_image()\n" );

    cv::cvtColor(rgb, hsv_, CV_BGR2HSV);
    image_utils::HSVfilter(hsv_, 255, final_mask_,
                           hue_min, hue_max,
                           saturation_min, saturation_max,
                           value_min, value_max,
                           hue_, saturation_, value_);
    //printf("final_mask_:%s", image_utils::infosImage(final_mask_).c_str());
  }

  ////////////////////////////////////////////////////////////////////////////////

  void find_color_blob_no_tracking(const cv::Mat3b & rgb) {
    printf("find_color_blob_no_tracking()\n");

    /* find center */
    //image_utils::biggestComponent_vector(final_mask_, biggest);
    disjoint_set.process_image(final_mask_);
    disjoint_set.biggestComponent_vector(final_mask_.cols, biggest_comp);
    printf("find_color_blob_no_tracking(): biggest.size():%i\n", (int) biggest_comp.size());

    // pencil found => move it
    if (biggest_comp.size() < MIN_BIGGEST_SIZE) {
      printf("find_color_blob_no_tracking(): biggest.size() not sufficient, is_color_blob_found = 0\n");
      is_color_blob_found = false;
      return;
    }

    color_blob_center = geometry_utils::barycenter(biggest_comp);
    move_track_window(rgb, color_blob_center);
    is_color_blob_found = true;
  }

  ////////////////////////////////////////////////////////////////////////////////

  void track_color_blob_init(const cv::Mat3b & rgb) {
    printf("track_color_blob_init()\n");

    // init the track window
    track_window_.width = TRACKING_WINDOW_WIDTH_PERCENT * rgb.cols / 100;
    track_window_.height = TRACKING_WINDOW_HEIGHT_PERCENT * rgb.rows / 100;
    move_track_window(rgb, cv::Point(rgb.cols / 2, rgb.rows / 2));

    // init the tracking
    is_color_blob_found = false;
  }

  ////////////////////////////////////////////////////////////////////////////////

  void track_color_blob(const cv::Mat3b & rgb) {
    printf("track_color_blob()\n");

    /* find center */
    final_mask_(track_window_).copyTo(final_mask_roi_);
    disjoint_set.process_image(final_mask_roi_);
    disjoint_set.biggestComponent_vector(final_mask_roi_.cols, biggest_comp);
    printf("track_color_blob(): biggest.size():%i\n", (int) biggest_comp.size());

    // pencil found => move it
    if (biggest_comp.size() < MIN_BIGGEST_SIZE) {
      printf("track_color_blob(): biggest.size() not sufficient, is_tracking_successful = 0\n");
      is_color_blob_found = false;
      return;
    }

    color_blob_center = geometry_utils::barycenter(biggest_comp);
    // add the shift of the ROI
    color_blob_center.x += track_window_.x;
    color_blob_center.y += track_window_.y;

    move_track_window(rgb, color_blob_center);
    is_color_blob_found = true;
  }

  ////////////////////////////////////////////////////////////////////////////////

  void move_track_window(const cv::Mat3b & rgb,
                         cv::Point p) {
    is_track_window_close_from_borders = 0;
    // look if we go over, with the x coordinate
    if (p.x - track_window_.width / 2 <= 0) {
      is_track_window_close_from_borders = 1;
      track_window_.x = 0;
    } else if (p.x + track_window_.width / 2 >= rgb.cols) {
      is_track_window_close_from_borders = 1;
      track_window_.x = rgb.cols - track_window_.width;
    } else
      track_window_.x = p.x - track_window_.width / 2;

    // look if we go over, with the y coordinate
    if (p.y - track_window_.height / 2 < 0) {
      is_track_window_close_from_borders = 1;
      track_window_.y = 0;
    } else if (p.y + track_window_.height / 2 > rgb.rows) {
      is_track_window_close_from_borders = 1;
      track_window_.y = rgb.rows - track_window_.height;
    } else
      track_window_.y = p.y - track_window_.height / 2;
  }

  //////////////////////////////////////////////////////////////////////////////

  void drawer() {
    printf("drawer()\n");

    // see if the drawing has just stopped
    if (is_color_blob_found == false) {
      if (is_color_blob_found_last_frame == false) {
        // the pencil was not found in the last image
        printf("The pencil has disappeard since : %g ms\n",
               pencil_last_appearance_timer.time());
        // if drawing and the pencil lost for too much time => stop
        if (is_drawing && timer.time() > TIME_THRES_DRAWING_STOP)
          stopDrawer();
      }
      else { // is_color_blob_found_last_frame == true
        printf("The pencil has just disappeard !\n");
        pencil_last_appearance_timer.reset();
      } // end is_color_blob_found_last_frame
    } // end if is_color_blob_found == false

    else { // is_color_blob_found == true
      move_pencil(color_blob_center.x, color_blob_center.y);
      if (is_drawing) {
        printf("The drawing goes on.\n");
        add_point_to_drawing(pencil_noReverse);
      }
      else // start a drawing
        startDrawer();
    } // end if is_color_blob_found == true

#if PZ_FIND_VIDEOS
    writer_frameOut.write(frameOut);
    cv::Mat3b illus(rgb.size());
    cvCopy(frame, illus);
    for (int x = 0; x < illus->width; ++x) {
      for (int y = 0; y < illus->height; ++y) {
        if (CV_IMAGE_ELEM( drawing_image, uchar, y, x ) != 0) {
          image_utils::drawPoint( illus, cv::Point(illus->width - x, y), CV_RGB(0,255,0) );
        }
      }
    }
    writer_frame_drawing.write(illus );
#endif

    if (is_drawing) {
      cv::circle(drawing_image,
                 cv::Point(drawing_image.cols - 35, drawing_image.rows - 35),
                 25, CV_RGB(255, 0, 0), 2);
    } else {
      cv::rectangle(drawing_image,
                    cv::Point(drawing_image.cols - 50, drawing_image.rows - 50),
                    cv::Point(drawing_image.cols - 10, drawing_image.rows - 10),
                    CV_RGB(0, 255, 0), 2);
      cv::rectangle(drawing_image,
                    cv::Point(drawing_image.cols - 30, drawing_image.rows - 50),
                    cv::Point(drawing_image.cols - 20, drawing_image.rows - 10),
                    CV_RGB(0, 255, 0), 2);
    }

    printf("end of drawer()\n");
  }

  /////
  ///// drawing
  /////
  //! to launch a new drawing_image, or to finish one (analyse the drawing_image)
  void startDrawer() {
    printf("startDrawer()\n");
    current_drawn_pencil.x = -1;
    drawing_image.setTo(0); // clear the image
    is_drawing = true;
  }

  void stopDrawer() {
    printf("stopDrawer()\n");
    drawing_image.setTo(0); // clear the image
    is_drawing = false;
  }

  void add_point_to_drawing(cv::Point p) {
    last_drawn_pencil = current_drawn_pencil;
    current_drawn_pencil = reverse_coord(p);
    if (last_drawn_pencil.x > -1)
      cv::line(drawing_image, last_drawn_pencil, current_drawn_pencil,
               cvScalarAll(255), PENCIL_DIAMETER);
  }

  //! move the pencil to a given point
  inline void move_pencil(int x, int y) {
    printf("move_pencil(%i, %i)\n", x, y);
    pencil_noReverse.x = x;
    pencil_noReverse.y = y;
  }

  //! get the coordinates of the pencil
  inline cv::Point reverse_coord(cv::Point p) {
    cv::Point pencil;
    pencil.x = drawing_image.cols - p.x;
    pencil.y = p.y;
    return pencil;
  }

  cv::Point color_blob_center;
  bool is_color_blob_found;
  bool is_color_blob_found_last_frame;
  //////////////////////////////////////////////////////////////////////////////
  //! parameters of the filter
  int hue_min, hue_max, saturation_min, saturation_max, value_min, value_max;
  cv::Mat3b hsv_;
  cv::Mat1b hue_, saturation_, value_;
  cv::Mat1b final_mask_;
  cv::Mat3b frameOut;
  cv::Rect track_window_;
  cv::Mat1b final_mask_roi_;
  bool is_track_window_close_from_borders;
  DisjointSets2 disjoint_set;
  std::vector<cv::Point> biggest_comp;
  std::string WIN1NAME;
  //////////////////////////////////////////////////////////////////////////////
  int PENCIL_DIAMETER;
  cv::Point last_drawn_pencil;
  cv::Point current_drawn_pencil;
  cv::Mat1b drawing_image; // the current drawing_image
  bool is_drawing;
  cv::Point pencil_noReverse;
  Timer timer, pencil_last_appearance_timer;
}; // end class CPaintRecognizer

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main(int, char**) {
  cv::VideoCapture cap(0); // open the default camera
  int w = 640, h = 480;
  //int w = 1600, h = 1200;
  printf("w:%i, h:%i\n", w, h);
  assert(cap.isOpened());  // check if we succeeded
  cap.set(CV_CAP_PROP_FRAME_WIDTH, w);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, h);
  CPaintRecognizer reco;

  for(;;) {
    cv::Mat frame;
    cap >> frame; // get a new frame from camera
    reco.process_rgb(frame);
  }
  // the camera will be deinitialized automatically in VideoCapture destructor
  return 0;
}
