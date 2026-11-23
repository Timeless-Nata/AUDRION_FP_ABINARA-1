// color_classifier.hpp - decides WHICH single block colour a camera frame shows (pure C++17, no OpenCV / ROS).
//
// The camera does not locate anything: position comes from the HC-SR04 range + the arm pose. The frame is only
// used to validate one colour: count the pixels of every colour class inside a region of interest, take the dominant
// class, and report "ambiguous" when a second class is too large (the arm should then move to another view).
// HSV uses the OpenCV scale: H 0..180, S 0..255, V 0..255.
#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace arm {

struct HsvRange {
  std::string name;
  int h_lo = 0, h_hi = 180;  // h_lo > h_hi means the range wraps around 180 (reds)
  int s_lo = 0, s_hi = 255;
  int v_lo = 0, v_hi = 255;
};

struct ColorConfig {
  std::vector<HsvRange> classes;     // a pixel belongs to the FIRST class that matches (order = priority)
  double roi_x0 = 0.0, roi_y0 = 0.0, roi_x1 = 1.0, roi_y1 = 1.0;  // region of interest, fractions of the image
  double min_fraction = 0.03;        // dominant class must cover at least this share of the ROI
  double max_second_ratio = 0.15;    // second / dominant pixel count above this = ambiguous

  // Values measured on MuJoCo camera frames of scene.xml (orange / black / beige blocks, white-grey floor).
  // SIMULATION ONLY: the real webcam needs its own calibration (config/camera_config.yaml, hardware section).
  static ColorConfig simulationDefaults();
};

enum class ColorStatus {
  Unique,     // exactly one colour dominates
  Ambiguous,  // a second colour is too large: move the camera and look again
  NoObject    // no colour covers min_fraction of the ROI
};

struct ColorResult {
  ColorStatus status = ColorStatus::NoObject;
  int dominant = -1;                 // index into ColorConfig::classes, -1 if none
  int second = -1;
  std::vector<double> fraction;      // per class, share of the ROI pixels
  double second_ratio = 0.0;         // second / dominant
  double second_cx = 0.5;            // horizontal centroid of the second class, 0 = left edge of the image, 1 = right edge
  double confidence = 0.0;           // 0..1: (1 - second_ratio / max_second_ratio) * min(1, dominant share / (3 * min_fraction))
  std::string name;                  // class name of `dominant` ("" if none)
};

// OpenCV-scale HSV of an 8-bit RGB pixel.
void rgbToHsv(uint8_t r, uint8_t g, uint8_t b, int& h, int& s, int& v);

// rgb: width*height*3 bytes, row-major, no padding (convert BGR / strided buffers before calling).
ColorResult classifyColor(const uint8_t* rgb, int width, int height, const ColorConfig& cfg);

}  // namespace arm
