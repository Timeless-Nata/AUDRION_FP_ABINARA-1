// Perception: HSV colour segmentation of the simulated camera image + ground-plane back-projection.
// Position model: the camera pose comes from forward kinematics of the measured joint angles (camera mounting transform
// taken from the MJCF); the pixel centroid of the dumbbell's lit top face defines a ray; the ray is intersected with the
// known horizontal plane z = plane_z (top face of an upright dumbbell). No depth camera and no simulator ground truth.
#include <cmath>
#include <map>
#include <mutex>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>

#include "autonomous_manipulator/ObjectDetectionArray.h"
#include "autonomous_manipulator/geometry_utils.hpp"

using namespace am;

class PerceptionNode {
 public:
  PerceptionNode() : nh_(), cfg_("/am/perception") {
    cfg_.param("plane_z", plane_z_, 0.074);
    cfg_.param("roi_u_max", roi_u_max_, 360);
    cfg_.param("min_area_px", min_area_, 120);
    cfg_.param("min_fill_ratio", min_fill_, 0.30);
    cfg_.param("top_face_v_ratio", top_v_ratio_, 0.85);
    cfg_.param("min_radius", min_r_, 0.15);
    cfg_.param("max_radius", max_r_, 0.50);
    cfg_.param("max_joint_speed", max_speed_, 0.02);
    cfg_.param("max_state_skew", max_skew_, 0.05);
    std::vector<std::string> names;
    cfg_.getParam("color_names", names);
    for (const auto& n : names) {
      std::vector<int> lo, hi;
      if (cfg_.getParam("colors/" + n + "/lower", lo) && cfg_.getParam("colors/" + n + "/upper", hi) && lo.size() == 3 &&
          hi.size() == 3)
        colors_[n] = {cv::Scalar(lo[0], lo[1], lo[2]), cv::Scalar(hi[0], hi[1], hi[2])};
      else
        ROS_ERROR("perception: bad HSV config for colour '%s'", n.c_str());
    }
    pub_ = nh_.advertise<autonomous_manipulator::ObjectDetectionArray>("/perception/detections", 5);
    dbg_ = nh_.advertise<sensor_msgs::Image>("/perception/debug_image", 2);
    sub_js_ = nh_.subscribe("/joint_states", 20, &PerceptionNode::onJoints, this);
    sub_ci_ = nh_.subscribe("/color_cam/camera_info", 1, &PerceptionNode::onInfo, this);
    sub_im_ = nh_.subscribe("/color_cam/image_raw", 2, &PerceptionNode::onImage, this);
    ROS_INFO("perception: ready (%zu colours)", colors_.size());
  }

 private:
  struct Range {
    cv::Scalar lo, hi;
  };

  void onInfo(const sensor_msgs::CameraInfoConstPtr& m) {
    std::lock_guard<std::mutex> l(mtx_);
    K_.fx = m->K[0]; K_.cx = m->K[2]; K_.fy = m->K[4]; K_.cy = m->K[5];
    have_info_ = true;
  }
  void onJoints(const sensor_msgs::JointStateConstPtr& m) {
    if (m->position.size() < 4 || m->velocity.size() < 4) return;
    std::lock_guard<std::mutex> l(mtx_);
    for (int i = 0; i < 4; ++i) { q_[i] = m->position[i]; qd_[i] = m->velocity[i]; }
    js_stamp_ = m->header.stamp;
    have_js_ = true;
  }

  void onImage(const sensor_msgs::ImageConstPtr& im) {
    JointVec q, qd;
    CameraIntrinsics K;
    {
      std::lock_guard<std::mutex> l(mtx_);
      if (!have_info_ || !have_js_) return;
      if (std::fabs((im->header.stamp - js_stamp_).toSec()) > max_skew_) return;
      q = q_; qd = qd_; K = K_;
    }
    for (int i = 0; i < 4; ++i)
      if (std::fabs(qd[i]) > max_speed_) return;  // arm moving: camera pose not trustworthy
    if (im->encoding != "rgb8") { ROS_WARN_THROTTLE(5, "perception: expected rgb8"); return; }

    cv::Mat rgb(im->height, im->width, CV_8UC3, const_cast<uint8_t*>(im->data.data()), im->step);
    cv::Mat hsv;
    cv::cvtColor(rgb, hsv, cv::COLOR_RGB2HSV);
    cv::Mat vis = rgb.clone();
    cv::Mat roi = cv::Mat::zeros(hsv.size(), CV_8UC1);
    roi(cv::Rect(0, 0, std::min<int>(roi_u_max_, hsv.cols), hsv.rows)).setTo(255);
    const Pose cam = cameraPose(q);

    autonomous_manipulator::ObjectDetectionArray out;
    out.header.stamp = im->header.stamp;
    out.header.frame_id = "world";
    out.plane_z = plane_z_;
    for (const auto& kv : colors_) {
      cv::Mat mask;
      cv::inRange(hsv, kv.second.lo, kv.second.hi, mask);
      cv::bitwise_and(mask, roi, mask);
      cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
      cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
      cv::Mat labels, stats, cents;
      int n = cv::connectedComponentsWithStats(mask, labels, stats, cents);
      for (int i = 1; i < n; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        int bw = stats.at<int>(i, cv::CC_STAT_WIDTH), bh = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        if (area < min_area_) continue;
        if (static_cast<double>(area) / (bw * bh) < min_fill_) continue;
        // top face = brightest part of the blob (spot light above the scene)
        cv::Rect box(stats.at<int>(i, cv::CC_STAT_LEFT), stats.at<int>(i, cv::CC_STAT_TOP), bw, bh);
        double vmax = 0;
        for (int y = box.y; y < box.y + bh; ++y)
          for (int x = box.x; x < box.x + bw; ++x)
            if (labels.at<int>(y, x) == i) vmax = std::max<double>(vmax, hsv.at<cv::Vec3b>(y, x)[2]);
        double su = 0, sv = 0, cnt = 0;
        for (int y = box.y; y < box.y + bh; ++y)
          for (int x = box.x; x < box.x + bw; ++x)
            if (labels.at<int>(y, x) == i && hsv.at<cv::Vec3b>(y, x)[2] >= top_v_ratio_ * vmax) {
              su += x; sv += y; cnt += 1;
            }
        double u, v;
        if (cnt >= 30) { u = su / cnt; v = sv / cnt; }
        else { u = cents.at<double>(i, 0); v = cents.at<double>(i, 1); }
        Vec3 o, d, hit;
        pixelRay(K, cam, u, v, o, d);
        if (!rayPlane(o, d, plane_z_, hit)) continue;
        double r = std::hypot(hit.x, hit.y);
        if (r < min_r_ || r > max_r_) continue;  // not an object in the workspace (e.g. robot's own parts)
        autonomous_manipulator::ObjectDetection det;
        det.color = kv.first;
        det.u = u; det.v = v; det.area_px = area;
        det.position.x = hit.x; det.position.y = hit.y; det.position.z = hit.z;
        det.ray_origin.x = o.x; det.ray_origin.y = o.y; det.ray_origin.z = o.z;
        det.ray_direction.x = d.x; det.ray_direction.y = d.y; det.ray_direction.z = d.z;
        out.detections.push_back(det);
        cv::rectangle(vis, box, cv::Scalar(255, 0, 255), 2);
        cv::putText(vis, kv.first, box.tl() + cv::Point(0, -4), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
        cv::circle(vis, cv::Point(static_cast<int>(u), static_cast<int>(v)), 3, cv::Scalar(255, 0, 255), -1);
      }
    }
    pub_.publish(out);
    if (dbg_.getNumSubscribers() > 0) {
      sensor_msgs::Image msg = *im;
      msg.data.assign(vis.data, vis.data + vis.total() * vis.elemSize());
      dbg_.publish(msg);
    }
  }

  ros::NodeHandle nh_, cfg_;
  ros::Publisher pub_, dbg_;
  ros::Subscriber sub_js_, sub_ci_, sub_im_;
  std::map<std::string, Range> colors_;
  std::mutex mtx_;
  CameraIntrinsics K_;
  JointVec q_{}, qd_{};
  ros::Time js_stamp_;
  bool have_info_ = false, have_js_ = false;
  double plane_z_, min_fill_, top_v_ratio_, min_r_, max_r_, max_speed_, max_skew_;
  int roi_u_max_, min_area_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "perception");
  PerceptionNode node;
  ros::spin();
  return 0;
}
