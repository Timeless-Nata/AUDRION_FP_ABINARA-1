// Camera model / ray geometry shared by perception and mission control.
#pragma once
#include <cmath>
#include "autonomous_manipulator/robot_model.hpp"

namespace am {

struct CameraIntrinsics {
  double fx = 1361.0, fy = 1361.0, cx = 320.0, cy = 240.0;
  // MuJoCo: fovy = 20 deg, 640x480 => fy = (H/2)/tan(fovy/2)
  static CameraIntrinsics fromFovy(double fovy_deg, int width, int height) {
    CameraIntrinsics k;
    k.fy = (height / 2.0) / std::tan(fovy_deg * M_PI / 360.0);
    k.fx = k.fy;
    k.cx = width / 2.0;
    k.cy = height / 2.0;
    return k;
  }
};

// Pixel (u right, v down) -> world ray. cam is the MuJoCo camera frame (x right, y up, looking along -z).
inline void pixelRay(const CameraIntrinsics& K, const Pose& cam, double u, double v, Vec3& origin, Vec3& dir) {
  Vec3 dc{(u - K.cx) / K.fx, -(v - K.cy) / K.fy, -1.0};
  origin = cam.p;
  dir = normalized(cam.R * dc);
}

// Intersect a ray with the horizontal plane z = zp. Returns false if the ray is (almost) parallel or points away.
inline bool rayPlane(const Vec3& o, const Vec3& d, double zp, Vec3& hit, double min_down = 0.05) {
  if (d.z > -min_down) return false;
  double t = (zp - o.z) / d.z;
  if (t <= 0) return false;
  hit = o + t * d;
  return true;
}

}  // namespace am
