// Minimal 3D math (no Eigen dependency) used by kinematics, perception and planning.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace am {

struct Vec3 {
  double x = 0, y = 0, z = 0;
};
inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(double s, const Vec3& a) { return {s * a.x, s * a.y, s * a.z}; }
inline double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline double norm(const Vec3& a) { return std::sqrt(dot(a, a)); }
inline Vec3 normalized(const Vec3& a) {
  double n = norm(a);
  return n > 1e-12 ? (1.0 / n) * a : Vec3{};
}

struct Mat3 {
  double m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  Vec3 col(int j) const { return {m[0][j], m[1][j], m[2][j]}; }
};
inline Vec3 operator*(const Mat3& A, const Vec3& v) {
  return {A.m[0][0] * v.x + A.m[0][1] * v.y + A.m[0][2] * v.z,
          A.m[1][0] * v.x + A.m[1][1] * v.y + A.m[1][2] * v.z,
          A.m[2][0] * v.x + A.m[2][1] * v.y + A.m[2][2] * v.z};
}
inline Mat3 operator*(const Mat3& A, const Mat3& B) {
  Mat3 C;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      C.m[i][j] = 0;
      for (int k = 0; k < 3; ++k) C.m[i][j] += A.m[i][k] * B.m[k][j];
    }
  return C;
}
inline Mat3 rotY(double a) {
  Mat3 R;
  double c = std::cos(a), s = std::sin(a);
  R.m[0][0] = c;  R.m[0][1] = 0; R.m[0][2] = s;
  R.m[1][0] = 0;  R.m[1][1] = 1; R.m[1][2] = 0;
  R.m[2][0] = -s; R.m[2][1] = 0; R.m[2][2] = c;
  return R;
}
inline Mat3 rotZ(double a) {
  Mat3 R;
  double c = std::cos(a), s = std::sin(a);
  R.m[0][0] = c; R.m[0][1] = -s; R.m[0][2] = 0;
  R.m[1][0] = s; R.m[1][1] = c;  R.m[1][2] = 0;
  R.m[2][0] = 0; R.m[2][1] = 0;  R.m[2][2] = 1;
  return R;
}

struct Pose {
  Vec3 p;
  Mat3 R;
};

inline double clampd(double v, double lo, double hi) { return std::min(hi, std::max(lo, v)); }

}  // namespace am
