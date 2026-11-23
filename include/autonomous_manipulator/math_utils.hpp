// math_utils.hpp - tiny dependency-free 3D math for the arm core (no Eigen, no ROS, no MuJoCo).
#pragma once
#include <cmath>
#include <vector>

namespace arm {

constexpr double kPi = 3.14159265358979323846;

struct Vec3 {
  double x = 0, y = 0, z = 0;
};
inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(double s, Vec3 a) { return {s * a.x, s * a.y, s * a.z}; }
inline Vec3 operator-(Vec3 a) { return {-a.x, -a.y, -a.z}; }
inline double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double norm(Vec3 a) { return std::sqrt(dot(a, a)); }
inline Vec3 normalized(Vec3 a) {
  const double n = norm(a);
  return n > 0 ? (1.0 / n) * a : a;
}

// Row-major 3x3 matrix.
struct Mat3 {
  double m[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double& operator()(int r, int c) { return m[3 * r + c]; }
  double operator()(int r, int c) const { return m[3 * r + c]; }
  Vec3 col(int c) const { return {m[c], m[3 + c], m[6 + c]}; }
};
inline Vec3 operator*(const Mat3& A, Vec3 v) {
  return {A.m[0] * v.x + A.m[1] * v.y + A.m[2] * v.z, A.m[3] * v.x + A.m[4] * v.y + A.m[5] * v.z,
          A.m[6] * v.x + A.m[7] * v.y + A.m[8] * v.z};
}
inline Mat3 operator*(const Mat3& A, const Mat3& B) {
  Mat3 C;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) C(r, c) = A(r, 0) * B(0, c) + A(r, 1) * B(1, c) + A(r, 2) * B(2, c);
  return C;
}
inline Mat3 transpose(const Mat3& A) {
  Mat3 T;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) T(r, c) = A(c, r);
  return T;
}
// Rotation by `angle` about the UNIT vector `axis` (Rodrigues).
inline Mat3 rotAxis(Vec3 a, double angle) {
  const double s = std::sin(angle), c = std::cos(angle), t = 1.0 - c;
  Mat3 R;
  R(0, 0) = t * a.x * a.x + c;       R(0, 1) = t * a.x * a.y - s * a.z; R(0, 2) = t * a.x * a.z + s * a.y;
  R(1, 0) = t * a.x * a.y + s * a.z; R(1, 1) = t * a.y * a.y + c;       R(1, 2) = t * a.y * a.z - s * a.x;
  R(2, 0) = t * a.x * a.z - s * a.y; R(2, 1) = t * a.y * a.z + s * a.x; R(2, 2) = t * a.z * a.z + c;
  return R;
}

// Rigid transform: x_parent = R * x_local + p
struct Pose {
  Mat3 R;
  Vec3 p;
};
inline Pose operator*(const Pose& a, const Pose& b) { return {a.R * b.R, a.R * b.p + a.p}; }
inline Vec3 apply(const Pose& T, Vec3 v) { return T.R * v + T.p; }

inline double wrapAngle(double a) {
  while (a > kPi) a -= 2 * kPi;
  while (a < -kPi) a += 2 * kPi;
  return a;
}
inline double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Solve A x = b for a dense n x n system (row-major A), Gaussian elimination with partial pivoting.
// Returns false when A is (numerically) singular.
inline bool solveLinear(std::vector<double> A, std::vector<double> b, int n, std::vector<double>& x) {
  for (int c = 0; c < n; ++c) {
    int piv = c;
    for (int r = c + 1; r < n; ++r)
      if (std::fabs(A[r * n + c]) > std::fabs(A[piv * n + c])) piv = r;
    if (std::fabs(A[piv * n + c]) < 1e-14) return false;
    if (piv != c) {
      for (int k = 0; k < n; ++k) std::swap(A[c * n + k], A[piv * n + k]);
      std::swap(b[c], b[piv]);
    }
    for (int r = c + 1; r < n; ++r) {
      const double f = A[r * n + c] / A[c * n + c];
      for (int k = c; k < n; ++k) A[r * n + k] -= f * A[c * n + k];
      b[r] -= f * b[c];
    }
  }
  x.assign(n, 0.0);
  for (int r = n - 1; r >= 0; --r) {
    double s = b[r];
    for (int k = r + 1; k < n; ++k) s -= A[r * n + k] * x[k];
    x[r] = s / A[r * n + r];
  }
  return true;
}

}  // namespace arm
