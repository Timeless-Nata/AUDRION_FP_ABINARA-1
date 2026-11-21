#include "autonomous_manipulator/robot_model.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace arm {
namespace {

[[noreturn]] void fail(const std::string& path, int line, const std::string& msg) {
  throw std::runtime_error(path + ":" + std::to_string(line) + ": " + msg);
}

void expectKey(std::istringstream& is, const char* key, const std::string& path, int line) {
  std::string k;
  if (!(is >> k) || k != key) fail(path, line, std::string("expected '") + key + "'");
}

Vec3 readVec(std::istream& is) {
  Vec3 v;
  is >> v.x >> v.y >> v.z;
  return v;
}

Mat3 readMat(std::istream& is) {
  Mat3 m;
  for (int i = 0; i < 9; ++i) is >> m.m[i];
  return m;
}

}  // namespace

RobotModel RobotModel::loadFromFile(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open robot model file: " + path);
  RobotModel M;
  std::string line;
  int ln = 0;
  auto nextLine = [&](std::string& out) {
    while (std::getline(f, out)) {
      ++ln;
      if (!out.empty() && out[0] != '#') return true;
    }
    return false;
  };
  while (nextLine(line)) {
    std::istringstream is(line);
    std::string kind;
    is >> kind;
    if (kind == "model") {
      is >> M.name;
    } else if (kind == "ndof") {
      is >> M.ndof;
    } else if (kind == "body") {
      Body b;
      is >> b.name;
      expectKey(is, "parent", path, ln); is >> b.parent;
      expectKey(is, "pos", path, ln);    b.fixed.p = readVec(is);
      expectKey(is, "rot", path, ln);    b.fixed.R = readMat(is);
      expectKey(is, "dof", path, ln);    is >> b.dof;
      expectKey(is, "coef", path, ln);   is >> b.coef;
      expectKey(is, "axis", path, ln);   b.axis = normalized(readVec(is));
      expectKey(is, "limits", path, ln); is >> b.lo >> b.hi;
      if (!is) fail(path, ln, "malformed body record");
      if (b.parent >= static_cast<int>(M.bodies.size())) fail(path, ln, "parent index must precede the body");
      M.bodies.push_back(b);
    } else if (kind == "frame") {
      Frame fr;
      is >> fr.name;
      expectKey(is, "body", path, ln); is >> fr.body;
      expectKey(is, "pos", path, ln);  fr.pose.p = readVec(is);
      expectKey(is, "rot", path, ln);  fr.pose.R = readMat(is);
      if (!is || fr.body < 0 || fr.body >= static_cast<int>(M.bodies.size())) fail(path, ln, "malformed frame record");
      M.frames.push_back(fr);
    } else if (kind == "hull") {
      Hull h;
      int nv = 0, nn = 0, ne = 0;
      is >> h.name;
      expectKey(is, "body", path, ln); is >> h.body;
      expectKey(is, "nv", path, ln);   is >> nv;
      expectKey(is, "nn", path, ln);   is >> nn;
      expectKey(is, "ne", path, ln);   is >> ne;
      if (!is || h.body < 0 || h.body >= static_cast<int>(M.bodies.size()) || nv < 4 || nn < 4)
        fail(path, ln, "malformed hull header");
      auto readN = [&](int n, std::vector<Vec3>& out) {
        for (int i = 0; i < n; ++i) {
          std::string l;
          if (!nextLine(l)) fail(path, ln, "unexpected end of file inside hull " + h.name);
          std::istringstream vs(l);
          Vec3 v = readVec(vs);
          if (!vs) fail(path, ln, "bad vertex/normal/edge line");
          out.push_back(v);
        }
      };
      readN(nv, h.v);
      readN(nn, h.normals);
      readN(ne, h.edges);
      Vec3 mn = h.v[0], mx = h.v[0];
      for (const Vec3& p : h.v) {
        mn = {std::fmin(mn.x, p.x), std::fmin(mn.y, p.y), std::fmin(mn.z, p.z)};
        mx = {std::fmax(mx.x, p.x), std::fmax(mx.y, p.y), std::fmax(mx.z, p.z)};
      }
      h.center = 0.5 * (mn + mx);
      for (const Vec3& p : h.v) h.radius = std::fmax(h.radius, norm(p - h.center));
      M.hulls.push_back(std::move(h));
    } else {
      fail(path, ln, "unknown record '" + kind + "'");
    }
  }
  if (M.bodies.empty() || M.bodies[0].name != "world") throw std::runtime_error(path + ": first body must be 'world'");
  if (M.ndof != kModelDof) throw std::runtime_error(path + ": expected ndof " + std::to_string(kModelDof));
  for (int i = 0; i < kArmDof; ++i) {
    bool found = false;
    for (const Body& b : M.bodies) found = found || b.dof == i;
    if (!found) throw std::runtime_error(path + ": missing joint for arm dof " + std::to_string(i));
  }
  if (M.frameIndex("tool") < 0) throw std::runtime_error(path + ": missing frame 'tool'");
  return M;
}

int RobotModel::bodyIndex(const std::string& n) const {
  for (size_t i = 0; i < bodies.size(); ++i)
    if (bodies[i].name == n) return static_cast<int>(i);
  return -1;
}

int RobotModel::frameIndex(const std::string& n) const {
  for (size_t i = 0; i < frames.size(); ++i)
    if (frames[i].name == n) return static_cast<int>(i);
  return -1;
}

std::array<int, kArmDof> RobotModel::armBodies() const {
  std::array<int, kArmDof> out{};
  for (size_t i = 0; i < bodies.size(); ++i)
    if (bodies[i].dof >= 0 && bodies[i].dof < kArmDof && bodies[i].coef == 1.0) out[bodies[i].dof] = static_cast<int>(i);
  return out;
}

}  // namespace arm
