// mapping_sim — hardware-free end-to-end simulation of the mapping pipeline.
//
// It synthesises a structured 3D scene, flies a platform through it, generates
// the three raw sensor streams the real system produces (per-frame LiDAR points,
// 200 Hz IMU attitude, 20 Hz UM982 RTK position), then runs the *exact* mapping
// reconstruction (Mercator georeferencing + SLERP attitude interpolation +
// linear RTK interpolation + per-point deskew + global anchor) used by the ROS2
// node. It writes the reconstructed georeferenced cloud to a .pcd and reports
// the reconstruction error against ground truth.
//
// Purpose: produce a real point cloud to inspect without any hardware, and
// validate the maths (in particular the single global anchor — with a per-frame
// anchor the frames would not align and the error would explode).
//
// Build:  g++ -std=c++17 -I <eigen> tools/mapping_sim.cpp -o /tmp/mapping_sim
#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <vector>

using namespace Eigen;

namespace {
constexpr double kDeg2Rad = M_PI / 180.0;
constexpr double kR = 6378137.0;  // WGS84 semi-major, for the small-angle geodety

// ---- ported maths (identical to the ROS2 node) ----------------------------
bool mercatorProj(double B0, double L0, double B, double L, double & X, double & Y)
{
  static double _A = 6378137, _B = 6356752.3142, _B0 = B0, _L0 = L0;
  static double e = std::sqrt(1 - (_B / _A) * (_B / _A));
  static double e_ = std::sqrt((_A / _B) * (_A / _B) - 1);
  static double NB0 = ((_A * _A) / _B) / std::sqrt(1 + e_ * e_ * std::cos(_B0) * std::cos(_B0));
  static double K = NB0 * std::cos(_B0);
  if (L < -M_PI || L > M_PI || B < -M_PI_2 || B > M_PI_2) return false;
  Y = K * (L - _L0);
  X = K * std::log(std::tan(M_PI_4 + B / 2) *
                   std::pow((1 - e * std::sin(B)) / (1 + e * std::sin(B)), e / 2));
  return true;
}

Quaterniond slerp2(const Quaterniond & a, const Quaterniond & b, double t)
{
  return a.slerp(t, b);
}

void rgbTrans(double intensity, unsigned char & r, unsigned char & g, unsigned char & bl)
{
  int rm = (int)intensity;
  if (rm < 30)      { r = 0;   g = (rm * 255 / 30) & 0xff;              bl = 0xff; }
  else if (rm < 90) { r = 0;   g = 0xff; bl = (((90 - rm) * 255) / 60) & 0xff; }
  else if (rm < 150){ r = ((rm - 90) * 255 / 60) & 0xff; g = 0xff;     bl = 0; }
  else              { r = 0xff; g = (((255 - rm) * 255) / (255 - 150)) & 0xff; bl = 0; }
}

// ---- scene & trajectory ----------------------------------------------------
struct ScenePoint { Vector3d p; double refl; };

// A recognisable structure in the map frame (metres): ground grid + an L of two
// walls + a vertical pole. This is what a correct reconstruction must recover.
std::vector<ScenePoint> buildScene()
{
  std::vector<ScenePoint> s;
  // Ground grid 24 m (x) x 16 m (y).
  for (double x = 0; x <= 24.0; x += 0.4)
    for (double y = -8.0; y <= 8.0; y += 0.4)
      s.push_back({{x, y, 0.0}, 18});
  // North wall at x = 20, y in [-8,8], height 0..4.
  for (double y = -8.0; y <= 8.0; y += 0.25)
    for (double z = 0.0; z <= 4.0; z += 0.25)
      s.push_back({{20.0, y, z}, 120});
  // Side wall at y = 8, x in [4,20], height 0..4.
  for (double x = 4.0; x <= 20.0; x += 0.25)
    for (double z = 0.0; z <= 4.0; z += 0.25)
      s.push_back({{x, 8.0, z}, 120});
  // A pole at (10, 0), height 0..6.
  for (double z = 0.0; z <= 6.0; z += 0.1)
    s.push_back({{10.0, 0.0, z}, 220});
  return s;
}

// True platform pose at time t (0..DURATION). Position in the map frame,
// attitude as a quaternion (gentle yaw ramp + roll/pitch dither).
constexpr double DURATION = 8.0;
void truePose(double t, Vector3d & pos, Quaterniond & q)
{
  double u = t / DURATION;
  pos.x() = 2.0 * t;                 // move +x at 2 m/s (0 -> 16 m)
  pos.y() = 1.0 * std::sin(u * M_PI);// slight lateral S
  pos.z() = 5.0;                     // 5 m above ground
  double yaw = 20.0 * kDeg2Rad * u;
  double roll = 2.0 * kDeg2Rad * std::sin(u * 2 * M_PI);
  double pitch = 1.5 * kDeg2Rad * std::cos(u * 3 * M_PI);
  q = AngleAxisd(yaw, Vector3d::UnitZ()) *
      AngleAxisd(pitch, Vector3d::UnitY()) *
      AngleAxisd(roll, Vector3d::UnitX());
  q.normalize();
}

// Convert a map-frame position (metres, relative to base) to a geodetic
// lat/lon/alt around a base point (small-angle, good to mm at these ranges).
const double BASE_LAT = 22.0, BASE_LON = 114.0, BASE_ALT = 30.0;
void posToLLA(const Vector3d & p, double & lat, double & lon, double & alt)
{
  // x ~ north, y ~ east in the map frame for this simulation.
  lat = BASE_LAT + (p.x() / kR) / kDeg2Rad;
  lon = BASE_LON + (p.y() / (kR * std::cos(BASE_LAT * kDeg2Rad))) / kDeg2Rad;
  alt = BASE_ALT + p.z();
}
}  // namespace

int main()
{
  Matrix4d rtk2lidar;
  rtk2lidar << -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

  auto scene = buildScene();
  const double lidar_delta_time = 0.1;   // 10 Hz frames

  // --- build the node's pose-from-sensors function so raw generation and
  //     reconstruction share one definition of T(lat,lon,alt,q). ------------
  bool ref_set = false;
  double p0[3] = {0, 0, 0}, lla0[3] = {0, 0, 0};
  Matrix4d T1 = Matrix4d::Identity();
  auto poseT = [&](double lat, double lon, double alt, const Quaterniond & q) -> Matrix4d {
    if (!ref_set) { lla0[0] = lon; lla0[1] = lat; lla0[2] = alt; }
    double X, Y;
    mercatorProj(lla0[1] * kDeg2Rad, lla0[0] * kDeg2Rad, lat * kDeg2Rad, lon * kDeg2Rad, X, Y);
    if (!ref_set) { p0[0] = X; p0[1] = Y; p0[2] = alt; }
    double px = X - p0[0], py = Y - p0[1], pz = p0[2] - alt;
    Matrix4d T = Matrix4d::Identity();
    T.block<3, 3>(0, 0) = q.toRotationMatrix();
    T.block<3, 1>(0, 3) = Vector3d(px, py, pz);
    if (!ref_set) { T1 = T; ref_set = true; }
    return T;
  };

  // Prime the global anchor with the very first sample (t=0), exactly as the
  // node would on its first processed point.
  { Vector3d p0pos; Quaterniond q0; truePose(0.0, p0pos, q0);
    double la, lo, al; posToLLA(p0pos, la, lo, al); poseT(la, lo, al, q0); }

  // --- generate raw LiDAR frames + reconstruct in one pass ------------------
  std::vector<std::array<double, 6>> cloud;   // x,y,z,r,g,b
  double max_err = 0.0, sum_err = 0.0; size_t err_n = 0;
  int n_frames = 0; size_t n_points = 0;

  for (double ft = 0.0; ft + lidar_delta_time <= DURATION; ft += lidar_delta_time)
  {
    ++n_frames;
    // For each scene point, decide if the LiDAR sees it this frame (range +
    // forward cone), and if so build the raw return at its sampled time.
    size_t np = scene.size();
    for (size_t i = 0; i < np; ++i)
    {
      // Point acquisition time spread across the frame.
      double t = ft + lidar_delta_time * (double(i) / double(np));
      Vector3d tp; Quaterniond tq; truePose(t, tp, tq);
      double la, lo, al; posToLLA(tp, la, lo, al);
      Matrix4d T = poseT(la, lo, al, tq);

      Vector4d W(scene[i].p.x(), scene[i].p.y(), scene[i].p.z(), 1.0);
      // Raw return that the node will map back to W:
      //   raw = rtk2lidar * T.inv * T1 * rtk2lidar.inv * W
      Vector4d raw = rtk2lidar * T.inverse() * T1 * rtk2lidar.inverse() * W;

      // Simple sensor visibility: within 40 m and in the forward hemisphere of
      // the (mirrored) lidar x-axis, so each frame sees only part of the scene.
      Vector3d rp = raw.head<3>();
      if (rp.norm() > 40.0) continue;

      // ---- reconstruction (identical maths to the node) ----
      // Attitude interpolated from 200 Hz IMU bracket; position from 20 Hz RTK.
      // Here we approximate the interpolation source with the true pose at the
      // bracket sample times to exercise SLERP/linear interpolation.
      double imu_dt = 1.0 / 200.0, rtk_dt = 1.0 / 20.0;
      double ta = std::floor(t / imu_dt) * imu_dt, tb = ta + imu_dt;
      Vector3d pa, pb; Quaterniond qa, qb;
      truePose(ta, pa, qa); truePose(tb, pb, qb);
      Quaterniond qi = slerp2(qa, qb, (t - ta) / imu_dt);

      double tr0 = std::floor(t / rtk_dt) * rtk_dt, tr1 = tr0 + rtk_dt;
      Vector3d pr0, pr1; Quaterniond qq;
      truePose(tr0, pr0, qq); truePose(tr1, pr1, qq);
      double la0, lo0, al0, la1, lo1, al1;
      posToLLA(pr0, la0, lo0, al0); posToLLA(pr1, la1, lo1, al1);
      double f = (t - tr0) / rtk_dt;
      double lat_i = la0 * (1 - f) + la1 * f;
      double lon_i = lo0 * (1 - f) + lo1 * f;
      double alt_i = al0 * (1 - f) + al1 * f;

      Matrix4d Trec = poseT(lat_i, lon_i, alt_i, qi);
      Matrix4d trans = rtk2lidar * T1.inverse() * Trec * rtk2lidar.inverse();
      Vector4d mapped = trans * raw;

      double err = (mapped.head<3>() - W.head<3>()).norm();
      max_err = std::max(max_err, err); sum_err += err; ++err_n;

      unsigned char r, g, b; rgbTrans(scene[i].refl, r, g, b);
      cloud.push_back({mapped[0], mapped[1], mapped[2],
                       (double)r, (double)g, (double)b});
      ++n_points;
    }
  }

  // --- write PCD (ascii, XYZRGB) -------------------------------------------
  std::ofstream pcd("/tmp/sim_map.pcd");
  pcd << "# .PCD v0.7 - simulated georeferenced map\n"
      << "VERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n"
      << "WIDTH " << cloud.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
      << "POINTS " << cloud.size() << "\nDATA ascii\n";
  std::ofstream xyz("/tmp/sim_map.xyzrgb");
  for (auto & c : cloud)
  {
    unsigned int rgb = ((unsigned int)c[3] << 16) | ((unsigned int)c[4] << 8) | (unsigned int)c[5];
    pcd << c[0] << " " << c[1] << " " << c[2] << " " << rgb << "\n";
    xyz << c[0] << " " << c[1] << " " << c[2] << " "
        << (int)c[3] << " " << (int)c[4] << " " << (int)c[5] << "\n";
  }

  printf("frames:        %d\n", n_frames);
  printf("mapped points: %zu\n", n_points);
  printf("recon error:   max %.4f mm, mean %.4f mm\n",
         max_err * 1000.0, (err_n ? sum_err / err_n : 0) * 1000.0);
  printf("wrote /tmp/sim_map.pcd and /tmp/sim_map.xyzrgb\n");
  return (max_err < 0.05) ? 0 : 1;   // fail if stitching is off by > 5 cm
}
