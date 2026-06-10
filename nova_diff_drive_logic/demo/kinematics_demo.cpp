// Dependency-free demo of nova_diff_drive_logic — build motion-control intuition.
// No ROS / no WMX needed.
//
//   compile:  g++ -std=c++17 -I../include kinematics_demo.cpp -o kinematics_demo
//   run:      ./kinematics_demo                 # canonical scenarios
//             ./kinematics_demo <v> <w> <secs>  # your own: e.g. ./kinematics_demo 0.4 0.6 10
//
// Shows: cmd_vel (v,w) -> wheel angular velocities (inverse kinematics),
// and the path the robot traces (odometry integration) as an ASCII top-down map.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#include "nova_diff_drive_logic/diff_drive_kinematics.hpp"
#include "nova_diff_drive_logic/odometry_integrator.hpp"
#include "nova_diff_drive_logic/odom_delta_accumulator.hpp"

using namespace nova_diff_drive_logic;

static void inverseTable(const DiffDriveModel & m)
{
  std::printf(
    "\n=== 역기구학: 명령속도 -> 바퀴 각속도  (R=%.3f m, L=%.3f m) ===\n",
    m.wheel_radius, m.wheel_separation);
  std::printf("   v[m/s]  w[rad/s] |  wheel_L[rad/s]  wheel_R[rad/s]   해석\n");
  struct Case {double v, w; const char * note;};
  const Case cases[] = {
    {0.50, 0.00, "직진 (양 바퀴 같은 속도)"},
    {0.00, 1.00, "제자리 좌회전 (반대 부호)"},
    {0.00, -1.00, "제자리 우회전"},
    {0.50, 0.50, "좌호선 (왼쪽이 느림)"},
    {0.50, -0.50, "우호선 (오른쪽이 느림)"},
  };
  for (const auto & c : cases) {
    const auto w = m.inverse({c.v, c.w});
    std::printf("   %5.2f  %7.2f  | %13.3f  %13.3f   %s\n", c.v, c.w, w.left, w.right, c.note);
  }
}

static void plotPath(const DiffDriveModel & m, double v, double w, double secs)
{
  const double dt = 0.05;
  OdometryIntegrator odom;
  OdomDeltaAccumulator deltas;
  std::vector<std::pair<double, double>> pts{{0.0, 0.0}};

  const int steps = static_cast<int>(secs / dt);
  for (int i = 0; i < steps; ++i) {
    odom.integrate({v, w}, dt);
    deltas.accumulate({v, w}, dt);
    pts.emplace_back(odom.pose().x, odom.pose().y);
  }
  const auto p = odom.pose();
  const auto d = deltas.peek();

  std::printf("\n=== 주행: v=%.2f m/s, w=%.2f rad/s, %.1f s ===\n", v, w, secs);
  std::printf(
    "   최종 pose: x=%.3f m, y=%.3f m, heading=%.1f deg  |  이동거리=%.2f m, 누적회전=%.2f rad\n",
    p.x, p.y, p.theta * 180.0 / M_PI, d.linear, d.angular);

  // ASCII top-down map: horizontal = x (forward +), vertical = y (left +, up on screen).
  double minx = 1e9, maxx = -1e9, miny = 1e9, maxy = -1e9;
  for (const auto & q : pts) {
    minx = std::min(minx, q.first); maxx = std::max(maxx, q.first);
    miny = std::min(miny, q.second); maxy = std::max(maxy, q.second);
  }
  const double padx = (maxx - minx) * 0.1 + 1e-2, pady = (maxy - miny) * 0.1 + 1e-2;
  minx -= padx; maxx += padx; miny -= pady; maxy += pady;

  const int W = 51, H = 19;
  std::vector<std::string> grid(H, std::string(W, ' '));
  for (size_t i = 0; i < pts.size(); ++i) {
    const int c = static_cast<int>(std::lround((pts[i].first - minx) / (maxx - minx) * (W - 1)));
    const int r = static_cast<int>(std::lround((maxy - pts[i].second) / (maxy - miny) * (H - 1)));
    if (r >= 0 && r < H && c >= 0 && c < W) {
      grid[r][c] = (i == 0) ? 'O' : (i + 1 == pts.size() ? 'E' : '.');
    }
  }
  std::printf("   (O=시작, E=끝, .=경로 | 가로→ = 전진 +x, 세로↑ = 왼쪽 +y)\n");
  for (const auto & line : grid) {std::printf("   |%s|\n", line.c_str());}
}

int main(int argc, char ** argv)
{
  const DiffDriveModel m{0.095, 0.55};  // Nova-ish defaults (wheel radius, separation)
  inverseTable(m);

  if (argc >= 3) {
    const double v = std::atof(argv[1]);
    const double w = std::atof(argv[2]);
    const double secs = (argc >= 4) ? std::atof(argv[3]) : 5.0;
    plotPath(m, v, w, secs);
  } else {
    plotPath(m, 0.5, 0.0, 4.0);    // straight line
    plotPath(m, 0.0, 1.0, 3.14);   // spin in place (~half turn) -> position doesn't change
    plotPath(m, 0.5, 0.5, 10.0);   // arc / circle (radius = v/w = 1.0 m)
  }
  std::printf("\n직접 해보기:  ./kinematics_demo <v> <w> <secs>   예) ./kinematics_demo 0.4 0.6 10\n");
  return 0;
}
