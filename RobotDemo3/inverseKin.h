// inverseKin.h — inverse / forward kinematics for the UH ECE servo arm
// DH table (HW3):
//   link | alpha | r       | d        | theta
//   1    | pi/2  | 0       | d1=110mm | th1
//   2    | 0     | r2=104  | 0        | th2
//   3    | 0     | r3=99   | 0        | th3
//   4    | pi/2  | 0       | 0        | th4
//   5    | 0     | 0       | d5=168   | th5   (wrist roll, does not move the tool point)
#pragma once
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace inverseKin {

struct ArmGeometry {
  float d1, r2, r3, d5;   // mm
};

struct Solution {
  float theta[4];       // th1..th4 in radians (DH convention)
  bool reachable;    // geometrically reachable (|D| <= 1)
  bool back;         // behind-the-back branch (th1 = atan2(oy,ox) + pi)
  bool elbowUp;      // elbow joint is the higher of the two elbow options
};

inline float wrapPi(float a) {
  while (a > (float)M_PI)   a -= 2.0f * (float)M_PI;
  while (a <= -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}

// Inverse kinematics for the tool point o5.
//   (px,py,pz) : desired tool point in frame 0 (mm)
//   pitch      : direction the gripper points, measured from horizontal (pointing
//                radially away from the base) — radians. -pi/2 = straight down.
//   solution[4]: filled in this order
//                0 = front, elbow up     1 = front, elbow down
//                2 = back,  elbow up     3 = back,  elbow down
// Returns the number of geometrically reachable solutions (0, 2 or 4).
inline int solve(const ArmGeometry& g, float px, float py, float pz, float pitch, Solution solution[4]) {
  const float rxy = sqrtf(px * px + py * py);
  const float psi = (rxy < 1e-4f) ? 0.0f : atan2f(py, px);
  int count = 0;

  for (int b = 0; b < 2; b++) {
    // theta1: front = atan2(oy,ox), behind-the-back = front + pi
    const float theta1 = b ? wrapPi(psi + (float)M_PI) : psi;
    // Coordinates in the arm plane (x1 horizontal, y1 = z0 vertical).
    // Behind the back, x1 points away from the target, so the radial distance is negative.
    const float u = b ? -rxy : rxy;
    const float toolAng = b ? (float)M_PI - pitch : pitch;   // tool direction in x1-y1 plane

    // Wrist center o3 = o5 - d5 * z5   (HW3 #7)
    const float wx = u - g.d5 * cosf(toolAng);
    const float wy = pz - g.d5 * sinf(toolAng) - g.d1;      // relative to the shoulder

    // Law of cosines (HW3 #3, #4)
    const float D = (wx * wx + wy * wy - g.r2 * g.r2 - g.r3 * g.r3) / (2.0f * g.r2 * g.r3);

    Solution s[2];
    for (int e = 0; e < 2; e++) {
      Solution& q = s[e];
      q.back = (b == 1);
      q.th[0] = th1;
      if (fabsf(D) > 1.0f) {
        q.reachable = false;
        q.th[1] = q.th[2] = q.th[3] = 0.0f;
        continue;
      }
      const float s3 = (e == 0 ? 1.0f : -1.0f) * sqrtf(1.0f - D * D);
      const float th3 = atan2f(s3, D);
      const float th2 = atan2f(wy, wx) - atan2f(g.r3 * sinf(th3), g.r2 + g.r3 * cosf(th3));
      // beta = th2 + th3 + th4 is the angle of x4; z4 (the gripper) is at beta - 90 deg  (HW3 #6)
      const float th4 = toolAng + (float)M_PI / 2.0f - th2 - th3;
      q.th[1] = wrapPi(th2);
      q.th[2] = wrapPi(th3);
      q.th[3] = wrapPi(th4);
      q.reachable = true;
      count++;
    }
    // Label by actual elbow height so "up" always means what you see.
    const float h0 = sinf(s[0].th[1]), h1 = sinf(s[1].th[1]);
    const bool firstIsUp = !(s[0].reachable && s[1].reachable) || (h0 >= h1);
    sol[2 * b + 0] = firstIsUp ? s[0] : s[1];
    sol[2 * b + 1] = firstIsUp ? s[1] : s[0];
    sol[2 * b + 0].elbowUp = true;
    sol[2 * b + 1].elbowUp = false;
  }
  return count;
}

// Forward kinematics: tool point o5 (mm, frame 0) from th1..th4 (radians).
inline void fk(const ArmGeom& g, const float th[4], float out[3]) {
  const float a2 = th[1], a23 = th[1] + th[2], tool = th[1] + th[2] + th[3] - (float)M_PI / 2.0f;
  const float rho = g.r2 * cosf(a2) + g.r3 * cosf(a23) + g.d5 * cosf(tool);
  out[0] = cosf(th[0]) * rho;
  out[1] = sinf(th[0]) * rho;
  out[2] = g.d1 + g.r2 * sinf(a2) + g.r3 * sinf(a23) + g.d5 * sinf(tool);
}

}  // namespace ik
