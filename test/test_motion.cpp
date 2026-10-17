// Arrival detection from polled PTZ positions.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "hikmicro_ptz_driver/motion.hpp"

using namespace hikmicro_ptz_driver;

namespace
{

PtzPose pose(double pan, double tilt, double zoom = 1.0)
{
  PtzPose p;
  p.pan = pan;
  p.tilt = tilt;
  p.zoom = zoom;
  return p;
}

// Simple PTZ that moves towards `goal` at `speed` deg/s, clamped to `tilt_max`.
struct SimPtz
{
  PtzPose p;
  PtzPose goal;
  double speed{60.0};
  double tilt_max{90.0};
  void step(double dt)
  {
    const double dp = angleDiff(goal.pan, p.pan);
    p.pan = wrap360(p.pan + std::max(-speed * dt, std::min(speed * dt, dp)));
    const double dtl = goal.tilt - p.tilt;
    p.tilt += std::max(-speed * dt, std::min(speed * dt, dtl));
    p.tilt = std::min(p.tilt, tilt_max);
    p.zoom += std::max(-5 * dt, std::min(5 * dt, goal.zoom - p.zoom));
  }
};

ArrivalDetector::State run(SimPtz & sim, ArrivalDetector & det, double dt, double max_t,
  double * t_end)
{
  double t = 0.0;
  ArrivalDetector::State s = ArrivalDetector::MOVING;
  while (t < max_t && s == ArrivalDetector::MOVING) {
    t += dt;
    sim.step(dt);
    s = det.update(t, sim.p);
  }
  *t_end = t;
  return s;
}

}  // namespace

TEST(Motion, AngleDiffWraps)
{
  EXPECT_DOUBLE_EQ(angleDiff(10.0, 350.0), 20.0);
  EXPECT_DOUBLE_EQ(angleDiff(350.0, 10.0), -20.0);
  EXPECT_DOUBLE_EQ(angleDiff(180.0, 0.0), 180.0);
  EXPECT_DOUBLE_EQ(angleDiff(0.0, 180.0), 180.0);
  EXPECT_DOUBLE_EQ(angleDiff(725.0, 5.0), 0.0);
  EXPECT_DOUBLE_EQ(wrap360(-10.0), 350.0);
  EXPECT_DOUBLE_EQ(wrap360(370.0), 10.0);
}

TEST(Motion, ReachesTargetAcrossZero)
{
  SimPtz sim;
  sim.p = pose(350.0, 0.0, 1.0);
  sim.goal = pose(20.0, -30.0, 4.0);   // shortest way is through 0
  ArrivalConfig cfg;
  ArrivalDetector det(sim.goal, cfg, 0.0);
  double t;
  EXPECT_EQ(run(sim, det, 0.1, 10.0, &t), ArrivalDetector::REACHED);
  // 30 deg at 60 deg/s = 0.5 s (+ zoom 0.6 s) + settle 0.3 s
  EXPECT_LT(t, 1.5);
  EXPECT_NEAR(det.last().pan, 20.0, cfg.pan_tolerance);
  EXPECT_NEAR(det.error().tilt, 0.0, cfg.tilt_tolerance);
}

TEST(Motion, NotReachedWhilePassingThrough)
{
  // a sample inside the tolerance while still moving must not count
  ArrivalConfig cfg;
  cfg.settle_sec = 0.3;
  ArrivalDetector det(pose(10.0, 0.0), cfg, 0.0);
  EXPECT_EQ(det.update(0.1, pose(9.0, 0.0)), ArrivalDetector::MOVING);
  EXPECT_EQ(det.update(0.2, pose(10.1, 0.0)), ArrivalDetector::MOVING);   // overshoot
  EXPECT_EQ(det.update(0.3, pose(10.15, 0.0)), ArrivalDetector::MOVING);
  EXPECT_EQ(det.update(0.4, pose(10.0, 0.0)), ArrivalDetector::MOVING);
  EXPECT_EQ(det.update(0.5, pose(10.0, 0.0)), ArrivalDetector::MOVING);   // still from 0.5
  EXPECT_EQ(det.update(0.8, pose(10.0, 0.0)), ArrivalDetector::REACHED);  // still 0.3 s
}

TEST(Motion, StallsAtLimit)
{
  SimPtz sim;
  sim.p = pose(0.0, 0.0);
  sim.goal = pose(0.0, 120.0);   // beyond the 90 deg end stop
  ArrivalConfig cfg;
  cfg.stall_sec = 1.0;
  ArrivalDetector det(sim.goal, cfg, 0.0);
  double t;
  EXPECT_EQ(run(sim, det, 0.1, 30.0, &t), ArrivalDetector::STALLED);
  EXPECT_NEAR(det.last().tilt, 90.0, 1e-9);
  EXPECT_NEAR(det.error().tilt, 30.0, 1e-9);
  EXPECT_LT(t, 3.0);   // 1.5 s to the stop + 1 s still, long before the timeout
}

TEST(Motion, TimesOut)
{
  SimPtz sim;
  sim.p = pose(0.0, 0.0);
  sim.goal = pose(180.0, 0.0);
  sim.speed = 1.0;   // far too slow
  ArrivalConfig cfg;
  cfg.timeout_sec = 2.0;
  ArrivalDetector det(sim.goal, cfg, 0.0);
  double t;
  EXPECT_EQ(run(sim, det, 0.1, 10.0, &t), ArrivalDetector::TIMEOUT);
  EXPECT_NEAR(t, 2.0, 0.11);
}

TEST(Motion, ZoomIgnoredWhenNotRequested)
{
  ArrivalConfig cfg;
  cfg.check_zoom = false;
  ArrivalDetector det(pose(5.0, 5.0, 0.0), cfg, 0.0);
  det.update(0.0, pose(5.0, 5.0, 12.0));
  EXPECT_EQ(det.update(0.1, pose(5.0, 5.0, 12.0)), ArrivalDetector::MOVING);   // still since 0.1
  EXPECT_EQ(det.update(0.5, pose(5.0, 5.0, 12.0)), ArrivalDetector::REACHED);
}

TEST(Motion, WaitForStopWithoutTarget)
{
  ArrivalConfig cfg;
  cfg.stall_sec = 1.0;
  // moves, then stops: reached after settle
  ArrivalDetector moving(PtzPose{}, cfg, 0.0, false);
  EXPECT_EQ(moving.update(0.0, pose(0.0, 0.0)), ArrivalDetector::MOVING);
  EXPECT_EQ(moving.update(0.1, pose(5.0, 0.0)), ArrivalDetector::MOVING);
  EXPECT_EQ(moving.update(0.2, pose(8.0, 0.0)), ArrivalDetector::MOVING);
  EXPECT_EQ(moving.update(0.3, pose(8.0, 0.0)), ArrivalDetector::MOVING);
  EXPECT_EQ(moving.update(0.7, pose(8.0, 0.0)), ArrivalDetector::REACHED);
  // never moves (already there): reached after stall_sec
  ArrivalDetector already(PtzPose{}, cfg, 0.0, false);
  already.update(0.0, pose(1.0, 1.0));
  EXPECT_EQ(already.update(0.5, pose(1.0, 1.0)), ArrivalDetector::MOVING);
  EXPECT_EQ(already.update(1.6, pose(1.0, 1.0)), ArrivalDetector::REACHED);  // still 1.1 s
}
