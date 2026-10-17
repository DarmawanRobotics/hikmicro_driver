// "Has the PTZ arrived?" from polled positions. The camera only acknowledges a move command, it
// does not say when it got there, so arrival is decided from the reported position: within
// tolerance and no longer moving for `settle_sec`. A move that stops short (end stop, limit,
// obstruction, lost steps) is reported as STALLED instead of waiting for the timeout.
// No ROS here.
#pragma once

#include <cmath>

#include "hikmicro_ptz_driver/types.hpp"

namespace hikmicro_ptz_driver
{

// Signed shortest difference a - b for angles in degrees, in (-180, 180].
inline double angleDiff(double a, double b)
{
  double d = std::fmod(a - b, 360.0);
  if (d <= -180.0) {d += 360.0;}
  if (d > 180.0) {d -= 360.0;}
  return d;
}

inline double wrap360(double a)
{
  double w = std::fmod(a, 360.0);
  return w < 0.0 ? w + 360.0 : w;
}

struct ArrivalConfig
{
  double pan_tolerance{0.2};    // deg
  double tilt_tolerance{0.2};   // deg
  double zoom_tolerance{0.1};   // x
  bool check_zoom{true};        // false when the goal leaves the zoom alone
  double settle_sec{0.3};       // in tolerance and still for this long: reached
  double still_deg{0.05};       // movement between samples below this counts as still
  double still_zoom{0.02};
  double stall_sec{2.0};        // still but out of tolerance for this long: stalled
  double timeout_sec{30.0};
};

class ArrivalDetector
{
public:
  enum State { MOVING, REACHED, STALLED, TIMEOUT };

  // `has_target` false: wait for the motion to end (preset without a known position).
  ArrivalDetector(const PtzPose & target, const ArrivalConfig & config, double start_time,
    bool has_target = true)
  : target_(target), cfg_(config), start_(start_time), has_target_(has_target) {}

  State update(double t, const PtzPose & p)
  {
    if (state_ != MOVING) {
      return state_;
    }
    bool still = false;
    if (have_last_) {
      still = std::fabs(angleDiff(p.pan, last_.pan)) < cfg_.still_deg &&
        std::fabs(p.tilt - last_.tilt) < cfg_.still_deg &&
        std::fabs(p.zoom - last_.zoom) < cfg_.still_zoom;
      if (!still) {
        moved_ = true;
      }
    }
    last_ = p;
    have_last_ = true;
    if (still) {
      if (still_since_ < 0.0) {
        still_since_ = t;
      }
    } else {
      still_since_ = -1.0;
    }
    const double still_for = still_since_ < 0.0 ? 0.0 : t - still_since_;

    if (has_target_) {
      error_.pan = angleDiff(target_.pan, p.pan);
      error_.tilt = target_.tilt - p.tilt;
      error_.zoom = cfg_.check_zoom ? target_.zoom - p.zoom : 0.0;
      const bool inside = std::fabs(error_.pan) <= cfg_.pan_tolerance &&
        std::fabs(error_.tilt) <= cfg_.tilt_tolerance &&
        (!cfg_.check_zoom || std::fabs(error_.zoom) <= cfg_.zoom_tolerance);
      if (inside && still_for >= cfg_.settle_sec) {
        return state_ = REACHED;
      }
      if (!inside && still_for >= cfg_.stall_sec) {
        return state_ = STALLED;
      }
    } else {
      // Moved and stopped again, or never moved (already there).
      if (still_for >= (moved_ ? cfg_.settle_sec : cfg_.stall_sec)) {
        return state_ = REACHED;
      }
    }
    if (t - start_ >= cfg_.timeout_sec) {
      return state_ = TIMEOUT;
    }
    return state_;
  }

  State state() const {return state_;}
  const PtzPose & error() const {return error_;}
  const PtzPose & last() const {return last_;}

private:
  PtzPose target_;
  ArrivalConfig cfg_;
  double start_;
  bool has_target_;
  State state_{MOVING};
  PtzPose last_{};
  PtzPose error_{};
  bool have_last_{false};
  bool moved_{false};
  double still_since_{-1.0};
};

inline const char * toString(ArrivalDetector::State s)
{
  switch (s) {
    case ArrivalDetector::MOVING: return "moving";
    case ArrivalDetector::REACHED: return "reached";
    case ArrivalDetector::STALLED: return "stalled";
    case ArrivalDetector::TIMEOUT: return "timeout";
  }
  return "unknown";
}

}  // namespace hikmicro_ptz_driver
