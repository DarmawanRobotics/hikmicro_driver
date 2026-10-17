#include "hikmicro_ptz_driver/ptz_interface.hpp"

#include <sensor_msgs/JointState.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

#include "hikmicro_ptz_driver/PtzState.h"

namespace hikmicro_ptz_driver
{

namespace
{

constexpr double kDegToRad = M_PI / 180.0;

int speed(double axis, int max_speed, bool invert)
{
  if (!std::isfinite(axis)) {
    return 0;
  }
  axis = std::max(-1.0, std::min(1.0, axis));
  return static_cast<int>(std::lround((invert ? -axis : axis) * max_speed));
}

double wrapPi(double rad)
{
  return std::atan2(std::sin(rad), std::cos(rad));
}

}  // namespace

double PtzInterface::now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
         .count();
}

PtzInterface::PtzInterface(
  ros::NodeHandle & nh, ros::NodeHandle & pnh, std::shared_ptr<HikCamera> camera)
: cam_(std::move(camera))
{
  pnh.param("state_rate", state_rate_, 10.0);
  pnh.param("pan_tolerance", pan_tol_, 0.2);
  pnh.param("tilt_tolerance", tilt_tol_, 0.2);
  pnh.param("zoom_tolerance", zoom_tol_, 0.1);
  pnh.param("settle_time", settle_, 0.3);
  pnh.param("stall_time", stall_, 2.0);
  pnh.param("move_timeout", move_timeout_, 30.0);
  pnh.param("cmd_vel_timeout", cmd_vel_timeout_, 0.5);
  pnh.param("max_speed", max_speed_, 60);
  max_speed_ = std::max(0, std::min(100, max_speed_));
  pnh.param("invert_pan", invert_pan_, false);
  pnh.param("invert_tilt", invert_tilt_, false);
  pnh.param("invert_zoom", invert_zoom_, false);
  pnh.param<std::string>("frame_id", frame_id_, "hikmicro_ptz");
  pnh.param("publish_joint_states", joint_states_, true);
  pnh.param<std::string>("pan_joint", pan_joint_, "hikmicro_pan_joint");
  pnh.param<std::string>("tilt_joint", tilt_joint_, "hikmicro_tilt_joint");
  pnh.param("pan_sign", pan_sign_, 1.0);
  pnh.param("tilt_sign", tilt_sign_, 1.0);

  state_pub_ = nh.advertise<PtzState>("ptz/state", 10);
  if (joint_states_) {
    joint_pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
  }
  cmd_vel_sub_ = nh.subscribe("ptz/cmd_vel", 10, &PtzInterface::onCmdVel, this);
  stop_srv_ = nh.advertiseService("ptz/stop", &PtzInterface::onStop, this);
  focus_srv_ = nh.advertiseService("ptz/focus", &PtzInterface::onFocus, this);
  set_preset_srv_ = nh.advertiseService("ptz/set_preset", &PtzInterface::onSetPreset, this);
  remove_preset_srv_ =
    nh.advertiseService("ptz/remove_preset", &PtzInterface::onRemovePreset, this);
  presets_srv_ = nh.advertiseService("ptz/get_presets", &PtzInterface::onGetPresets, this);
  watchdog_ = nh.createTimer(ros::Duration(0.05), &PtzInterface::onWatchdog, this);

  move_server_ = std::make_unique<actionlib::SimpleActionServer<MoveToAction>>(
    nh, "ptz/move_to", [this](const MoveToGoalConstPtr & g) {executeMoveTo(g);}, false);
  preset_server_ = std::make_unique<actionlib::SimpleActionServer<GotoPresetAction>>(
    nh, "ptz/goto_preset", [this](const GotoPresetGoalConstPtr & g) {executeGotoPreset(g);},
    false);

  state_thread_ = std::thread(&PtzInterface::stateLoop, this);
  move_server_->start();
  preset_server_->start();
}

PtzInterface::~PtzInterface()
{
  running_ = false;
  sample_cv_.notify_all();
  if (state_thread_.joinable()) {
    state_thread_.join();
  }
  move_server_->shutdown();
  preset_server_->shutdown();
}

// ---------------------------------------------------------------------------- state polling

void PtzInterface::stateLoop()
{
  const auto period = std::chrono::duration<double>(1.0 / std::max(state_rate_, 0.5));
  PtzPose prev;
  bool have_prev = false;
  bool was_connected = false;
  uint64_t seq = 0;
  auto next = std::chrono::steady_clock::now();
  while (running_ && ros::ok()) {
    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    std::string err;
    const auto pose = cam_->position(&err);
    const bool ok = pose.has_value();
    connected_ = ok;
    if (ok && !was_connected) {
      // first contact or back after an outage: the firmware may have changed
      const bool hp = cam_->probeHighPrecision();
      ROS_INFO("camera reachable, PTZ position %s", hp ? "through absoluteEx (0.001 deg)" :
        "through /status (0.1 deg)");
    } else if (!ok && was_connected) {
      ROS_WARN("lost the camera: %s", err.c_str());
    } else if (!ok) {
      ROS_WARN_THROTTLE(10.0, "camera not reachable: %s", err.c_str());
    }
    was_connected = ok;

    PtzState msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    msg.connected = ok;
    msg.high_precision = cam_->highPrecision();
    if (ok) {
      msg.pan = pose->pan;
      msg.tilt = pose->tilt;
      msg.zoom = pose->zoom;
      msg.moving = have_prev && (std::fabs(angleDiff(pose->pan, prev.pan)) > 0.01 ||
        std::fabs(pose->tilt - prev.tilt) > 0.01 || std::fabs(pose->zoom - prev.zoom) > 0.01);
      prev = *pose;
      have_prev = true;
      {
        std::lock_guard<std::mutex> lk(sample_mtx_);
        sample_.pose = *pose;
        sample_.t = now();
        sample_.seq = ++seq;
      }
      sample_cv_.notify_all();
      if (joint_states_) {
        sensor_msgs::JointState js;
        js.header = msg.header;
        js.name = {pan_joint_, tilt_joint_};
        js.position = {wrapPi(pan_sign_ * pose->pan * kDegToRad),
          tilt_sign_ * pose->tilt * kDegToRad};
        joint_pub_.publish(js);
      }
    }
    state_pub_.publish(msg);
    std::this_thread::sleep_until(next);
    if (std::chrono::steady_clock::now() > next + std::chrono::seconds(1)) {
      next = std::chrono::steady_clock::now();  // a slow request: do not try to catch up
    }
  }
}

bool PtzInterface::waitSample(uint64_t after_seq, double timeout_sec, Sample * out)
{
  std::unique_lock<std::mutex> lk(sample_mtx_);
  const bool got = sample_cv_.wait_for(lk, std::chrono::duration<double>(timeout_sec),
      [&] {return sample_.seq > after_seq || !running_;});
  if (!got || !running_) {
    return false;
  }
  *out = sample_;
  return true;
}

// ---------------------------------------------------------------------------- actions

ArrivalConfig PtzInterface::arrivalConfig(double tolerance, double timeout) const
{
  ArrivalConfig c;
  c.pan_tolerance = tolerance > 0.0 ? tolerance : pan_tol_;
  c.tilt_tolerance = tolerance > 0.0 ? tolerance : tilt_tol_;
  c.zoom_tolerance = zoom_tol_;
  c.settle_sec = settle_;
  c.stall_sec = stall_;
  c.timeout_sec = timeout > 0.0 ? timeout : move_timeout_;
  // resolution of the position the camera reports
  const double still = cam_->highPrecision() ? 0.005 : 0.05;
  c.still_deg = still;
  return c;
}

template<typename Server, typename Feedback>
PtzInterface::Outcome PtzInterface::track(Server & server, const PtzPose & target,
  bool has_target, const ArrivalConfig & cfg, uint64_t token)
{
  Outcome out;
  const double t0 = now();
  ArrivalDetector det(target, cfg, t0, has_target);
  uint64_t seq;
  {
    std::lock_guard<std::mutex> lk(sample_mtx_);
    seq = sample_.seq;  // only samples taken after the command count
  }
  while (ros::ok() && running_) {
    if (server.isPreemptRequested()) {
      cam_->stop();
      out.label = "preempted";
      out.message = "preempted, PTZ stopped";
      return out;
    }
    if (motion_token_.load() != token) {
      out.label = "superseded";
      out.message = "superseded by a newer motion command";
      return out;
    }
    Sample s;
    if (!waitSample(seq, 1.0, &s)) {
      if (now() - t0 > cfg.timeout_sec) {
        out.state = ArrivalDetector::TIMEOUT;
        out.label = "timeout";
        out.message = "no position from the camera";
        return out;
      }
      continue;
    }
    seq = s.seq;
    out.state = det.update(s.t, s.pose);
    out.last = s.pose;
    Feedback fb;
    fb.pan = s.pose.pan;
    fb.tilt = s.pose.tilt;
    fb.zoom = s.pose.zoom;
    fb.pan_error = det.error().pan;
    fb.tilt_error = det.error().tilt;
    fb.zoom_error = det.error().zoom;
    server.publishFeedback(fb);
    if (out.state != ArrivalDetector::MOVING) {
      out.label = toString(out.state);
      char buf[160];
      std::snprintf(buf, sizeof(buf), "%s at pan %.3f tilt %.3f zoom %.2f (error %.3f, %.3f)",
        out.label.c_str(), s.pose.pan, s.pose.tilt, s.pose.zoom, det.error().pan,
        det.error().tilt);
      out.message = buf;
      return out;
    }
  }
  out.label = "error";
  out.message = "shutting down";
  return out;
}

void PtzInterface::executeMoveTo(const MoveToGoalConstPtr & goal)
{
  MoveToResult res;
  const uint64_t token = ++motion_token_;
  PtzPose target;
  target.pan = wrap360(goal->pan);
  target.tilt = goal->tilt;
  target.zoom = goal->zoom;
  std::string err;
  if (!std::isfinite(goal->pan) || !std::isfinite(goal->tilt) || !std::isfinite(goal->zoom)) {
    res.state = "error";
    res.message = "pan, tilt and zoom must be finite";
    move_server_->setAborted(res, res.message);
    return;
  }
  if (!cam_->moveAbsolute(target, goal->speed, &err)) {
    res.state = "error";
    res.message = err;
    move_server_->setAborted(res, err);
    return;
  }
  ArrivalConfig cfg = arrivalConfig(goal->tolerance, goal->timeout);
  cfg.check_zoom = goal->zoom > 0.0;
  const Outcome o = track<actionlib::SimpleActionServer<MoveToAction>, MoveToFeedback>(
    *move_server_, target, true, cfg, token);
  res.reached = o.state == ArrivalDetector::REACHED;
  res.state = o.label;
  res.pan = o.last.pan;
  res.tilt = o.last.tilt;
  res.zoom = o.last.zoom;
  res.message = o.message;
  if (res.reached) {
    move_server_->setSucceeded(res, res.message);
  } else if (o.label == "preempted") {
    move_server_->setPreempted(res, res.message);
  } else {
    move_server_->setAborted(res, res.message);
  }
}

void PtzInterface::executeGotoPreset(const GotoPresetGoalConstPtr & goal)
{
  GotoPresetResult res;
  const uint64_t token = ++motion_token_;
  std::string err;
  PtzPose target;
  bool has_target = false;
  if (const auto list = cam_->presets(&err)) {
    for (const auto & p : *list) {
      if (p.id == goal->id && p.has_position) {
        target = p.pose;
        has_target = true;
      }
    }
  }
  if (!cam_->gotoPreset(goal->id, &err)) {
    res.state = "error";
    res.message = err;
    preset_server_->setAborted(res, err);
    return;
  }
  // presets are listed with 0.1 deg resolution
  ArrivalConfig cfg = arrivalConfig(std::max(goal->tolerance, 0.0), goal->timeout);
  cfg.pan_tolerance = std::max(cfg.pan_tolerance, 0.15);
  cfg.tilt_tolerance = std::max(cfg.tilt_tolerance, 0.15);
  cfg.check_zoom = has_target && target.zoom > 0.0;
  const Outcome o = track<actionlib::SimpleActionServer<GotoPresetAction>, GotoPresetFeedback>(
    *preset_server_, target, has_target, cfg, token);
  res.reached = o.state == ArrivalDetector::REACHED;
  res.state = o.label;
  res.pan = o.last.pan;
  res.tilt = o.last.tilt;
  res.zoom = o.last.zoom;
  res.message = o.message + (has_target ? "" : " (preset position unknown, waited for stop)");
  if (res.reached) {
    preset_server_->setSucceeded(res, res.message);
  } else if (o.label == "preempted") {
    preset_server_->setPreempted(res, res.message);
  } else {
    preset_server_->setAborted(res, res.message);
  }
}

// ---------------------------------------------------------------------------- manual control

void PtzInterface::onCmdVel(const geometry_msgs::Twist::ConstPtr & msg)
{
  const int pan = speed(msg->angular.z, max_speed_, invert_pan_);
  const int tilt = speed(msg->angular.y, max_speed_, invert_tilt_);
  const int zoom = speed(msg->linear.z, max_speed_, invert_zoom_);
  std::lock_guard<std::mutex> lk(cmd_mtx_);
  last_cmd_t_ = now();
  const bool moving = pan != 0 || tilt != 0 || zoom != 0;
  if (moving) {
    ++motion_token_;   // manual control wins over a running goal
  }
  if (pan == last_pan_ && tilt == last_tilt_ && zoom == last_zoom_ && moving == cmd_moving_) {
    return;   // the camera keeps the last continuous command
  }
  std::string err;
  if (!cam_->continuous(pan, tilt, zoom, &err)) {
    ROS_WARN_THROTTLE(2.0, "cmd_vel: %s", err.c_str());
    return;
  }
  last_pan_ = pan;
  last_tilt_ = tilt;
  last_zoom_ = zoom;
  cmd_moving_ = moving;
}

void PtzInterface::onWatchdog(const ros::TimerEvent &)
{
  std::lock_guard<std::mutex> lk(cmd_mtx_);
  if (cmd_moving_ && now() - last_cmd_t_ > cmd_vel_timeout_) {
    std::string err;
    if (cam_->stop(&err)) {
      ROS_INFO("no cmd_vel for %.2f s, PTZ stopped", cmd_vel_timeout_);
      cmd_moving_ = false;
      last_pan_ = last_tilt_ = last_zoom_ = 0;
    } else {
      ROS_WARN_THROTTLE(2.0, "cmd_vel watchdog: %s", err.c_str());
    }
  }
}

bool PtzInterface::onStop(std_srvs::Trigger::Request &, std_srvs::Trigger::Response & res)
{
  ++motion_token_;
  std::string err;
  res.success = cam_->stop(&err);
  res.message = res.success ? "stopped" : err;
  std::lock_guard<std::mutex> lk(cmd_mtx_);
  cmd_moving_ = false;
  last_pan_ = last_tilt_ = last_zoom_ = 0;
  return true;
}

bool PtzInterface::onFocus(std_srvs::Trigger::Request &, std_srvs::Trigger::Response & res)
{
  std::string err;
  res.success = cam_->onePushFocus(&err);
  res.message = res.success ? "one-push focus started" : err;
  return true;
}

bool PtzInterface::onSetPreset(SetPreset::Request & req, SetPreset::Response & res)
{
  std::string err;
  if (req.id < 1) {
    res.message = "preset id must be >= 1";
    return true;
  }
  res.success = cam_->setPreset(req.id, req.name, &err);
  res.message = res.success ? "saved the current position as preset " + std::to_string(req.id) :
    err;
  return true;
}

bool PtzInterface::onRemovePreset(SetPreset::Request & req, SetPreset::Response & res)
{
  std::string err;
  res.success = cam_->removePreset(req.id, &err);
  res.message = res.success ? "removed preset " + std::to_string(req.id) : err;
  return true;
}

bool PtzInterface::onGetPresets(GetPresets::Request &, GetPresets::Response & res)
{
  std::string err;
  const auto list = cam_->presets(&err);
  res.success = list.has_value();
  if (!list) {
    res.message = err;
    return true;
  }
  for (const auto & p : *list) {
    PtzPreset m;
    m.id = p.id;
    m.name = p.name;
    m.has_position = p.has_position;
    m.pan = p.pose.pan;
    m.tilt = p.pose.tilt;
    m.zoom = p.pose.zoom;
    res.presets.push_back(m);
  }
  res.message = std::to_string(res.presets.size()) + " presets";
  return true;
}

}  // namespace hikmicro_ptz_driver
