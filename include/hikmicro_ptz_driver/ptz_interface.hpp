// ROS side of the PTZ: state polling, the move_to / goto_preset actions (finish when the camera
// has arrived), presets, cmd_vel with a watchdog, stop and one-push focus.
#pragma once

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "hikmicro_ptz_driver/GetPresets.h"
#include "hikmicro_ptz_driver/GotoPresetAction.h"
#include "hikmicro_ptz_driver/MoveToAction.h"
#include "hikmicro_ptz_driver/SetPreset.h"
#include "hikmicro_ptz_driver/camera.hpp"
#include "hikmicro_ptz_driver/motion.hpp"

namespace hikmicro_ptz_driver
{

class PtzInterface
{
public:
  PtzInterface(ros::NodeHandle & nh, ros::NodeHandle & pnh, std::shared_ptr<HikCamera> camera);
  ~PtzInterface();

  PtzInterface(const PtzInterface &) = delete;
  PtzInterface & operator=(const PtzInterface &) = delete;

private:
  struct Sample
  {
    PtzPose pose;
    double t{0.0};        // steady clock, s
    uint64_t seq{0};
  };

  struct Outcome
  {
    ArrivalDetector::State state{ArrivalDetector::MOVING};
    std::string label;    // reached, stalled, timeout, preempted, superseded, error
    PtzPose last;
    std::string message;
  };

  void stateLoop();
  bool waitSample(uint64_t after_seq, double timeout_sec, Sample * out);

  // Runs the arrival detector on fresh samples until it decides, the goal is preempted or a
  // newer motion command supersedes it. `feedback(pose, error)` is called on every sample.
  template<typename Server, typename Feedback>
  Outcome track(Server & server, const PtzPose & target, bool has_target,
    const ArrivalConfig & cfg, uint64_t token);

  void executeMoveTo(const MoveToGoalConstPtr & goal);
  void executeGotoPreset(const GotoPresetGoalConstPtr & goal);
  ArrivalConfig arrivalConfig(double tolerance, double timeout) const;

  void onCmdVel(const geometry_msgs::Twist::ConstPtr & msg);
  void onWatchdog(const ros::TimerEvent &);
  bool onStop(std_srvs::Trigger::Request &, std_srvs::Trigger::Response & res);
  bool onFocus(std_srvs::Trigger::Request &, std_srvs::Trigger::Response & res);
  bool onSetPreset(SetPreset::Request & req, SetPreset::Response & res);
  bool onRemovePreset(SetPreset::Request & req, SetPreset::Response & res);
  bool onGetPresets(GetPresets::Request & req, GetPresets::Response & res);

  static double now();

  std::shared_ptr<HikCamera> cam_;

  // parameters
  double state_rate_{10.0};
  double pan_tol_{0.2}, tilt_tol_{0.2}, zoom_tol_{0.1};
  double settle_{0.3}, stall_{2.0}, move_timeout_{30.0};
  double cmd_vel_timeout_{0.5};
  int max_speed_{60};
  bool invert_pan_{false}, invert_tilt_{false}, invert_zoom_{false};
  std::string frame_id_;
  bool joint_states_{true};
  std::string pan_joint_, tilt_joint_;
  double pan_sign_{1.0}, tilt_sign_{1.0};

  ros::Publisher state_pub_;
  ros::Publisher joint_pub_;
  ros::Subscriber cmd_vel_sub_;
  ros::ServiceServer stop_srv_, focus_srv_, set_preset_srv_, remove_preset_srv_, presets_srv_;
  ros::Timer watchdog_;
  std::unique_ptr<actionlib::SimpleActionServer<MoveToAction>> move_server_;
  std::unique_ptr<actionlib::SimpleActionServer<GotoPresetAction>> preset_server_;

  // state polling
  std::atomic<bool> running_{true};
  std::atomic<bool> connected_{false};
  std::thread state_thread_;
  std::mutex sample_mtx_;
  std::condition_variable sample_cv_;
  Sample sample_;

  // every new motion command (goal, cmd_vel, stop) bumps the token; older goals give up
  std::atomic<uint64_t> motion_token_{0};

  std::mutex cmd_mtx_;
  bool cmd_moving_{false};
  int last_pan_{0}, last_tilt_{0}, last_zoom_{0};
  double last_cmd_t_{0.0};
};

}  // namespace hikmicro_ptz_driver
