// ROS side of the pixel-to-pixel thermometry: a temperature image (32FC1, degrees Celsius) and
// full-frame statistics at radiometric_rate, and temperatures at a point / in a box on request.
#pragma once

#include <ros/ros.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "hikmicro_msgs/GetTemperature.h"
#include "hikmicro_driver/camera.hpp"

namespace hikmicro_driver
{

using namespace hikmicro_msgs;   // messages, services, actions

class ThermalInterface
{
public:
  ThermalInterface(ros::NodeHandle & nh, ros::NodeHandle & pnh, std::shared_ptr<HikCamera> camera);
  ~ThermalInterface();

  ThermalInterface(const ThermalInterface &) = delete;
  ThermalInterface & operator=(const ThermalInterface &) = delete;

private:
  struct Frame
  {
    std::shared_ptr<const RadiometricFrame> data;
    ros::Time stamp;
    std::chrono::steady_clock::time_point taken;
  };

  void loop();
  // Latest frame if newer than max_age, else a fresh one. nullopt + err on failure.
  std::optional<Frame> frame(double max_age, std::string * err);
  std::optional<Frame> fetch(std::string * err);
  void publish(const Frame & f);
  bool onGetTemperature(GetTemperature::Request & req, GetTemperature::Response & res);

  std::shared_ptr<HikCamera> cam_;
  double rate_{1.0};
  double max_age_{2.0};
  std::string frame_id_;

  ros::Publisher image_pub_;
  ros::Publisher stats_pub_;
  ros::ServiceServer temperature_srv_;

  std::mutex fetch_mtx_;   // one request at a time: frames are ~1.5 MB
  std::mutex frame_mtx_;
  std::optional<Frame> latest_;

  std::atomic<bool> running_{true};
  std::mutex wake_mtx_;
  std::condition_variable wake_cv_;
  std::thread thread_;
};

}  // namespace hikmicro_driver
