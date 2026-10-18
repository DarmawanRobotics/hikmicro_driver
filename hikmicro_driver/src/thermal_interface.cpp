#include "hikmicro_driver/thermal_interface.hpp"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "hikmicro_msgs/ThermalStats.h"

namespace hikmicro_driver
{

namespace
{

ThermalStats toMsg(const TemperatureStats & s, uint8_t mode, const RadiometricFrame & f,
  const ros::Time & stamp, const std::string & frame_id)
{
  ThermalStats m;
  m.header.stamp = stamp;
  m.header.frame_id = frame_id;
  m.mode = mode;
  m.width = static_cast<uint32_t>(f.width);
  m.height = static_cast<uint32_t>(f.height);
  m.x1 = static_cast<uint32_t>(s.x1);
  m.y1 = static_cast<uint32_t>(s.y1);
  m.x2 = static_cast<uint32_t>(s.x2);
  m.y2 = static_cast<uint32_t>(s.y2);
  m.max = s.max;
  m.min = s.min;
  m.mean = s.mean;
  m.max_x = static_cast<uint32_t>(s.max_x);
  m.max_y = static_cast<uint32_t>(s.max_y);
  m.min_x = static_cast<uint32_t>(s.min_x);
  m.min_y = static_cast<uint32_t>(s.min_y);
  return m;
}

}  // namespace

ThermalInterface::ThermalInterface(
  ros::NodeHandle & nh, ros::NodeHandle & pnh, std::shared_ptr<HikCamera> camera)
: cam_(std::move(camera))
{
  pnh.param("radiometric_rate", rate_, 1.0);
  pnh.param("radiometric_max_age", max_age_, 2.0);
  pnh.param<std::string>("thermal_frame_id", frame_id_, "hikmicro_thermal_optical_frame");

  image_pub_ = nh.advertise<sensor_msgs::Image>("thermal/temperature", 1);
  stats_pub_ = nh.advertise<ThermalStats>("thermal/stats", 10);
  temperature_srv_ =
    nh.advertiseService("thermal/get_temperature", &ThermalInterface::onGetTemperature, this);
  thread_ = std::thread(&ThermalInterface::loop, this);
}

ThermalInterface::~ThermalInterface()
{
  running_ = false;
  wake_cv_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
}

std::optional<ThermalInterface::Frame> ThermalInterface::fetch(std::string * err)
{
  std::lock_guard<std::mutex> lk(fetch_mtx_);
  const ros::Time stamp = ros::Time::now();
  auto data = cam_->radiometric(err);
  if (!data) {
    return std::nullopt;
  }
  Frame f;
  f.data = std::make_shared<const RadiometricFrame>(std::move(*data));
  f.stamp = stamp;
  f.taken = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lk2(frame_mtx_);
  latest_ = f;
  return f;
}

std::optional<ThermalInterface::Frame> ThermalInterface::frame(double max_age, std::string * err)
{
  {
    std::lock_guard<std::mutex> lk(frame_mtx_);
    if (latest_ && std::chrono::duration<double>(std::chrono::steady_clock::now() -
      latest_->taken).count() <= max_age)
    {
      return latest_;
    }
  }
  return fetch(err);
}

void ThermalInterface::publish(const Frame & f)
{
  const RadiometricFrame & d = *f.data;
  if (const auto s = regionStats(d, 0, 0, d.width - 1, d.height - 1)) {
    stats_pub_.publish(toMsg(*s, ThermalStats::FRAME, d, f.stamp, frame_id_));
  }
  if (image_pub_.getNumSubscribers() == 0) {
    return;
  }
  sensor_msgs::Image img;
  img.header.stamp = f.stamp;
  img.header.frame_id = frame_id_;
  img.width = static_cast<uint32_t>(d.width);
  img.height = static_cast<uint32_t>(d.height);
  img.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  img.is_bigendian = 0;
  img.step = img.width * sizeof(float);
  img.data.resize(img.step * img.height);
  std::memcpy(img.data.data(), d.celsius.data(), img.data.size());
  image_pub_.publish(img);
}

void ThermalInterface::loop()
{
  if (rate_ <= 0.0) {
    return;  // on request only
  }
  const auto period = std::chrono::duration<double>(1.0 / rate_);
  double backoff = 0.0;
  while (running_ && ros::ok()) {
    const auto start = std::chrono::steady_clock::now();
    std::string err;
    if (const auto f = fetch(&err)) {
      publish(*f);
      backoff = 0.0;
    } else {
      // e.g. "notSupport" on firmware without pixel-to-pixel thermometry
      backoff = std::min(30.0, backoff > 0.0 ? backoff * 2.0 : 2.0);
      ROS_WARN_THROTTLE(30.0, "radiometric frame: %s (retrying in %.0f s)", err.c_str(), backoff);
    }
    const auto wait = backoff > 0.0 ? std::chrono::duration<double>(backoff) : period;
    std::unique_lock<std::mutex> lk(wake_mtx_);
    wake_cv_.wait_until(lk, start + std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(wait), [this] {return !running_.load();});
  }
}

bool ThermalInterface::onGetTemperature(
  GetTemperature::Request & req, GetTemperature::Response & res)
{
  std::string err;
  const auto f = frame(req.max_age > 0.0f ? req.max_age : max_age_, &err);
  if (!f) {
    res.message = err;
    return true;
  }
  const RadiometricFrame & d = *f->data;
  const auto px = [&](float v, int size) {
      const double p = req.normalized ? v * (size - 1) : v;
      return static_cast<int>(std::lround(p));
    };
  int x1 = 0, y1 = 0, x2 = d.width - 1, y2 = d.height - 1;
  switch (req.mode) {
    case GetTemperature::Request::POINT:
      x1 = x2 = px(req.x, d.width);
      y1 = y2 = px(req.y, d.height);
      break;
    case GetTemperature::Request::REGION:
      x1 = px(req.x, d.width);
      y1 = px(req.y, d.height);
      x2 = px(req.x2, d.width);
      y2 = px(req.y2, d.height);
      break;
    case GetTemperature::Request::FRAME:
      break;
    default:
      res.message = "mode must be POINT (0), REGION (1) or FRAME (2)";
      return true;
  }
  if (req.mode != GetTemperature::Request::FRAME &&
    (std::max(x1, x2) < 0 || std::min(x1, x2) >= d.width || std::max(y1, y2) < 0 ||
    std::min(y1, y2) >= d.height))
  {
    res.message = "outside the " + std::to_string(d.width) + "x" + std::to_string(d.height) +
      " frame";
    return true;
  }
  const auto s = regionStats(d, x1, y1, x2, y2);
  if (!s) {
    res.message = "invalid frame";
    return true;
  }
  res.success = true;
  res.stats = toMsg(*s, req.mode, d, f->stamp, frame_id_);
  char buf[128];
  std::snprintf(buf, sizeof(buf), "max %.2f C at (%d, %d), min %.2f C, mean %.2f C", s->max,
    s->max_x, s->max_y, s->min, s->mean);
  res.message = buf;
  return true;
}

}  // namespace hikmicro_driver
