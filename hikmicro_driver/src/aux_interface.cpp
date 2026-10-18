#include "hikmicro_driver/aux_interface.hpp"

#include <algorithm>
#include <cctype>

namespace hikmicro_driver
{

namespace
{

bool looksBinary(const std::string & ct, const std::string & body)
{
  if (ct.find("image/") != std::string::npos || ct.find("octet-stream") != std::string::npos ||
    ct.find("multipart") != std::string::npos)
  {
    return true;
  }
  const size_t n = std::min<size_t>(body.size(), 512);
  for (size_t i = 0; i < n; ++i) {
    if (body[i] == '\0') {
      return true;
    }
  }
  return false;
}

}  // namespace

AuxInterface::AuxInterface(
  ros::NodeHandle & nh, ros::NodeHandle & pnh, std::shared_ptr<HikCamera> camera)
: cam_(std::move(camera))
{
  pnh.param<std::string>("visible_frame_id", visible_frame_, "hikmicro_visible_optical_frame");
  pnh.param<std::string>("thermal_frame_id", thermal_frame_, "hikmicro_thermal_optical_frame");
  light_srv_ = nh.advertiseService("aux/light", &AuxInterface::onLight, this);
  wiper_srv_ = nh.advertiseService("aux/wiper", &AuxInterface::onWiper, this);
  aux_srv_ = nh.advertiseService("aux/set", &AuxInterface::onAux, this);
  day_night_srv_ = nh.advertiseService("image/day_night", &AuxInterface::onDayNight, this);
  snapshot_srv_ = nh.advertiseService("snapshot", &AuxInterface::onSnapshot, this);
  raw_srv_ = nh.advertiseService("isapi/request", &AuxInterface::onRaw, this);
}

bool AuxInterface::onLight(std_srvs::SetBool::Request & req, std_srvs::SetBool::Response & res)
{
  std::string err;
  res.success = cam_->setAux("LIGHT", req.data, &err);
  res.message = res.success ? std::string("light ") + (req.data ? "on" : "off") : err;
  return true;
}

bool AuxInterface::onWiper(std_srvs::SetBool::Request & req, std_srvs::SetBool::Response & res)
{
  std::string err;
  res.success = cam_->setAux("WIPER", req.data, &err);
  res.message = res.success ? std::string("wiper ") + (req.data ? "on" : "off") : err;
  return true;
}

bool AuxInterface::onAux(SetAux::Request & req, SetAux::Response & res)
{
  std::string err;
  res.success = cam_->setAux(req.type, req.enabled, &err);
  res.message = res.success ? req.type + (req.enabled ? " on" : " off") : err;
  return true;
}

bool AuxInterface::onDayNight(SetDayNight::Request & req, SetDayNight::Response & res)
{
  std::string err;
  std::string mode = req.mode;
  std::transform(mode.begin(), mode.end(), mode.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  if (!mode.empty() && !cam_->setDayNight(mode, &err)) {
    res.message = err;
    return true;
  }
  const auto now = cam_->dayNight(&err);
  res.success = now.has_value();
  res.mode = now.value_or("");
  res.message = res.success ? "day/night mode " + res.mode : err;
  return true;
}

bool AuxInterface::onSnapshot(Snapshot::Request & req, Snapshot::Response & res)
{
  std::string err;
  const bool thermal = req.channel == Snapshot::Request::THERMAL;
  const int channel = thermal ? cam_->config().thermal_channel : cam_->config().visible_channel;
  const ros::Time stamp = ros::Time::now();
  const auto jpg = cam_->snapshot(channel, &err);
  res.success = jpg.has_value();
  if (!jpg) {
    res.message = err;
    return true;
  }
  res.image.header.stamp = stamp;
  res.image.header.frame_id = thermal ? thermal_frame_ : visible_frame_;
  res.image.format = "jpeg";
  res.image.data.assign(jpg->begin(), jpg->end());
  res.message = std::to_string(jpg->size()) + " bytes";
  return true;
}

bool AuxInterface::onRaw(RawRequest::Request & req, RawRequest::Response & res)
{
  std::string method = req.method.empty() ? "GET" : req.method;
  std::transform(method.begin(), method.end(), method.begin(),
    [](unsigned char c) {return static_cast<char>(std::toupper(c));});
  if (req.path.rfind("/ISAPI/", 0) != 0 && req.path.rfind("/SDK/", 0) != 0) {
    res.message = "path must start with /ISAPI/ or /SDK/";
    return true;
  }
  const auto r = cam_->raw(method, req.path, req.body, req.content_type);
  res.status = static_cast<int32_t>(r.status);
  res.content_type = r.content_type;
  if (!r.error.empty()) {
    res.message = r.error;
  } else if (looksBinary(r.content_type, r.body)) {
    res.message = "binary answer, " + std::to_string(r.body.size()) + " bytes";
  } else {
    res.body = r.body;
    res.message = r.ok() ? "ok" : "HTTP " + std::to_string(r.status);
  }
  return true;
}

}  // namespace hikmicro_driver
