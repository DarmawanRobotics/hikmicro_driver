#include "hikmicro_ptz_driver/camera.hpp"

#include <algorithm>
#include <cctype>

#include "hikmicro_ptz_driver/isapi_xml.hpp"

namespace hikmicro_ptz_driver
{

namespace
{

HttpClient::Options httpOptions(const CameraConfig & c)
{
  HttpClient::Options o;
  o.base_url = std::string(c.https ? "https://" : "http://") + c.host + ":" +
    std::to_string(c.port);
  o.user = c.user;
  o.password = c.password;
  o.timeout = c.timeout;
  return o;
}

std::string upper(std::string s)
{
  for (auto & ch : s) {ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));}
  return s;
}

void setErr(std::string * err, const std::string & msg)
{
  if (err) {
    *err = msg;
  }
}

}  // namespace

HikCamera::HikCamera(CameraConfig config)
: cfg_(std::move(config)), http_(httpOptions(cfg_))
{
  for (const auto & kv : cfg_.aux_ids) {
    if (kv.second > 0) {
      aux_cache_[upper(kv.first)] = kv.second;
    }
  }
}

std::string HikCamera::ptzPath(const std::string & leaf) const
{
  return "/ISAPI/PTZCtrl/channels/" + std::to_string(cfg_.ptz_channel) + "/" + leaf;
}

bool HikCamera::check(const HttpResponse & r, std::string * err, const std::string & what)
{
  if (!r.error.empty()) {
    setErr(err, what + ": " + r.error);
    return false;
  }
  if (r.status < 200 || r.status >= 300) {
    setErr(err, what + ": " + isapi::describeError(r.status, r.body));
    return false;
  }
  // Some firmware answers 200 with a failing ResponseStatus.
  const IsapiStatus s = isapi::parseStatus(r.body);
  if (s.parsed && !s.ok()) {
    setErr(err, what + ": " + isapi::describeError(r.status, r.body));
    return false;
  }
  return true;
}

std::optional<DeviceInfo> HikCamera::deviceInfo(std::string * err)
{
  const auto r = http_.get("/ISAPI/System/deviceInfo");
  if (!check(r, err, "deviceInfo")) {
    return std::nullopt;
  }
  auto d = isapi::parseDeviceInfo(r.body);
  if (!d) {
    setErr(err, "deviceInfo: unexpected answer");
  }
  return d;
}

bool HikCamera::probeHighPrecision()
{
  const auto r = http_.get(ptzPath("absoluteEx"));
  const bool ok = r.ok() && isapi::parseAbsoluteEx(r.body).has_value();
  high_precision_ = ok;
  return ok;
}

std::optional<PtzPose> HikCamera::position(std::string * err)
{
  if (high_precision_) {
    const auto r = http_.get(ptzPath("absoluteEx"));
    if (check(r, err, "absoluteEx")) {
      if (auto p = isapi::parseAbsoluteEx(r.body)) {
        return p;
      }
      setErr(err, "absoluteEx: unexpected answer");
    }
    return std::nullopt;
  }
  const auto r = http_.get(ptzPath("status"));
  if (!check(r, err, "PTZ status")) {
    return std::nullopt;
  }
  auto p = isapi::parsePtzStatus(r.body);
  if (!p) {
    setErr(err, "PTZ status: unexpected answer");
  }
  return p;
}

bool HikCamera::moveAbsolute(const PtzPose & target, double speed, std::string * err)
{
  if (high_precision_) {
    return check(http_.put(ptzPath("absoluteEx"), isapi::absoluteExXml(target, speed)), err,
             "absoluteEx");
  }
  return check(http_.put(ptzPath("absolute"), isapi::absoluteHighXml(target)), err, "absolute");
}

bool HikCamera::continuous(int pan, int tilt, int zoom, std::string * err)
{
  return check(http_.put(ptzPath("continuous"), isapi::continuousXml(pan, tilt, zoom)), err,
           "continuous");
}

bool HikCamera::onePushFocus(std::string * err)
{
  return check(http_.put(ptzPath("onepushfoucs/start")), err, "one-push focus");
}

std::optional<std::vector<Preset>> HikCamera::presets(std::string * err)
{
  const auto r = http_.get(ptzPath("presets"));
  if (!check(r, err, "presets")) {
    return std::nullopt;
  }
  return isapi::parsePresetList(r.body);
}

bool HikCamera::setPreset(int id, const std::string & name, std::string * err)
{
  const std::string n = name.empty() ? "preset " + std::to_string(id) : name;
  return check(http_.put(ptzPath("presets/" + std::to_string(id)), isapi::presetXml(id, n)), err,
           "set preset");
}

bool HikCamera::gotoPreset(int id, std::string * err)
{
  return check(http_.put(ptzPath("presets/" + std::to_string(id) + "/goto")), err, "goto preset");
}

bool HikCamera::removePreset(int id, std::string * err)
{
  return check(http_.request("DELETE", ptzPath("presets/" + std::to_string(id))), err,
           "remove preset");
}

std::optional<std::vector<AuxDevice>> HikCamera::auxDevices(std::string * err)
{
  const auto r = http_.get(ptzPath("auxcontrols"));
  if (!check(r, err, "auxcontrols")) {
    return std::nullopt;
  }
  auto list = isapi::parseAuxList(r.body);
  std::lock_guard<std::mutex> lk(aux_mtx_);
  for (const auto & a : list) {
    if (!aux_cache_.count(a.type)) {
      aux_cache_[a.type] = a.id;
    }
  }
  return list;
}

bool HikCamera::setAux(const std::string & type_in, bool on, std::string * err)
{
  const std::string type = upper(type_in);
  int id = 0;
  {
    std::lock_guard<std::mutex> lk(aux_mtx_);
    auto it = aux_cache_.find(type);
    if (it != aux_cache_.end()) {
      id = it->second;
    }
  }
  if (id == 0) {
    std::string e;
    if (!auxDevices(&e)) {
      setErr(err, e);
      return false;
    }
    std::lock_guard<std::mutex> lk(aux_mtx_);
    auto it = aux_cache_.find(type);
    if (it == aux_cache_.end()) {
      setErr(err, "the camera lists no " + type + " auxiliary");
      return false;
    }
    id = it->second;
  }
  return check(http_.put(ptzPath("auxcontrols/" + std::to_string(id)),
           isapi::auxXml(id, type, on)), err, type);
}

std::optional<std::string> HikCamera::dayNight(std::string * err)
{
  const auto r = http_.get(
    "/ISAPI/Image/channels/" + std::to_string(cfg_.visible_channel) + "/ircutFilter");
  if (!check(r, err, "ircutFilter")) {
    return std::nullopt;
  }
  auto m = isapi::parseIrcutMode(r.body);
  if (!m) {
    setErr(err, "ircutFilter: unexpected answer");
  }
  return m;
}

bool HikCamera::setDayNight(const std::string & mode, std::string * err)
{
  const std::string path =
    "/ISAPI/Image/channels/" + std::to_string(cfg_.visible_channel) + "/ircutFilter";
  std::string first;
  if (check(http_.put(path, isapi::ircutXml(mode)), &first, "ircutFilter")) {
    return true;
  }
  // Some firmware wants the whole object back: read, change the mode, write.
  const auto cur = http_.get(path);
  if (cur.ok()) {
    if (auto full = isapi::setElementText(cur.body, "IrcutFilterType", mode)) {
      if (check(http_.put(path, *full), err, "ircutFilter")) {
        return true;
      }
      return false;
    }
  }
  setErr(err, first);
  return false;
}

std::optional<std::string> HikCamera::snapshot(int channel, std::string * err)
{
  const auto r = http_.request("GET",
      "/ISAPI/Streaming/channels/" + std::to_string(channel * 100 + 1) + "/picture", "", "",
      cfg_.image_timeout);
  if (!check(r, err, "snapshot")) {
    return std::nullopt;
  }
  if (r.body.size() < 4 || static_cast<unsigned char>(r.body[0]) != 0xFF ||
    static_cast<unsigned char>(r.body[1]) != 0xD8)
  {
    setErr(err, "snapshot: not a JPEG (" + r.content_type + ")");
    return std::nullopt;
  }
  return r.body;
}

std::optional<RadiometricFrame> HikCamera::radiometric(std::string * err)
{
  const auto r = http_.request("GET",
      "/ISAPI/Thermal/channels/" + std::to_string(cfg_.thermal_channel) +
      "/thermometry/jpegPicWithAppendData?format=json", "", "", cfg_.image_timeout);
  if (!r.error.empty() || r.status < 200 || r.status >= 300) {
    check(r, err, "jpegPicWithAppendData");
    return std::nullopt;
  }
  std::string e;
  auto f = decodeRadiometric(r.content_type, r.body, &e);
  if (!f) {
    setErr(err, "jpegPicWithAppendData: " + e);
  }
  return f;
}

HttpResponse HikCamera::raw(
  const std::string & method, const std::string & path, const std::string & body,
  const std::string & content_type)
{
  std::string ct = content_type;
  if (ct.empty() && !body.empty()) {
    ct = body.front() == '{' ? "application/json" : "application/xml; charset=\"UTF-8\"";
  }
  return http_.request(method, path, body, ct, cfg_.image_timeout);
}

std::string HikCamera::rtspUrl(int channel, int stream, int rtsp_port) const
{
  std::string cred;
  if (!cfg_.user.empty()) {
    cred = urlEncode(cfg_.user);
    if (!cfg_.password.empty()) {
      cred += ":" + urlEncode(cfg_.password);
    }
    cred += "@";
  }
  return "rtsp://" + cred + cfg_.host + ":" + std::to_string(rtsp_port) +
         "/Streaming/Channels/" + std::to_string(channel * 100 + stream);
}

}  // namespace hikmicro_ptz_driver
