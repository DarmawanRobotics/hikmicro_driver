// HIKMICRO / Hikvision PT series bi-spectrum camera over ISAPI: PTZ (high-accuracy absolute
// position with fallback to the basic one), presets, auxiliaries (light, wiper), day/night,
// one-push focus, JPEG snapshots and pixel-to-pixel thermometry. Thread-safe, no ROS.
#pragma once

#include <atomic>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "hikmicro_ptz_driver/http_client.hpp"
#include "hikmicro_ptz_driver/radiometric.hpp"
#include "hikmicro_ptz_driver/types.hpp"

namespace hikmicro_ptz_driver
{

struct CameraConfig
{
  std::string host{"192.168.1.64"};
  int port{80};
  bool https{false};
  std::string user{"admin"};
  std::string password;
  int ptz_channel{1};
  int visible_channel{1};
  int thermal_channel{2};
  double timeout{3.0};               // s, control requests
  double image_timeout{10.0};        // s, snapshots and radiometric frames
  std::map<std::string, int> aux_ids;  // type -> id override (LIGHT, WIPER, ...); empty: ask
};

class HikCamera
{
public:
  explicit HikCamera(CameraConfig config);

  const CameraConfig & config() const {return cfg_;}

  // --- device ---
  std::optional<DeviceInfo> deviceInfo(std::string * err = nullptr);

  // Tries GET .../absoluteEx; on success positions are read and commanded with 0.001 deg
  // resolution, otherwise through /status and /absolute (0.1 deg).
  bool probeHighPrecision();
  bool highPrecision() const {return high_precision_.load();}

  // --- PTZ ---
  std::optional<PtzPose> position(std::string * err = nullptr);
  // zoom <= 0 keeps the current zoom. speed > 0 only with high precision (device units).
  bool moveAbsolute(const PtzPose & target, double speed = 0.0, std::string * err = nullptr);
  bool continuous(int pan, int tilt, int zoom, std::string * err = nullptr);
  bool stop(std::string * err = nullptr) {return continuous(0, 0, 0, err);}
  bool onePushFocus(std::string * err = nullptr);

  std::optional<std::vector<Preset>> presets(std::string * err = nullptr);
  bool setPreset(int id, const std::string & name, std::string * err = nullptr);
  bool gotoPreset(int id, std::string * err = nullptr);
  bool removePreset(int id, std::string * err = nullptr);

  // --- auxiliaries and image ---
  std::optional<std::vector<AuxDevice>> auxDevices(std::string * err = nullptr);
  // type: LIGHT, WIPER, FAN, HEATER (case-insensitive).
  bool setAux(const std::string & type, bool on, std::string * err = nullptr);
  std::optional<std::string> dayNight(std::string * err = nullptr);
  bool setDayNight(const std::string & mode, std::string * err = nullptr);

  // --- images ---
  // JPEG from /ISAPI/Streaming/channels/<channel>01/picture.
  std::optional<std::string> snapshot(int channel, std::string * err = nullptr);
  std::optional<RadiometricFrame> radiometric(std::string * err = nullptr);

  // --- anything else ---
  HttpResponse raw(const std::string & method, const std::string & path,
    const std::string & body = "", const std::string & content_type = "");

  // rtsp://user:pass@host:554/Streaming/Channels/<channel>0<stream>
  std::string rtspUrl(int channel, int stream = 1, int rtsp_port = 554) const;

private:
  bool check(const HttpResponse & r, std::string * err, const std::string & what);
  std::string ptzPath(const std::string & leaf) const;

  CameraConfig cfg_;
  HttpClient http_;
  std::atomic<bool> high_precision_{false};
  std::mutex aux_mtx_;
  std::map<std::string, int> aux_cache_;
};

}  // namespace hikmicro_ptz_driver
