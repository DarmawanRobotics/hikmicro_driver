// Plain data types shared by the ISAPI client and the ROS nodes. No ROS here.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace hikmicro_driver
{

// PTZ position as the camera reports it: azimuth (pan) 0..360 deg, elevation (tilt) in deg,
// zoom as a ratio (1.0 = 1x). Signs follow the device, see README "Verify on the real camera".
struct PtzPose
{
  double pan{0.0};
  double tilt{0.0};
  double zoom{0.0};
};

struct Preset
{
  int id{0};
  std::string name;
  bool enabled{true};
  bool has_position{false};
  PtzPose pose;
};

// PTZ auxiliaries (/ISAPI/PTZCtrl/channels/<ID>/auxcontrols): LIGHT, WIPER, FAN, HEATER.
struct AuxDevice
{
  int id{0};
  std::string type;
  bool on{false};
};

struct DeviceInfo
{
  std::string name;
  std::string model;
  std::string serial;
  std::string firmware;
  std::string firmware_date;
  std::string mac;
  std::string type;
};

// ISAPI ResponseStatus (XML) or JSON error body.
struct IsapiStatus
{
  bool parsed{false};
  int status_code{0};          // 0/1 OK, 2 busy, 3 device error, 4 invalid operation, ...
  std::string status_string;
  std::string sub_status_code;
  std::string error_msg;
  bool ok() const {return status_code == 0 || status_code == 1;}
};

}  // namespace hikmicro_driver
