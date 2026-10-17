// ISAPI message builders and parsers for the PT series thermography cameras
// (ISAPI "Thermography thermal cameras PT series"). Pure functions, no I/O, no ROS.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "hikmicro_ptz_driver/types.hpp"

namespace hikmicro_ptz_driver
{
namespace isapi
{

// --- PTZ ------------------------------------------------------------------------------------

// High-accuracy absolute position, PUT /ISAPI/PTZCtrl/channels/<ID>/absoluteEx:
// elevation/azimuth in degrees (step 0.001), absoluteZoom as a ratio. zoom <= 0 leaves the
// zoom alone; speed > 0 adds horizontalSpeed/verticalSpeed (device units).
std::string absoluteExXml(const PtzPose & target, double speed = 0.0);

// Basic absolute position, PUT /ISAPI/PTZCtrl/channels/<ID>/absolute: AbsoluteHigh in tenths
// of a degree (elevation -900..2700, azimuth 0..3600) and tenths of a zoom ratio (10 = 1x).
std::string absoluteHighXml(const PtzPose & target);

// PUT /ISAPI/PTZCtrl/channels/<ID>/continuous, speeds -100..100 (0 stops).
std::string continuousXml(int pan, int tilt, int zoom);

// GET /ISAPI/PTZCtrl/channels/<ID>/absoluteEx
std::optional<PtzPose> parseAbsoluteEx(const std::string & xml);

// GET /ISAPI/PTZCtrl/channels/<ID>/status (AbsoluteHigh, tenths)
std::optional<PtzPose> parsePtzStatus(const std::string & xml);

// --- presets --------------------------------------------------------------------------------

// PUT /ISAPI/PTZCtrl/channels/<ID>/presets/<presetID>: saves the current position.
std::string presetXml(int id, const std::string & name);

// GET /ISAPI/PTZCtrl/channels/<ID>/presets
std::vector<Preset> parsePresetList(const std::string & xml);

// --- auxiliaries, image ---------------------------------------------------------------------

// PUT /ISAPI/PTZCtrl/channels/<ID>/auxcontrols/<auxID>; type LIGHT, WIPER, FAN or HEATER.
std::string auxXml(int id, const std::string & type, bool on);

// GET /ISAPI/PTZCtrl/channels/<ID>/auxcontrols
std::vector<AuxDevice> parseAuxList(const std::string & xml);

// PUT /ISAPI/Image/channels/<ID>/ircutFilter with only IrcutFilterType (auto, day, night, ...).
std::string ircutXml(const std::string & mode);

// GET /ISAPI/Image/channels/<ID>/ircutFilter: IrcutFilterType.
std::optional<std::string> parseIrcutMode(const std::string & xml);

// --- system ---------------------------------------------------------------------------------

// GET /ISAPI/System/deviceInfo
std::optional<DeviceInfo> parseDeviceInfo(const std::string & xml);

// ResponseStatus (XML) or the JSON error form ({"statusCode": 4, "subStatusCode": ...}).
IsapiStatus parseStatus(const std::string & body);

// Human-readable error for a failed request: "HTTP 403: Invalid Operation (notSupport)".
std::string describeError(long http_status, const std::string & body);

// Replaces the text of the first <element> anywhere in `xml`; nullopt if not found / bad XML.
std::optional<std::string> setElementText(
  const std::string & xml, const std::string & element, const std::string & text);

}  // namespace isapi
}  // namespace hikmicro_ptz_driver
