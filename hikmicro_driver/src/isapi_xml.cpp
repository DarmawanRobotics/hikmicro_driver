#include "hikmicro_driver/isapi_xml.hpp"

#include <tinyxml2.h>

#include <nlohmann/json.hpp>

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>

namespace hikmicro_driver
{
namespace isapi
{

namespace
{

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;
using tinyxml2::XMLNode;

constexpr const char * kNs = "http://www.isapi.org/ver20/XMLSchema";

std::string trim(const std::string & s)
{
  size_t b = 0, e = s.size();
  while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) {++b;}
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) {--e;}
  return s.substr(b, e - b);
}

// Text of an element, skipping comments (the document's examples carry them).
std::string text(const XMLElement * e)
{
  std::string out;
  if (!e) {
    return out;
  }
  for (const XMLNode * n = e->FirstChild(); n; n = n->NextSibling()) {
    if (const auto * t = n->ToText()) {
      out += t->Value();
    }
  }
  return trim(out);
}

const XMLElement * child(const XMLElement * e, const char * name)
{
  return e ? e->FirstChildElement(name) : nullptr;
}

std::optional<double> number(const XMLElement * e)
{
  const std::string s = text(e);
  if (s.empty()) {
    return std::nullopt;
  }
  char * end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (end == s.c_str()) {
    return std::nullopt;
  }
  return v;
}

// Root element by name; ISAPI uses a default namespace, so names have no prefix.
const XMLElement * root(XMLDocument & doc, const std::string & xml, const char * name)
{
  if (doc.Parse(xml.c_str(), xml.size()) != tinyxml2::XML_SUCCESS) {
    return nullptr;
  }
  const XMLElement * r = doc.RootElement();
  if (!r) {
    return nullptr;
  }
  if (name && std::string(r->Name()) != name) {
    return r->FirstChildElement(name);  // tolerate a wrapper element
  }
  return r;
}

std::string fmt(double v, int decimals)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.*f", decimals, v);
  return buf;
}

std::string open(const char * name)
{
  return std::string("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<") + name + " xmlns=\"" +
         kNs + "\" version=\"2.0\">";
}

const XMLElement * findAnywhere(const XMLElement * e, const char * name)
{
  for (const XMLElement * c = e ? e->FirstChildElement() : nullptr; c;
    c = c->NextSiblingElement())
  {
    if (std::string(c->Name()) == name) {
      return c;
    }
    if (const XMLElement * d = findAnywhere(c, name)) {
      return d;
    }
  }
  return nullptr;
}

std::optional<PtzPose> absoluteHigh(const XMLElement * high)
{
  const auto el = number(child(high, "elevation"));
  const auto az = number(child(high, "azimuth"));
  if (!el || !az) {
    return std::nullopt;
  }
  PtzPose p;
  p.tilt = *el / 10.0;
  p.pan = *az / 10.0;
  p.zoom = number(child(high, "absoluteZoom")).value_or(0.0) / 10.0;
  return p;
}

}  // namespace

std::string absoluteExXml(const PtzPose & target, double speed)
{
  std::ostringstream x;
  x << open("PTZAbsoluteEx")
    << "<elevation>" << fmt(target.tilt, 3) << "</elevation>"
    << "<azimuth>" << fmt(target.pan, 3) << "</azimuth>";
  if (target.zoom > 0.0) {
    x << "<absoluteZoom>" << fmt(target.zoom, 2) << "</absoluteZoom>";
  }
  if (speed > 0.0) {
    x << "<horizontalSpeed>" << fmt(speed, 2) << "</horizontalSpeed>"
      << "<verticalSpeed>" << fmt(speed, 2) << "</verticalSpeed>";
  }
  x << "</PTZAbsoluteEx>";
  return x.str();
}

std::string absoluteHighXml(const PtzPose & target)
{
  std::ostringstream x;
  x << open("PTZData") << "<AbsoluteHigh>"
    << "<elevation>" << std::lround(target.tilt * 10.0) << "</elevation>"
    << "<azimuth>" << std::lround(target.pan * 10.0) << "</azimuth>";
  if (target.zoom > 0.0) {
    x << "<absoluteZoom>" << std::lround(target.zoom * 10.0) << "</absoluteZoom>";
  }
  x << "</AbsoluteHigh></PTZData>";
  return x.str();
}

std::string continuousXml(int pan, int tilt, int zoom)
{
  const auto clamp = [](int v) {return v < -100 ? -100 : (v > 100 ? 100 : v);};
  std::ostringstream x;
  x << open("PTZData") << "<pan>" << clamp(pan) << "</pan><tilt>" << clamp(tilt)
    << "</tilt><zoom>" << clamp(zoom) << "</zoom></PTZData>";
  return x.str();
}

std::optional<PtzPose> parseAbsoluteEx(const std::string & xml)
{
  XMLDocument doc;
  const XMLElement * r = root(doc, xml, "PTZAbsoluteEx");
  const auto el = number(child(r, "elevation"));
  const auto az = number(child(r, "azimuth"));
  if (!el || !az) {
    return std::nullopt;
  }
  PtzPose p;
  p.tilt = *el;
  p.pan = *az;
  p.zoom = number(child(r, "absoluteZoom")).value_or(0.0);
  return p;
}

std::optional<PtzPose> parsePtzStatus(const std::string & xml)
{
  XMLDocument doc;
  const XMLElement * r = root(doc, xml, "PTZStatus");
  return absoluteHigh(child(r, "AbsoluteHigh"));
}

std::string presetXml(int id, const std::string & name)
{
  tinyxml2::XMLPrinter printer(nullptr, true);  // escapes the name
  printer.PushText(name.c_str());
  std::ostringstream x;
  x << open("PTZPreset") << "<enabled>true</enabled><id>" << id << "</id><presetName>"
    << printer.CStr() << "</presetName></PTZPreset>";
  return x.str();
}

std::vector<Preset> parsePresetList(const std::string & xml)
{
  std::vector<Preset> out;
  XMLDocument doc;
  const XMLElement * r = root(doc, xml, "PTZPresetList");
  for (const XMLElement * e = r ? r->FirstChildElement("PTZPreset") : nullptr; e;
    e = e->NextSiblingElement("PTZPreset"))
  {
    const auto id = number(child(e, "id"));
    if (!id) {
      continue;  // ids are strings in the schema; only numeric ones can be called
    }
    Preset p;
    p.id = static_cast<int>(*id);
    p.name = text(child(e, "presetName"));
    p.enabled = text(child(e, "enabled")) != "false";
    if (auto pose = absoluteHigh(child(e, "AbsoluteHigh"))) {
      p.has_position = true;
      p.pose = *pose;
    }
    out.push_back(p);
  }
  return out;
}

std::string auxXml(int id, const std::string & type, bool on)
{
  std::ostringstream x;
  x << open("PTZAux") << "<id>" << id << "</id><type>" << type << "</type><status>"
    << (on ? "on" : "off") << "</status></PTZAux>";
  return x.str();
}

std::vector<AuxDevice> parseAuxList(const std::string & xml)
{
  std::vector<AuxDevice> out;
  XMLDocument doc;
  const XMLElement * r = root(doc, xml, "PTZAuxList");
  for (const XMLElement * e = r ? r->FirstChildElement("PTZAux") : nullptr; e;
    e = e->NextSiblingElement("PTZAux"))
  {
    const auto id = number(child(e, "id"));
    if (!id) {
      continue;
    }
    AuxDevice a;
    a.id = static_cast<int>(*id);
    a.type = text(child(e, "type"));
    for (auto & c : a.type) {c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));}
    a.on = text(child(e, "status")) == "on";
    out.push_back(a);
  }
  return out;
}

std::string ircutXml(const std::string & mode)
{
  return open("IrcutFilter") + "<IrcutFilterType>" + mode + "</IrcutFilterType></IrcutFilter>";
}

std::optional<std::string> parseIrcutMode(const std::string & xml)
{
  XMLDocument doc;
  const XMLElement * r = root(doc, xml, "IrcutFilter");
  const std::string mode = text(child(r, "IrcutFilterType"));
  if (mode.empty()) {
    return std::nullopt;
  }
  return mode;
}

std::optional<DeviceInfo> parseDeviceInfo(const std::string & xml)
{
  XMLDocument doc;
  const XMLElement * r = root(doc, xml, "DeviceInfo");
  if (!r) {
    return std::nullopt;
  }
  DeviceInfo d;
  d.name = text(child(r, "deviceName"));
  d.model = text(child(r, "model"));
  d.serial = text(child(r, "serialNumber"));
  d.firmware = text(child(r, "firmwareVersion"));
  d.firmware_date = text(child(r, "firmwareReleasedDate"));
  d.mac = text(child(r, "macAddress"));
  d.type = text(child(r, "deviceType"));
  return d;
}

IsapiStatus parseStatus(const std::string & body)
{
  IsapiStatus s;
  const std::string t = trim(body);
  if (t.empty()) {
    return s;
  }
  if (t[0] == '{') {
    const auto j = nlohmann::json::parse(t, nullptr, false);
    if (j.is_discarded() || !j.is_object()) {
      return s;
    }
    s.parsed = j.contains("statusCode");
    if (j.contains("statusCode") && j["statusCode"].is_number()) {
      s.status_code = j["statusCode"].get<int>();
    }
    s.status_string = j.value("statusString", "");
    s.sub_status_code = j.value("subStatusCode", "");
    s.error_msg = j.value("errorMsg", "");
    return s;
  }
  XMLDocument doc;
  const XMLElement * r = root(doc, t, "ResponseStatus");
  if (!r) {
    return s;
  }
  const auto code = number(child(r, "statusCode"));
  s.parsed = code.has_value();
  s.status_code = static_cast<int>(code.value_or(0));
  s.status_string = text(child(r, "statusString"));
  s.sub_status_code = text(child(r, "subStatusCode"));
  s.error_msg = text(child(r, "description"));
  return s;
}

std::string describeError(long http_status, const std::string & body)
{
  std::string msg = "HTTP " + std::to_string(http_status);
  const IsapiStatus s = parseStatus(body);
  if (s.parsed) {
    msg += ": " + (s.status_string.empty() ? std::to_string(s.status_code) : s.status_string);
    if (!s.sub_status_code.empty()) {
      msg += " (" + s.sub_status_code + ")";
    }
  } else if (http_status == 401) {
    msg += ": wrong user or password";
  }
  return msg;
}

std::optional<std::string> setElementText(
  const std::string & xml, const std::string & element, const std::string & value)
{
  XMLDocument doc;
  if (doc.Parse(xml.c_str(), xml.size()) != tinyxml2::XML_SUCCESS || !doc.RootElement()) {
    return std::nullopt;
  }
  auto * target = const_cast<XMLElement *>(findAnywhere(doc.RootElement(), element.c_str()));
  if (!target) {
    return std::nullopt;
  }
  target->DeleteChildren();
  target->SetText(value.c_str());
  tinyxml2::XMLPrinter printer;
  doc.Print(&printer);
  return std::string(printer.CStr());
}

}  // namespace isapi
}  // namespace hikmicro_driver
