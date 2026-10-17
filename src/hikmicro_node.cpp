// hikmicro_ptz_node: one ISAPI connection to the camera, shared by the PTZ, auxiliary and
// thermometry interfaces. Callbacks run on an AsyncSpinner, so a slow request (a snapshot, a
// radiometric frame) never blocks a stop.
#include <ros/ros.h>

#include <cstdlib>
#include <map>
#include <memory>
#include <string>

#include "hikmicro_ptz_driver/aux_interface.hpp"
#include "hikmicro_ptz_driver/camera.hpp"
#include "hikmicro_ptz_driver/ptz_interface.hpp"
#include "hikmicro_ptz_driver/thermal_interface.hpp"

using namespace hikmicro_ptz_driver;

namespace
{

CameraConfig loadConfig(ros::NodeHandle & pnh)
{
  CameraConfig c;
  pnh.param<std::string>("host", c.host, c.host);
  pnh.param("port", c.port, 80);
  pnh.param("https", c.https, false);
  pnh.param<std::string>("user", c.user, "admin");
  // The password param is printed by roslaunch; prefer the environment variable.
  pnh.param<std::string>("password", c.password, "");
  if (c.password.empty()) {
    std::string env_name;
    pnh.param<std::string>("password_env", env_name, "HIKMICRO_PASSWORD");
    if (const char * env = std::getenv(env_name.c_str())) {
      c.password = env;
    }
  }
  pnh.param("ptz_channel", c.ptz_channel, 1);
  pnh.param("visible_channel", c.visible_channel, 1);
  pnh.param("thermal_channel", c.thermal_channel, 2);
  pnh.param("timeout", c.timeout, 3.0);
  pnh.param("image_timeout", c.image_timeout, 10.0);
  int light = 0, wiper = 0;
  pnh.param("light_aux_id", light, 0);
  pnh.param("wiper_aux_id", wiper, 0);
  if (light > 0) {
    c.aux_ids["LIGHT"] = light;
  }
  if (wiper > 0) {
    c.aux_ids["WIPER"] = wiper;
  }
  return c;
}

}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "hikmicro_ptz_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  const CameraConfig cfg = loadConfig(pnh);
  if (cfg.password.empty()) {
    ROS_WARN("no password (~password or $HIKMICRO_PASSWORD): the camera will answer 401");
  }
  auto cam = std::make_shared<HikCamera>(cfg);
  ROS_INFO("ISAPI on %s://%s:%d as %s", cfg.https ? "https" : "http", cfg.host.c_str(), cfg.port,
    cfg.user.c_str());
  std::string err;
  if (const auto info = cam->deviceInfo(&err)) {
    ROS_INFO("camera: %s, serial %s, firmware %s %s", info->model.c_str(), info->serial.c_str(),
      info->firmware.c_str(), info->firmware_date.c_str());
  } else {
    ROS_WARN("deviceInfo: %s (ptz/state shows when the camera answers)", err.c_str());
  }

  PtzInterface ptz(nh, pnh, cam);
  AuxInterface aux(nh, pnh, cam);
  ThermalInterface thermal(nh, pnh, cam);

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
