// ROS side of the camera's housekeeping: light and wiper (PTZ auxiliaries), day/night (IR-cut
// and IR light), full-resolution JPEG snapshots, and a raw ISAPI request for everything else.
#pragma once

#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include <memory>
#include <string>

#include "hikmicro_ptz_driver/RawRequest.h"
#include "hikmicro_ptz_driver/SetAux.h"
#include "hikmicro_ptz_driver/SetDayNight.h"
#include "hikmicro_ptz_driver/Snapshot.h"
#include "hikmicro_ptz_driver/camera.hpp"

namespace hikmicro_ptz_driver
{

class AuxInterface
{
public:
  AuxInterface(ros::NodeHandle & nh, ros::NodeHandle & pnh, std::shared_ptr<HikCamera> camera);

private:
  bool onLight(std_srvs::SetBool::Request & req, std_srvs::SetBool::Response & res);
  bool onWiper(std_srvs::SetBool::Request & req, std_srvs::SetBool::Response & res);
  bool onAux(SetAux::Request & req, SetAux::Response & res);
  bool onDayNight(SetDayNight::Request & req, SetDayNight::Response & res);
  bool onSnapshot(Snapshot::Request & req, Snapshot::Response & res);
  bool onRaw(RawRequest::Request & req, RawRequest::Response & res);

  std::shared_ptr<HikCamera> cam_;
  std::string visible_frame_, thermal_frame_;
  ros::ServiceServer light_srv_, wiper_srv_, aux_srv_, day_night_srv_, snapshot_srv_, raw_srv_;
};

}  // namespace hikmicro_ptz_driver
