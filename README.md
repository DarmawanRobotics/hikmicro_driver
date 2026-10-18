<div align="center">

# hikmicro_driver

**ROS Noetic driver for the HIKMICRO HM-TD5567T bi-spectrum (thermal + visible) PTZ camera —
precise pan/tilt/zoom that tells you when it has arrived, presets, auxiliaries, RTSP images and
temperatures in °C at any pixel.**

[![CI](https://github.com/DarmawanRobotics/hikmicro_driver/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/DarmawanRobotics/hikmicro_driver/actions/workflows/ci.yml)
[![ROS](https://img.shields.io/badge/ROS-Noetic-22314E?logo=ros&logoColor=white)](https://wiki.ros.org/noetic)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/20.04/)
![C++](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)
[![License](https://img.shields.io/badge/license-Apache%202.0-blue)](LICENSE)

</div>

---

Move to a position and know when the camera is really there, save and call presets, switch the
light, wiper and day/night, read temperatures from the radiometric image and watch both sensors.
All over Ethernet: **ISAPI** (HTTP, digest authentication) for control and thermometry, **RTSP**
for video.

**Docs:** [ISAPI summary](docs/isapi.md) · [network setup](docs/network.md) ·
[first run on the camera](docs/first_run.md) · [troubleshooting](docs/troubleshooting.md) ·
[messages](hikmicro_msgs/README.md)

| Package | |
|---|---|
| [`hikmicro_msgs`](hikmicro_msgs) | messages, services and the `MoveTo` / `GotoPreset` actions |
| [`hikmicro_driver`](hikmicro_driver) | the ISAPI client (no ROS), `hikmicro_ptz_node`, `hikmicro_camera_node` |

## Quick start

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/DarmawanRobotics/hikmicro_driver.git
cd ~/catkin_ws
rosdep update --rosdistro noetic --include-eol-distros
rosdep install --from-paths src --ignore-src -y --rosdistro noetic
catkin_make && source devel/setup.bash

export HIKMICRO_PASSWORD='<camera password>'        # never printed by roslaunch
roslaunch hikmicro_driver hikmicro.launch host:=192.168.1.64   # cameras:=false for control only
```

The camera's factory address is `192.168.1.64` (user `admin`): give the robot computer an
address in `192.168.1.x` and check with
`curl --digest -u admin:<password> http://192.168.1.64/ISAPI/System/deviceInfo`.

## Interface

Everything under `/hikmicro`.

| Name | Type | |
|---|---|---|
| `ptz/move_to` | action `MoveTo` | absolute pan / tilt / zoom; succeeds only when the camera **has arrived** |
| `ptz/goto_preset` | action `GotoPreset` | call a preset, same arrival check |
| `ptz/state` | `PtzState` | connected, pan, tilt, zoom, moving |
| `joint_states` | `sensor_msgs/JointState` | pan / tilt joints |
| `ptz/cmd_vel` (in) | `geometry_msgs/Twist` | manual: `angular.z` pan, `angular.y` tilt, `linear.z` zoom |
| `thermal/temperature` | `sensor_msgs/Image` | 32FC1, °C per pixel |
| `thermal/stats` | `ThermalStats` | frame max / min / mean and where |
| `visible/image_raw`, `thermal/image_raw` | `sensor_msgs/Image` | the two video streams |
| `ptz/stop`, `ptz/focus` | `std_srvs/Trigger` | stop motion, one-push autofocus |
| `ptz/set_preset`, `ptz/remove_preset`, `ptz/get_presets` | services | presets |
| `aux/light`, `aux/wiper`, `aux/set` | services | auxiliaries |
| `image/day_night` | `SetDayNight` | `auto` / `day` / `night` |
| `snapshot` | `Snapshot` | full-resolution JPEG, visible or thermal |
| `thermal/get_temperature` | `GetTemperature` | °C at a point, in a box or over the frame |
| `isapi/request` | `RawRequest` | any other ISAPI call |

**`move_to` states:** `reached` · `stalled` (still, outside tolerance) · `timeout` ·
`preempted` · `superseded` · `error`. Only `reached` succeeds.

## Usage

```bash
# move and wait
rostopic pub -1 /hikmicro/ptz/move_to/goal hikmicro_msgs/MoveToActionGoal \
  "{goal: {pan: 123.4, tilt: -12.5, zoom: 8.0}}"
rostopic echo -n 1 /hikmicro/ptz/move_to/result

# presets, light, day/night
rosservice call /hikmicro/ptz/set_preset "{id: 3, name: 'point 3'}"
rosservice call /hikmicro/aux/light "data: true"
rosservice call /hikmicro/image/day_night "mode: 'night'"

# temperature in a box (thermal pixels)
rosservice call /hikmicro/thermal/get_temperature "{mode: 1, x: 300, y: 200, x2: 360, y2: 260}"
```

```python
import actionlib, rospy
from hikmicro_msgs.msg import MoveToAction, MoveToGoal
from hikmicro_msgs.srv import GetTemperature

rospy.init_node('thermal_inspection')
move = actionlib.SimpleActionClient('/hikmicro/ptz/move_to', MoveToAction)
move.wait_for_server()
temperature = rospy.ServiceProxy('/hikmicro/thermal/get_temperature', GetTemperature)

move.send_goal_and_wait(MoveToGoal(pan=123.4, tilt=-12.5, zoom=8.0))
if move.get_result().reached:
    t = temperature(mode=1, x=300, y=200, x2=360, y2=260, max_age=0.5)
    rospy.loginfo('max %.1f C', t.stats.max)
```

Parameters are commented in [hikmicro.yaml](hikmicro_driver/config/hikmicro.yaml) and
[camera.yaml](hikmicro_driver/config/camera.yaml).

## Verify on the real camera

1. `ptz/state` shows `connected: True`; the log shows model, firmware and `high_precision`.
2. Note which way `pan` / `tilt` turn and set `pan_sign` / `tilt_sign` and `invert_*`.
3. A far move ends `reached`; a move past the tilt limit ends `stalled` or `error`.
4. `aux/light` / `aux/wiper` switch the right device (else set `light_aux_id` / `wiper_aux_id`).
5. A hand in front of the lens reads 30–36 °C with `get_temperature`.

## Develop

```bash
catkin_make run_tests && catkin_test_results build/test_results
bash src/hikmicro_driver/hikmicro_driver/test/smoke_test.sh
```

Unit tests cover the ISAPI messages, radiometric decoding, arrival detection, RTSP and the client
against a fake camera; the smoke test runs the launch file against it. CI runs both on
`ros:noetic`.

Released under the [Apache License 2.0](LICENSE).
