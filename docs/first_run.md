# First run on the camera

The driver is tested against a simulated camera
([fake_camera.py](../hikmicro_driver/test/fake_camera.py)). Fill in the **Result** column on
the real camera; it decides the defaults that go into `hikmicro.yaml`.

```bash
export HIKMICRO_PASSWORD='<camera password>'
roslaunch hikmicro_driver hikmicro.launch host:=192.168.1.64
```

| # | Check | Command | Expected | Result |
|---|---|---|---|---|
| 1 | Connection | `rostopic echo -n 1 /hikmicro/ptz/state` | `connected: True`; the log shows model and firmware | |
| 2 | Precision | same | `high_precision: True` (absoluteEx, 0.001°) | |
| 3 | Pan direction | `move_to` pan 10, then pan 20 | turns which way? → `pan_sign`, `invert_pan` | |
| 4 | Tilt direction | `move_to` tilt 0, then tilt 10 | tilts which way? → `tilt_sign`, `invert_tilt` | |
| 5 | Arrival | `rostopic pub -1 /hikmicro/ptz/move_to/goal hikmicro_msgs/MoveToActionGoal "{goal: {pan: 120, tilt: -10, zoom: 4}}"` | result `state: "reached"` within tolerance | |
| 6 | Limit | `move_to` beyond the tilt limit | `stalled` or `error` | |
| 7 | Aux ids | `rosservice call /hikmicro/isapi/request "{method: GET, path: /ISAPI/PTZCtrl/channels/1/auxcontrols}"` | ids of LIGHT / WIPER | light: · wiper: |
| 8 | Light / wiper | `rosservice call /hikmicro/aux/light "data: true"`, `aux/wiper` | the right device switches | |
| 9 | Night | `rosservice call /hikmicro/image/day_night "mode: 'night'"` | B/W image with IR light | |
| 10 | Thermometry | `rosservice call /hikmicro/thermal/get_temperature "{mode: 0, normalized: true, x: 0.5, y: 0.5}"` on a hand | 30–36 °C | |
| 11 | Streams | `rostopic hz /hikmicro/visible/image_raw /hikmicro/thermal/image_raw` | stream frame rates | codec: |

## If a check fails

- **2 `False`:** the firmware has no `absoluteEx`; moves use `/status` with 0.1° steps.
- **7 / 8:** set `light_aux_id` / `wiper_aux_id` in [hikmicro.yaml](../hikmicro_driver/config/hikmicro.yaml).
- **10 `notSupport`:** no pixel-to-pixel thermometry on this firmware.
- **11 with `decoder: jetson`:** set `codec` to what `ffprobe` shows (H.264 / H.265).

See also [troubleshooting.md](troubleshooting.md).
