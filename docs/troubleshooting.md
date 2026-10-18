# Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `deviceInfo: ... Couldn't connect` | address or subnet | [network.md](network.md); `ping 192.168.1.64` |
| `HTTP 401 ... wrong user or password` | password, or the account is locked after failed logins | `export HIKMICRO_PASSWORD=...`; wait for the lock to expire |
| `no password ... the camera will answer 401` | `HIKMICRO_PASSWORD` not set | set it, or `password:=` (printed by roslaunch) |
| `ptz/state` `high_precision: False` | no `absoluteEx` on this firmware | works with 0.1° via `/status` |
| `move_to` → `stalled` | end stop, limit or obstruction | target inside the camera's range |
| `move_to` → `timeout` | slow move, or tolerance too tight | raise `move_timeout` / `pan_tolerance` / `tilt_tolerance` |
| `move_to` → `superseded` | a newer goal, `cmd_vel` or `ptz/stop` took over | |
| arrived but the image still shakes | settles slower than `settle_time` | raise `settle_time` |
| `aux/light` switches the wrong device or `missing` | aux ids differ | `light_aux_id`, `wiper_aux_id` (first run check 7) |
| `get_temperature` `notSupport` | no pixel-to-pixel thermometry | |
| `radiometric frame: ...` warnings every 30 s | thermometry failing; the node backs off | see the message (`notSupport`, timeout) |
| `cannot open rtsp://...` repeating | port 554, credentials or decoder | `ffprobe` the URL; `decoder: ffmpeg` |
| `cmd_vel` keeps moving | publisher stopped without a zero command | stops by itself after `cmd_vel_timeout` (0.5 s) |

```bash
roslaunch hikmicro_driver hikmicro.launch host:=<ip> 2>&1 | tee hikmicro.log
rosservice call /hikmicro/isapi/request "{method: GET, path: /ISAPI/PTZCtrl/channels/1/capabilities}"
```
