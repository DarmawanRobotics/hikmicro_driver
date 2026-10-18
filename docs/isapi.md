# ISAPI summary

What the driver uses from *ISAPI Thermography thermal cameras PT series* (Hikvision / HIKMICRO).
All requests are HTTP with **digest authentication** on port 80; `<ch>` is 1 for the visible /
PTZ channel and 2 for the thermal one.

## PTZ

| Method, path | Body / answer | Driver |
|---|---|---|
| `GET /ISAPI/PTZCtrl/channels/1/absoluteEx` | `elevation`, `azimuth` (degrees, 0.001), `absoluteZoom` (ratio) | `ptz/state` (high precision) |
| `PUT /ISAPI/PTZCtrl/channels/1/absoluteEx` | same fields, optional `horizontalSpeed` / `verticalSpeed` | `ptz/move_to` |
| `GET /ISAPI/PTZCtrl/channels/1/status` | `AbsoluteHigh`: elevation −900..2700, azimuth 0..3600 (tenths), `absoluteZoom` ×10 | fallback without `absoluteEx` |
| `PUT /ISAPI/PTZCtrl/channels/1/absolute` | `AbsoluteHigh` | fallback move |
| `PUT /ISAPI/PTZCtrl/channels/1/continuous` | `pan`, `tilt`, `zoom` −100..100 (0 0 0 stops) | `ptz/cmd_vel`, `ptz/stop` |
| `GET /ISAPI/PTZCtrl/channels/1/presets` | id, name, position | `ptz/get_presets` |
| `PUT` / `DELETE /ISAPI/PTZCtrl/channels/1/presets/<id>` | save the current position / delete | `ptz/set_preset`, `ptz/remove_preset` |
| `PUT /ISAPI/PTZCtrl/channels/1/presets/<id>/goto` | — | `ptz/goto_preset` |
| `PUT /ISAPI/PTZCtrl/channels/1/onepushfoucs/start` | (the camera's own spelling) | `ptz/focus` |

The camera only acknowledges a move; "arrived" is decided from the polled position (in
tolerance and still for `settle_time`).

## Auxiliaries and image

| Method, path | Body / answer | Driver |
|---|---|---|
| `GET /ISAPI/PTZCtrl/channels/1/auxcontrols` | list of id + type (`LIGHT`, `WIPER`, `FAN`, `HEATER`) | ids for the aux services |
| `PUT /ISAPI/PTZCtrl/channels/1/auxcontrols/<id>` | `PTZAux`: type, `status` on / off | `aux/light`, `aux/wiper`, `aux/set` |
| `GET` / `PUT /ISAPI/Image/channels/1/ircutFilter` | `IrcutFilterType` auto / day / night | `image/day_night` |
| `GET /ISAPI/Streaming/channels/<ch>01/picture` | JPEG | `snapshot` |
| `GET /ISAPI/System/deviceInfo` | model, serial, firmware | log at start |

## Thermometry

`GET /ISAPI/Thermal/channels/2/thermometry/jpegPicWithAppendData?format=json` answers
`multipart/form-data`:

1. JSON `JpegPictureWithAppendData`: `jpegPicWidth`, `jpegPicHeight`,
   `temperatureDataLength`, `scale`, `offset`
2. the thermal JPEG
3. the temperature matrix: float32 °C, or 16-bit where `K = raw / scale + offset`
4. on bi-spectrum models, the visible JPEG

The driver publishes it as `thermal/temperature` (32FC1 °C) and answers
`thermal/get_temperature` from it.

## Errors

`ResponseStatus` (XML) or JSON with `statusCode`, `subStatusCode` (e.g. `notSupport`), and
`errorMsg`; the driver puts them in the service / action messages.

## RTSP

`rtsp://<ip>:554/Streaming/Channels/101` visible, `/201` thermal (`102` / `202` sub streams).
