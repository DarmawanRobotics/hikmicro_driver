# hikmicro_msgs

Messages, services and actions of [hikmicro_driver](../README.md).

## Actions

| | |
|---|---|
| `MoveTo` | goal: `pan`, `tilt`, `zoom` (≤ 0 keeps it), `speed`, `tolerance`, `timeout` · result: `reached`, `state` (`reached`, `stalled`, `timeout`, `preempted`, `superseded`, `error`), position, `message` · feedback: position and errors |
| `GotoPreset` | goal: preset `id`; same result and feedback |

## Messages

| | |
|---|---|
| `PtzState` | connected, high precision, moving, pan / tilt (°), zoom (x) |
| `PtzPreset` | id, name, enabled, position |
| `ThermalStats` | mode (`POINT`, `REGION`, `FRAME`), box used, max / min / mean (°C) and where |

## Services

| | |
|---|---|
| `GetPresets` | every preset with its position |
| `SetPreset` | save the current position as `id` / `name` (also used to remove) |
| `SetAux` | auxiliary by type (`LIGHT`, `WIPER`, `FAN`, `HEATER`), `enabled` |
| `SetDayNight` | IR-cut mode `auto` / `day` / `night` (empty: read) |
| `Snapshot` | JPEG of the visible (`0`) or thermal (`1`) sensor |
| `GetTemperature` | °C at a point, in a box or over the frame; pixels or normalized |
| `RawRequest` | any ISAPI call |

```python
from hikmicro_msgs.msg import MoveToAction, MoveToGoal, PtzState
from hikmicro_msgs.srv import GetTemperature
```
