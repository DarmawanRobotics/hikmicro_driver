#!/usr/bin/env bash
# End-to-end smoke test: hikmicro.launch against test/fake_camera.py on the loopback.
# PTZ state, move_to (reached / stalled / preempted), presets, cmd_vel and its watchdog,
# light, wiper, day/night, focus, snapshots, thermometry, raw ISAPI, and both camera nodes
# staying up without a stream. Needs a sourced workspace.
set -u
# rospy / rostopic output into a pipe is block-buffered and lost when timeout kills it
export PYTHONUNBUFFERED=1

here="$(cd "$(dirname "$0")" && pwd)"
work="$(mktemp -d)"
port=18080
ns=/hikmicro
fail=0
pids=()

cleanup() {
  for p in "${pids[@]}"; do kill -INT "$p" 2>/dev/null; done
  sleep 2
  for p in "${pids[@]}"; do kill -9 "$p" 2>/dev/null; done
}
trap cleanup EXIT

after() {  # after <name> <first pattern> <second pattern> <file>: second after first
  if awk -v a="$2" -v b="$3" '$0 ~ a {seen=1; next} seen && $0 ~ b {found=1} END {exit !found}' "$4"
  then
    echo "ok   $1"
  else
    echo "FAIL $1"
    tail -n 15 "$4"
    fail=1
  fi
}

check() {  # check <name> <pattern> <command...>
  local name="$1" pattern="$2"
  shift 2
  local out
  out="$(timeout 45 "$@" 2>&1)"
  if grep -qE -- "$pattern" <<<"$out"; then
    echo "ok   $name"
  else
    echo "FAIL $name"
    echo "$out" | tail -n 25
    fail=1
  fi
}

# the password goes through the environment, so roslaunch never prints it
export HIKMICRO_PASSWORD=test12345
python3 "$here/fake_camera.py" --port "$port" --log "$work/camera.log" &
pids+=($!)
roslaunch hikmicro_driver hikmicro.launch host:=127.0.0.1 port:="$port" \
  > "$work/launch.log" 2>&1 &
pids+=($!)
for _ in $(seq 1 30); do
  rostopic list 2>/dev/null | grep -q "$ns/ptz/state" && break
  sleep 1
done
sleep 3

check "ptz state connected" "connected: True" rostopic echo -n 1 $ns/ptz/state
check "ptz state high precision" "high_precision: True" rostopic echo -n 1 $ns/ptz/state
check "joint states" "hikmicro_pan_joint" rostopic echo -n 1 $ns/joint_states

check "move_to reached" "reached=True state=reached pan=45.5" \
  python3 "$here/action_client.py" move 45.5 -10.25 4
check "move_to feedback" "feedback=[1-9]" python3 "$here/action_client.py" move 90 0 2
check "move_to stalled at end stop" "reached=False state=stalled" \
  python3 "$here/action_client.py" move 90 120
check "move_to preempted" "state=preempted" python3 "$here/action_client.py" preempt 300 -40

check "set_preset" "success: True" \
  rosservice call $ns/ptz/set_preset "{id: 3, name: 'trafo 1 bushing'}"
check "get_presets" "trafo 1 bushing" rosservice call $ns/ptz/get_presets
python3 "$here/action_client.py" move 10 0 > /dev/null 2>&1
check "goto_preset reached" "reached=True state=reached" python3 "$here/action_client.py" preset 3
check "goto_preset unknown" "reached=False state=error" python3 "$here/action_client.py" preset 42
check "remove_preset" "success: True" rosservice call $ns/ptz/remove_preset "{id: 3}"

timeout 10 rostopic pub -r 10 $ns/ptz/cmd_vel geometry_msgs/Twist \
  "{angular: {z: 0.5, y: -0.25}}" > /dev/null 2>&1 &
sleep 2
kill -INT $! 2>/dev/null
sleep 1.5   # cmd_vel_timeout 0.5 s
check "cmd_vel moves" "continuous.*<pan>30</pan><tilt>-15</tilt>" cat "$work/camera.log"
after "cmd_vel watchdog stops" "<pan>30</pan><tilt>-15</tilt>" \
  "continuous.*<pan>0</pan><tilt>0</tilt><zoom>0</zoom>" "$work/camera.log"
check "stop" "success: True" rosservice call $ns/ptz/stop
check "focus" "success: True" rosservice call $ns/ptz/focus

check "light on" "success: True" rosservice call $ns/aux/light "data: true"
check "light reached the camera" "auxcontrols/1 .*<type>LIGHT</type><status>on</status>" \
  cat "$work/camera.log"
check "wiper on" "success: True" rosservice call $ns/aux/wiper "data: true"
check "heater missing" "success: False" rosservice call $ns/aux/set "{type: HEATER, enabled: true}"
check "day/night" 'mode: "night"' rosservice call $ns/image/day_night "mode: 'night'"

check "snapshot visible" "format: \"jpeg\"" rosservice call $ns/snapshot "channel: 0"
check "snapshot thermal" "success: True" rosservice call $ns/snapshot "channel: 1"

check "temperature at hot spot" "max: 87.5" \
  rosservice call $ns/thermal/get_temperature "{mode: 0, x: 100, y: 40}"
check "temperature in a box" "max_x: 100" \
  rosservice call $ns/thermal/get_temperature "{mode: 1, x: 90, y: 30, x2: 110, y2: 50}"
check "temperature normalized" "success: True" \
  rosservice call $ns/thermal/get_temperature "{mode: 0, normalized: true, x: 0.5, y: 0.5}"
check "temperature outside" "success: False" \
  rosservice call $ns/thermal/get_temperature "{mode: 0, x: 500, y: 40}"
check "thermal stats" "max_y: 40" rostopic echo -n 1 $ns/thermal/stats
check "temperature image" "encoding: \"32FC1\"" \
  rostopic echo -n 1 --noarr $ns/thermal/temperature

check "raw isapi" "HM-TD5567T" \
  rosservice call $ns/isapi/request "{method: GET, path: /ISAPI/System/deviceInfo}"

check "visible camera up" "cannot open rtsp://admin@127.0.0.1:554/Streaming/Channels/101" \
  cat "$work/launch.log"
check "thermal camera up" "cannot open rtsp://admin@127.0.0.1:554/Streaming/Channels/201" \
  cat "$work/launch.log"
if grep -q "test12345" "$work/launch.log"; then
  echo "FAIL password in the log"
  fail=1
fi
if grep -qE "process has died|REQUIRED process" "$work/launch.log"; then
  echo "FAIL a node died"
  fail=1
fi
if [ "$fail" != 0 ]; then
  echo "---- launch log"
  tail -n 60 "$work/launch.log"
  echo "---- camera log"
  tail -n 30 "$work/camera.log"
fi
exit "$fail"
