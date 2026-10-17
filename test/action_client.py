#!/usr/bin/env python3
"""Smoke-test client for the PTZ actions.

  action_client.py move PAN TILT ZOOM     move_to, wait for the result
  action_client.py preset ID              goto_preset, wait for the result
  action_client.py preempt PAN TILT       move_to, cancel after 0.3 s
"""

import sys

import actionlib
import rospy
from hikmicro_ptz_driver.msg import GotoPresetAction, GotoPresetGoal, MoveToAction, MoveToGoal


def main():
    rospy.init_node('smoke_action_client', anonymous=True)
    ns = rospy.get_param('~ns', '/hikmicro')
    kind = sys.argv[1]
    if kind == 'preset':
        client = actionlib.SimpleActionClient(ns + '/ptz/goto_preset', GotoPresetAction)
        goal = GotoPresetGoal(id=int(sys.argv[2]))
    else:
        client = actionlib.SimpleActionClient(ns + '/ptz/move_to', MoveToAction)
        zoom = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
        goal = MoveToGoal(pan=float(sys.argv[2]), tilt=float(sys.argv[3]), zoom=zoom)
    if not client.wait_for_server(rospy.Duration(15)):
        print('no action server')
        return 2
    feedback = []
    client.send_goal(goal, feedback_cb=feedback.append)
    if kind == 'preempt':
        rospy.sleep(0.3)
        client.cancel_goal()
    client.wait_for_result(rospy.Duration(40))
    r = client.get_result()
    if r is None:
        print('no result')
        return 3
    print('result reached=%s state=%s pan=%.3f tilt=%.3f zoom=%.2f feedback=%d message=%s'
          % (r.reached, r.state, r.pan, r.tilt, r.zoom, len(feedback), r.message))
    return 0


if __name__ == '__main__':
    sys.exit(main())
