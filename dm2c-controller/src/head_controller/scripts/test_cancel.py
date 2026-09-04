#!/usr/bin/env python3
"""
Cancel current HeadSweep action.
Usage: rosrun head_controller test_cancel.py
"""

import rospy
import actionlib
from common_comms.msg import HeadSweepAction, HeadSweepGoal


def main():
    rospy.init_node("test_head_sweep_cancel")

    client = actionlib.SimpleActionClient("/head_sweep", HeadSweepAction)
    if not client.wait_for_server(rospy.Duration(3.0)):
        rospy.logerr("cannot connect to /head_sweep, is head_controller running?")
        return

    client.cancel_all_goals()
    rospy.loginfo("cancel sent")


if __name__ == "__main__":
    main()
