#!/usr/bin/env python3
"""
head_controller 测试脚本：
发送 HeadSweep action 目标，实时显示 feedback。
用法: rosrun head_controller test_head_sweep.py [yaw_deg] [pitch_deg]
示例: rosrun head_controller test_head_sweep.py 45 45   # yaw右转45, pitch向下45
        rosrun head_controller test_head_sweep.py -45 0   # yaw左转45, pitch不动
"""

import sys
import rospy
import actionlib
from common_comms.msg import HeadSweepAction, HeadSweepGoal


def feedback_cb(fb):
    rospy.loginfo("[Feedback] yaw=%.1f  pitch=%.1f", fb.current_yaw, fb.current_pitch)


def main():
    rospy.init_node("test_head_sweep")

    yaw = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    pitch = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0

    ac_name = "/head_sweep"
    client = actionlib.SimpleActionClient(ac_name, HeadSweepAction)

    rospy.loginfo("等待 action server '%s' ...", ac_name)
    rospy.loginfo("提示: 如需查看所有话题: rostopic list | grep head")
    if not client.wait_for_server(rospy.Duration(10.0)):
        rospy.logerr("连接 '%s' 超时, 请检查:", ac_name)
        rospy.logerr("  1. head_controller 节点是否运行? (rosnode list)")
        rospy.logerr("  2. 话题名是否正确? (rostopic list | grep head)")
        return

    goal = HeadSweepGoal(yaw=yaw, pitch=pitch)
    rospy.loginfo("发送: yaw=%.1f  pitch=%.1f", yaw, pitch)

    client.send_goal(goal, feedback_cb=feedback_cb)
    client.wait_for_result()

    result = client.get_result()
    rospy.loginfo("结果: code=%d  msg=%s", result.error_code, result.message)


if __name__ == "__main__":
    main()
