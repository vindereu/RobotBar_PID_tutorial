#!/usr/bin/env python3
import rospy
import signal
import threading
import time
import sys
from tdk_26 import Single_PID
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

def task():
    while not rospy.is_shutdown():
        target = float(input("New target: "))
        PID_height.set_target(target)

def callback(data: LaserScan):
    height = data.ranges[0] - 0.105
    output = PID_height.update(height)
    pub_motor.publish(-output)
    pub_effort.publish(output*0.01)

def signal_handler(signum, frame):
    if signum == signal.SIGINT.value:
        rospy.signal_shutdown("")
        sys.exit()

if __name__ == "__main__":
    PID_height = Single_PID(1, 100, 10, 0, 0.001)
    PID_height.set_output_limit(0, 2500)
    PID_height.set_I_limit(0, 1800)

    signal.signal(signal.SIGINT, signal_handler)

    thread = threading.Thread(target=task)
    thread.setDaemon(True)
    thread.start()

    rospy.init_node("flight_altitude_control")
    pub_motor = rospy.Publisher("/motor_controller/command", Float64, queue_size=2)
    pub_effort = rospy.Publisher("/board_controller/command", Float64, queue_size=2)
    rospy.Subscriber("/vl53lx1_sensor/distance", LaserScan, callback, queue_size=2)
    rospy.spin()