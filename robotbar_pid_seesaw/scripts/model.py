#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from pid_scope.msg import Sensor, Control

def sensor_callback(msg: LaserScan):
    msg_sensor.header.stamp = msg.header.stamp
    msg_sensor.name = ["distance"]
    msg_sensor.data = msg.ranges
    pub_sensor.publish(msg_sensor)

def scope_callback(msg: Control):
    pub_motor.publish(msg.value[0])


if __name__ == "__main__":
    rospy.init_node("data_transport")
    
    msg_control = Float64()
    msg_sensor = Sensor()
    pub_sensor = rospy.Publisher("ik", Sensor, queue_size=3)
    pub_motor = rospy.Publisher("/seesaw/shaft_controller/command", Float64, queue_size=2)
    
    rospy.Subscriber("/vl53l1x_sensor/distance", LaserScan, sensor_callback)
    rospy.Subscriber("/pid_scope/control_data", Control, scope_callback)
    rospy.spin()
