#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from octomap_msgs.msg import ContactSensor, ContactSensorArray
from geometry_msgs.msg import Pose

if __name__ == '__main__':
    rospy.init_node('test_contact_sensor')
    _frame_id = rospy.get_param('~frame_id', "/base_footprint")
    contact_sensor_array_pub = rospy.Publisher('/contact_sensors_in', ContactSensorArray, latch=True)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        _header = Header(frame_id=_frame_id, stamp=rospy.Time.now())
        msg = ContactSensorArray()
        msg.header = _header
        l_gripper_palm_link_sensor = ContactSensor(header=_header, contact=False, link_name='l_gripper_palm_link')
        l_gripper_l_finger_link_sensor = ContactSensor(header=_header, contact=False, link_name='l_gripper_l_finger_link')
        l_gripper_r_finger_link_sensor = ContactSensor(header=_header, contact=False, link_name='l_gripper_r_finger_link')
        l_forearm_link_sensor = ContactSensor(header=_header, contact=False, link_name='l_forearm_link')
        msg.datas = [l_gripper_palm_link_sensor, l_gripper_l_finger_link_sensor, l_gripper_r_finger_link_sensor, l_forearm_link_sensor]
        contact_sensor_array_pub.publish(msg)
        r.sleep()
