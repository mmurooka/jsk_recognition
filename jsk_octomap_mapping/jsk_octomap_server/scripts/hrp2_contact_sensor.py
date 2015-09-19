#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from octomap_msgs.msg import ContactSensor, ContactSensorArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import WrenchStamped

import numpy as np

def lhsensor_cb(msg):
    global lhsensor_contact, lhsensor_contact_front, lhsensor_contact_lower, lhsensor_contact_upper, lhsensor_contact_inner, lhsensor_contact_outer
    force_msg = msg.wrench.force
    force_x = force_msg.x
    force_y = force_msg.y
    force_z = force_msg.z
    force = np.array([force_x, force_y, force_z])
    force_norm = np.linalg.norm(force)
    force_elem_thre = 2.5
    force_norm_thre = 3.5
    # norm
    if force_norm > force_norm_thre:
        lhsensor_contact = True
        rospy.loginfo("detect lhsensor contact. [%f / %f]" % (force_norm, force_norm_thre))
    else:
        lhsensor_contact = False
    # front
    if force_z > force_elem_thre:
        lhsensor_contact_front = True
        rospy.loginfo("detect lhsensor contact in front part. [%f / %f]" % (force_z, force_elem_thre))
    else:
        lhsensor_contact_front = False
    # lower
    if force_x > force_elem_thre:
        lhsensor_contact_lower = True
        rospy.loginfo("detect lhsensor contact in lower part. [%f / %f]" % (force_x, force_elem_thre))
    else:
        lhsensor_contact_lower = False
    # upper
    if force_x < -1 * force_elem_thre:
        lhsensor_contact_upper = True
        rospy.loginfo("detect lhsensor contact in upper part. [%f / %f]" % (force_x, - force_elem_thre))
    else:
        lhsensor_contact_upper = False
    # inner
    if force_y > force_elem_thre:
        lhsensor_contact_inner = True
        rospy.loginfo("detect lhsensor contact in inner part. [%f / %f]"  % (force_y, force_elem_thre))
    else:
        lhsensor_contact_inner = False
    # outer
    if force_y < -1 * force_elem_thre:
        lhsensor_contact_outer = True
        rospy.loginfo("detect lhsensor contact in outer part. [%f / %f]" % (force_y, - force_elem_thre))
    else:
        lhsensor_contact_outer = False


if __name__ == '__main__':
    global lhsensor_contact, lhsensor_contact_front, lhsensor_contact_lower, lhsensor_contact_upper, lhsensor_contact_inner, lhsensor_contact_outer
    rospy.init_node('test_contact_sensor')
    _frame_id = rospy.get_param('~frame_id', "/base_footprint")
    contact_sensor_array_pub = rospy.Publisher('/contact_sensors_in', ContactSensorArray, latch=True)
    rospy.Subscriber('/off_lhsensor', WrenchStamped, lhsensor_cb)
    r = rospy.Rate(10)
    lhsensor_contact = False
    lhsensor_contact_front = False
    lhsensor_contact_lower = False
    lhsensor_contact_upper = False
    lhsensor_contact_inner = False
    lhsensor_contact_outer = False
    while not rospy.is_shutdown():
        _header = Header(frame_id=_frame_id, stamp=rospy.Time.now())
        msg = ContactSensorArray()
        msg.header = _header
        LARM_LINK0_sensor = ContactSensor(header=_header, contact=False, link_name='LARM_LINK0')
        LARM_LINK1_sensor = ContactSensor(header=_header, contact=False, link_name='LARM_LINK1')
        LARM_LINK2_sensor = ContactSensor(header=_header, contact=False, link_name='LARM_LINK2')
        LARM_LINK3_sensor = ContactSensor(header=_header, contact=False, link_name='LARM_LINK3')
        LARM_LINK4_sensor = ContactSensor(header=_header, contact=False, link_name='LARM_LINK4')
        LARM_LINK5_sensor = ContactSensor(header=_header, contact=False, link_name='LARM_LINK5')
        LARM_LINK6_sensor = ContactSensor(header=_header, contact=False, link_name='LARM_LINK6')
        LARM_LINK6_FRONT_sensor = ContactSensor(header=_header, contact=lhsensor_contact_front, link_name='LARM_LINK6_FRONT')
        LARM_LINK6_UPPER_sensor = ContactSensor(header=_header, contact=lhsensor_contact_upper, link_name='LARM_LINK6_UPPER')
        LARM_LINK6_LOWER_sensor = ContactSensor(header=_header, contact=lhsensor_contact_lower, link_name='LARM_LINK6_LOWER')
        LARM_LINK6_INNER_sensor = ContactSensor(header=_header, contact=lhsensor_contact_inner, link_name='LARM_LINK6_INNER')
        LARM_LINK6_OUTER_sensor = ContactSensor(header=_header, contact=lhsensor_contact_outer, link_name='LARM_LINK6_OUTER')
        # LARM_LINK6_sensor = ContactSensor(header=_header, contact=lhsensor_contact, link_name='LARM_LINK6')
        msg.datas = [LARM_LINK0_sensor, LARM_LINK1_sensor, LARM_LINK2_sensor, LARM_LINK3_sensor, LARM_LINK4_sensor, LARM_LINK5_sensor, LARM_LINK6_sensor, LARM_LINK6_FRONT_sensor, LARM_LINK6_INNER_sensor, LARM_LINK6_OUTER_sensor, LARM_LINK6_UPPER_sensor, LARM_LINK6_LOWER_sensor]
        contact_sensor_array_pub.publish(msg)
        r.sleep()
