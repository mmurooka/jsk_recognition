#!/usr/bin/env python
import rospy
import argparse
import os

PKG='jsk_pcl_ros'

from visualization_msgs.msg import Marker
from jsk_pcl_ros.srv import VoxelModelGenerate

def markerCallback(marker_msg):
    global cb_flag
    # request service to generate urdf model
    rospy.loginfo("Request model generation.")
    if not generate_srv(voxel=marker_msg, name="test_model", filename="/tmp/test.urdf"):
        rospy.logwarn("Request of model generation failed.")
        return
    if args.gazebo:
        # generate model name for gazebo
        from gazebo_msgs.srv import GetWorldProperties
        gazebo_prop_srv = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
        gazebo_prop = gazebo_prop_srv()
        model_name = args.model or "test0"
        model_id = 0
        while model_name in gazebo_prop.model_names:
        model_id += 1
        model_name = "test%d" % model_id
        # spawn model to gazebo
        rospy.loginfo("Spawn urdf model into gazebo. model name is %s." % model_name)
        os.system("gzfactory spawn -f /tmp/test.urdf -m %s -x 0 -y 0 -z 1" % model_name)
    cb_flag = True

def main():
    global args, marker_sub, generate_srv, cb_flag
    rospy.init_node("voxel_urdf_client")
    # parse argument
    parser = argparse.ArgumentParser()
    parser.add_argument('--gazebo', default=False)
    parser.add_argument('--model', default=False)
    parser.add_argument('--loop', default=True)
    args = parser.parse_args(rospy.myargv()[1:])

    marker_sub = rospy.Subscriber("input", Marker, markerCallback, queue_size=1)
    generate_srv = rospy.ServiceProxy("/generate_voxel_urdf", VoxelModelGenerate)

    cb_flag = False
    while args.loop or (not cb_flag)
        rospy.sleep(1.0)

if __name__ == "__main__":
    main()
