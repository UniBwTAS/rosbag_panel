#!/usr/bin/env python

import os
import xml.etree.ElementTree as ET  # noqa

import rospy
import sys
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

if len(sys.argv) < 2:
    print("Please pass parent folder of XML file(s) as argument.")
    exit(1)

rospy.init_node("publish_tf_static_from_xml", anonymous=True)
pub = StaticTransformBroadcaster()

for filename in os.listdir(sys.argv[1]):
    if filename.endswith('.xml'):
        try:
            tree = ET.parse(os.path.join(sys.argv[1], filename))
        except ET.ParseError:
            continue
        root = tree.getroot()
        if root.tag != 'StaticTfMsgs':
            continue
        for tfs_xml in root:
            tfs = []
            for tf_xml in tfs_xml:
                tf = TransformStamped()
                tf.header.stamp.secs = int(tf_xml.attrib['sec'])
                tf.header.stamp.nsecs = int(tf_xml.attrib['nsec'])
                tf.header.frame_id = tf_xml.attrib['frame_id']
                tf.child_frame_id = tf_xml.attrib['child_frame_id']
                tf.transform.translation.x = float(tf_xml.attrib['translation.x'])
                tf.transform.translation.y = float(tf_xml.attrib['translation.y'])
                tf.transform.translation.z = float(tf_xml.attrib['translation.z'])
                tf.transform.rotation.x = float(tf_xml.attrib['rotation.x'])
                tf.transform.rotation.y = float(tf_xml.attrib['rotation.y'])
                tf.transform.rotation.z = float(tf_xml.attrib['rotation.z'])
                tf.transform.rotation.w = float(tf_xml.attrib['rotation.w'])
                tfs.append(tf)
            pub.sendTransform(tfs)

rospy.spin()
