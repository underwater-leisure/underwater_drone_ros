#!/usr/bin/env python
import json

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def img_loader(img_path_list):
    len_img_list = len(img_path_list)
    idx = 0
    while not rospy.is_shutdown():
        if idx == len_img_list:
            idx = 0
        img_l = cv2.imread(img_path_list[idx][0])
        img_r = cv2.imread(img_path_list[idx][1])
        idx += 1

        yield img_l, img_r


def image_publisher(matching_img):
    pub_l = rospy.Publisher('/stereo/left/image_raw', Image, queue_size=10)
    pub_r = rospy.Publisher('/stereo/right/image_raw', Image, queue_size=10)
    pub_l_color = rospy.Publisher(
        '/stereo/left/image_color', Image, queue_size=10)
    pub_r_color = rospy.Publisher(
        '/stereo/right/image_color', Image, queue_size=10)
    pub_l_mono = rospy.Publisher(
        '/stereo/left/image_mono', Image, queue_size=10)
    pub_r_mono = rospy.Publisher(
        '/stereo/right/image_mono', Image, queue_size=10)

    bridge = CvBridge()

    rospy.init_node('img_publisher', anonymous=True)
    rate = rospy.Rate(15)  # 15hz

    img_list = []
    for idx in range(10):
        img_l_temp = cv2.imread(matching_img[idx][0])
        img_r_temp = cv2.imread(matching_img[idx][1])

        img_list.append((img_l_temp, img_r_temp))

    idx = 0
    while not rospy.is_shutdown():
        info_str = "image publish %s" % rospy.get_time()
        rospy.loginfo(info_str)

        if idx == len(img_list):
            idx = 0
        img_l, img_r = img_list[idx]

        try:
            imgmsg_l = bridge.cv2_to_imgmsg(img_l, "bgr8")
            imgmsg_r = bridge.cv2_to_imgmsg(img_r, "bgr8")

            img_l_gray = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
            img_r_gray = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)

            imgmsg_l_gray = bridge.cv2_to_imgmsg(img_l_gray, "mono8")
            imgmsg_r_gray = bridge.cv2_to_imgmsg(img_r_gray, "mono8")
        except CvBridgeError as e:
            print(e)

        pub_l.publish(imgmsg_l)
        pub_r.publish(imgmsg_r)

        pub_l_color.publish(imgmsg_l)
        pub_r_color.publish(imgmsg_r)

        pub_l_mono.publish(imgmsg_l_gray)
        pub_r_mono.publish(imgmsg_r_gray)

        rate.sleep()
        idx += 1


if __name__ == '__main__':

    with open("./datasets/1206_freemove.json", "r") as f:
        matching_data = json.load(f)

    matching_img = [
        (
            key,
            values["right_img"],
        )
        for key, values in matching_data.items()]

    try:
        image_publisher(matching_img)
    except rospy.ROSInterruptException:
        pass
