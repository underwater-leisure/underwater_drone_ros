#!/usr/bin/env python


import message_filters
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray


class DiverPoseRosDummy:

    def __init__(self):
        """
        Initialize the ros node
        """

        # create camera image subscriber
        self.image_subscriber_l = message_filters.Subscriber(
            "/stereo/left/image_rect_color", Image)
        self.image_subscriber_r = message_filters.Subscriber(
            "/stereo/right/image_rect_color", Image)

        # match the message and send to callback
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_subscriber_l, self.image_subscriber_r], 10, slop=0.1)
        ts.registerCallback(self.callback)

        # create publisher for tcp client node
        self.result_string_publisher = rospy.Publisher(
            "/diver_pose/array", Float64MultiArray, queue_size=10)

    def callback(
            self,
            img_l: Image,
            img_r: Image):
        """
        This function work when subscriber recive the message. When subscriber recive the message,
        find the diver's rotation and depth. Then publish the result to TCP node.

        Args:
            img_l (Image): left image message from camera
            img_r (Image): right image message from camera
            disparity_img (Optional[DisparityImage]) :  disparity image message from stereo camera
        """
        # convert to cv from topic message
        if img_l and img_r:
            rospy.loginfo("recevie the image msg")
        else:
            rospy.loginfo("don't recevie the image msg!!!")

        prediction = [[0.] * 6] * 16

        # convert to Float64MultiArray
        prediction_msg = Float64MultiArray()
        prediction_msg.data = np.array(prediction).flatten()

        # publish the result
        rospy.loginfo(f"publish the result : {prediction}")
        self.result_string_publisher.publish(prediction_msg)


def main():

    # init the ros node
    rospy.init_node('diver_pose_ros_dummy')
    diver_pose_ros_dummy = DiverPoseRosDummy()  # noqa: F841

    # run the ros node
    rospy.spin()


if __name__ == '__main__':
    main()
