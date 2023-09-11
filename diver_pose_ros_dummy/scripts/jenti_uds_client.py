# ref: https://blog.furas.pl/python-socket-send-and-receive-at-the-same-time-gb.html

import argparse
import socket
import sys
import threading
from typing import Callable

import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray

from diver_pose_ros_dummy.jenti_comm import (RECOG_TO_JENTI_MSG_SIZE, SOT,  # noqa: I101
                                             JentiToRecogData,
                                             RecogToJentiData)

buffer_size = 1024
SOT_BYTE = SOT.to_bytes(4, byteorder='little')


def recv_msg(sock):
    feed_data = b""
    while True:
        while len(feed_data) < RECOG_TO_JENTI_MSG_SIZE:
            new_feed_data = sock.recv(buffer_size)
            feed_data = feed_data + new_feed_data

        sot_index = feed_data.find(SOT_BYTE)
        if sot_index >= 0:
            feed_data = feed_data[sot_index:]
            if len(feed_data) >= RECOG_TO_JENTI_MSG_SIZE:
                one_data = feed_data[:RECOG_TO_JENTI_MSG_SIZE]
                recog_to_jenti_data = RecogToJentiData.from_bytes(one_data)
                print(recog_to_jenti_data)
                feed_data = feed_data[RECOG_TO_JENTI_MSG_SIZE:]
        else:
            print(f"skip bytes:{len(feed_data)} -> {len(feed_data[-3:])}")
            feed_data = feed_data[-3:]  # SOT 가 나타나지 않았으므로, 3byte 만 남기고 버림.


class TcpClient:
    def __init__(
            self,
            server_addr: str):
        """
        This is Jenti TCP Client module.

        Args:
            server_addr (str): TCP sever address
        """

        # init the socket
        self.server_addr = server_addr

        print("\nRunning feed_proc ... ")
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

        try:
            print(f'connecting to server: {server_addr}')
            self.sock.connect(server_addr)
        except socket.error as msg:
            print(msg)
            sys.exit(1)

    def start_threading(self):
        """
        thread has to start before other loop
        """
        t = threading.Thread(target=recv_msg, args=(self.sock,))
        t.start()

    def send_data(
            self,
            data: bytes):
        """
        Send byte data that is result from pipeline model.

        Args:
            data (bytes): byte data that is result from pipeline model
        """
        self.sock.sendall(data)
        print("message sent")

    def close(self):
        """
        Close the Jenti Client
        """
        self.sock.close()


def callback(
        data: Float64MultiArray,
        client: Callable):
    """
    This function work when subscriber recive the message. When subscriber recive the message,
    message is convert to bytes

    Args:
        data (Float64MultiArray): _description_
        client (Callable): _description_
    """
    data = np.array(data.data).reshape(16, 6).tolist()
    jenti_to_recog_data = JentiToRecogData(
        diver_positions=data)
    client.send_data(jenti_to_recog_data.to_bytes())


def listener(args):

    # init the ros node
    rospy.init_node('tcp_node')

    # init the TCP client
    server_addr = args.server_addr
    client = TcpClient(server_addr)
    client.start_threading()

    # ensure the socket is closed when ROS node is shutdown
    rospy.on_shutdown(client.close)

    # make Subscriber from model pipeline node
    rospy.Subscriber('/diver_pose/array',
                     Float64MultiArray, callback, callback_args=client)

    # run the ros node
    rospy.spin()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="ROS node with argument parsing")
    parser.add_argument(
        '--server_addr', type=str,
        default="./uds_socket", help="TCP soket")
    args = parser.parse_args()

    listener(args)
