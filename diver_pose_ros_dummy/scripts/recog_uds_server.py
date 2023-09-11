import os
import socket
import sys
import threading
import time
from binascii import unhexlify

import numpy as np

from diver_pose_ros_dummy.jenti_comm import JentiToRecogData

TEST_DATA_STR = "ABABABAB01030300A80000009A9999999999B93F9A9999999999C93F333333333333D33F9A9999999999D93" \
                "F000000000000E03F333333333333E33F666666666666E63F9A9999999999E93FCDCCCCCCCCCCEC3F000000" \
                "000000F03F9A9999999999F13F333333333333F33FCDCCCCCCCCCCF43F666666666666F63F000000000000F" \
                "83F9A9999999999F93F333333333333FB3FCDCCCCCCCCCCFC3F1300000014000000BBAABBAA"


# --- functions ---

def recv_msg():
    while True:
        recv_msg = conn.recv(1024)
        if not recv_msg:
            sys.exit(0)
        print(f"Server Recv: {recv_msg}")
        data = JentiToRecogData.from_bytes(recv_msg)  # 데이터 분할 및 데이터병합 처리를 하지 않아 에러 발생할 수 있음.
        print(data)


def send_msg():
    while True:
        time.sleep(5 + np.random.randint(6))
        conn.sendall(unhexlify(TEST_DATA_STR))
        print("Server Send O.K.")


# --- main ---

server_address = './uds_socket'

# Make sure the socket does not already exist
try:
    os.unlink(server_address)
except OSError:
    if os.path.exists(server_address):
        raise

# Create a UDS socket with address family AF_UNIX
sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

sys.stderr.write(f'starting up on {server_address}')
sock.bind(server_address)

sock.listen(1)

print("Waiting for connections")
conn, addr = sock.accept()

print("Client has connected")

# thread has to start before other loop
t = threading.Thread(target=recv_msg)
t.start()

send_msg()
