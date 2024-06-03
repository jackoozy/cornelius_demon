#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import threading
import socket

class ROSNode:
    def __init__(self):
        self.node_name = "cornelius_node"
        self.publisher = rospy.Publisher('chatter', String, queue_size=10)

    def start(self):
        rospy.init_node(self.node_name, anonymous=True)
        self.keep_alive_timer = rospy.Timer(rospy.Duration(1), self.keep_alive)
        self.socket_thread = threading.Thread(target=self.socket_server)
        self.socket_thread.start()

    def keep_alive(self, event):
        pass

    def publish_message(self, message):
        rospy.loginfo(message)
        self.publisher.publish(String(data=message))

    def socket_server(self):
        host = '127.0.0.1'
        port = 65432
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((host, port))
            s.listen()
            print(f"Socket server started on {host}:{port}")
            while not rospy.is_shutdown():
                conn, addr = s.accept()
                with conn:
                    print(f"Connected by {addr}")
                    while True:
                        data = conn.recv(1024)
                        if not data:
                            break
                        message = data.decode('utf-8')
                        self.publish_message(message)

def ros_spin_thread():
    rospy.spin()

if __name__ == "__main__":
    ros_node = ROSNode()
    ros_node.start()

    ros_spin = threading.Thread(target=ros_spin_thread)
    ros_spin.start()
