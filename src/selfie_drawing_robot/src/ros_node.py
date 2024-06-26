#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import threading
import socket
import subprocess

class ROSNode:
    def __init__(self):
        self.node_name = "cornelius_node"
        self.publisher = rospy.Publisher('gui_com', String, queue_size=10)
        self.subscriber = rospy.Subscriber('gui_com', String, self.message_callback)
        self.last_message = ""
        
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

    def message_callback(self, msg):
        rospy.loginfo(f"Received message: {msg.data}")
        self.last_message = msg.data
        self.send_last_message_via_socket()  # Send the last message to the UI

    def getLastMessage(self):
        return self.last_message

    def send_last_message_via_socket(self):
        host = '127.0.0.1'
        port = 65433  # Use a different port to avoid conflict with the existing server
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((host, port))
                s.sendall(self.last_message.encode('utf-8'))
        except Exception as e:
            rospy.logerr(f"Failed to send message via socket: {e}")

    def kill_process_using_port(self, port):
        try:
            result = subprocess.run(['sudo', 'lsof', '-t', f'-i:{port}'], capture_output=True, text=True)
            pids = result.stdout.split()
            for pid in pids:
                subprocess.run(['sudo', 'kill', '-9', pid])
            if pids:
                print(f"Killed process(es) using port {port}: {', '.join(pids)}")
        except Exception as e:
            print(f"Failed to kill process using port {port}: {e}")

    def socket_server(self):
        host = '127.0.0.1'
        port = 65432

        # Ensure the port is free before binding
        self.kill_process_using_port(port)

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((host, port))
            s.listen()
            s.settimeout(5)  # Set a timeout of 5 seconds for the socket operations
            print(f"Socket server started on {host}:{port}")
            while not rospy.is_shutdown():
                try:
                    conn, addr = s.accept()
                    with conn:
                        print(f"Connected by {addr}")
                        while True:
                            data = conn.recv(1024)
                            if not data:
                                break
                            message = data.decode('utf-8')
                            self.publish_message(message)
                except socket.timeout:
                    continue  # If a timeout occurs, continue the loop to check for shutdown

def ros_spin_thread():
    rospy.spin()

if __name__ == "__main__":
    ros_node = ROSNode()
    ros_node.start()

    ros_spin = threading.Thread(target=ros_spin_thread)
    ros_spin.start()
