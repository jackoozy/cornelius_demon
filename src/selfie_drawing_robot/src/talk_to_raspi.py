import roslibpy

class RobotController:
    def __init__(self, host, port):
        self.client = roslibpy.Ros(host=host, port=port)
        self.joint_state_subscriber = None
        self.control_publisher = None

    def connect(self):
        try:
            self.client.run()
        except Exception as e:
            print(f"Error connecting to ROS: {e}")
        finally:
            self.client.terminate()

    def subscribe_to_joint_states(self, topic_name='/joint_states', message_type='sensor_msgs/JointState'):
        self.joint_state_subscriber = roslibpy.Topic(self.client, topic_name, message_type)
        self.joint_state_subscriber.subscribe(self._joint_state_callback)

    def _joint_state_callback(self, message):
        print(message)

    def create_control_publisher(self, topic_name='/scaled_pos_joint_traj_controller/command', message_type='trajectory_msgs/JointTrajectory'):
        self.control_publisher = roslibpy.Topic(self.client, topic_name, message_type)
        self.control_publisher.advertise()

    def send_joint_trajectory(self, joint_names, positions, time_from_start):
        joint_trajectory_msg = {
            'joint_names': joint_names,
            'points': [{
                'positions': positions,
                'time_from_start': {'secs': time_from_start[0], 'nsecs': time_from_start[1]}
            }]
        }
        self.control_publisher.publish(roslibpy.Message(joint_trajectory_msg))

if __name__ == "__main__":
    # Replace with Raspberry Pi IP address and port
    host = '192.168.27.1'
    port = 9090
    
    controller = RobotController(host, port)
    
    controller.connect()
    controller.subscribe_to_joint_states()
    
    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    positions = [1.57, -1.57, 0.0, -1.57, 0.0, 0.0]
    time_from_start = (5, 0)  # 5 seconds
    
    controller.create_control_publisher()
    controller.send_joint_trajectory(joint_names, positions, time_from_start)
    
    # Keep the script running to maintain the connection
    while True:
        pass
