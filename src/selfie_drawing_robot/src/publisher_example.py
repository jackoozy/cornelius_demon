#!/home/lucas/cornEnv/bin/python

import rospy

def main():
    print("Before rospy.init_node")
    try:
        rospy.init_node('test_node', anonymous=True)
        print("After rospy.init_node")
    except Exception as e:
        print(f"Exception during rospy.init_node: {e}")
    rospy.loginfo("Test node started")
    rospy.spin()

if __name__ == "__main__":
    main()
