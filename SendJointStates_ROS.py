#!/usr/bin/env python3

import rospy
import socket
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class SendJointStates:
    def __init__(self):
        # 1) Initialize the ROS node
        rospy.init_node('send_joints', anonymous=True)
        
        # 2) Create a UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(2.0)  # 2-second timeout for receiving any response

        # 3) Define Harmony client address
        self.harmony_address = ('192.168.2.1', 12346)
        rospy.loginfo(f"Connecting to Harmony at {self.harmony_address}")

        # 4) Publisher for any response from Harmony
        self.pub = rospy.Publisher('response_topic', String, queue_size=10)
        
        # 5) Optional: still listen on a generic "request_topic" if you want
        self.sub = rospy.Subscriber('request_topic', String, self.send_message_callback)

        # 6) NEW: Subscribe to a JointState topic 
        self.joint_sub = rospy.Subscriber('/desired_joint_states', JointState, self.joint_state_callback)

        rospy.loginfo("SendJointStates node initialized. "
                      "Listening to '/desired_joint_states' for desired joint positions.")
    
    def joint_state_callback(self, joint_msg):
        """
        Called whenever a JointState message arrives on '/desired_joint_states'.
        We'll build a 'SET RARM JOINTOVERRIDE' command with 14 values:
         position1, stiffness1, position2, stiffness2, ...
        """
        positions = list(joint_msg.position)
        # Expect 7 positions for RARM (adjust if you have more or fewer)
        if len(positions) != 7:
            rospy.logwarn("Received JointState with %d positions; expected 7 for RARM." % len(positions))
            return
        
        # For simplicity, set a fixed stiffness (e.g., 10.0 Nm/rad) for each joint.
        stiffness = 10.0

        # Build the string: "SET RARM JOINTOVERRIDE p0 s0 p1 s1 p2 s2 ... p6 s6"
        override_values = []
        for p in positions:
            override_values.append(str(p))       # position in rad
            override_values.append(str(stiffness))  # fixed stiffness

        # Join them into a single space-separated string
        override_str = " ".join(override_values)

        # Our final command to Harmony:
        # e.g.  "SET RARM JOINTOVERRIDE 0.1 10.0 -0.2 10.0 0.3 10.0 ..."
        command = f"SET RARM JOINTOVERRIDE {override_str}"
        rospy.loginfo(f"Sending override command: {command}")

        # Send it via UDP
        self.send_udp_command(command)

    def send_message_callback(self, ros_msg):
        """
        (Unchanged from your skeleton)
        Allows any arbitrary commands (String) published to 'request_topic'.
        """
        command = ros_msg.data
        rospy.loginfo(f"[HarmonyBridge] Sending command to Harmony: {command}")
        self.send_udp_command(command)

    def send_udp_command(self, command):
        """
        Helper function to send a command string over UDP, then publish the response.
        """
        try:
            # Send command
            self.sock.sendto(command.encode(), self.harmony_address)
            
            # Attempt to receive a response
            response_data, addr = self.sock.recvfrom(4096)
            decoded_response = response_data.decode()
            
            rospy.loginfo(f"[HarmonyBridge] Received response from {addr}: {decoded_response}")
            self.pub.publish(decoded_response)
        
        except socket.timeout:
            rospy.logerr("Timed out waiting for a response from Harmony.")
        except Exception as e:
            rospy.logerr(f"Unexpected error while communicating with Harmony: {e}")


if __name__ == '__main__':
    try:
        SendJointStates()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
