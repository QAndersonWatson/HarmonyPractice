#!/usr/bin/env python3

import rospy
import socket
from std_msgs.msg import String

class SendJointStates:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('send_joints', anonymous=True)
        
        # Create a UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(2.0)  # Time in seconds to wait for a response

        # Define Harmony client address
        self.harmony_address = ('192.168.2.1', 12346)
        rospy.loginfo(f"Connecting to Harmony at {self.harmony_address}")

        # Create a Publisher for the response
        self.pub = rospy.Publisher('response_topic', String, queue_size=10)
        
        # Subscribe to the request topic
        self.sub = rospy.Subscriber('request_topic', String, self.send_message_callback)
        
        rospy.loginfo("SendJointStates node initialized. Waiting for messages on 'request_topic'.")

    def send_message_callback(self, ros_msg):
        """Callback function that sends the received ROS String message to Harmony and publishes the response."""
        # Extract the command from the ROS message
        command = ros_msg.data
        
        rospy.loginfo(f"[HarmonyBridge] Sending command to Harmony: {command}")
        
        try:
            # Send command via UDP
            self.sock.sendto(command.encode(), self.harmony_address)
            
            # Receive a response (up to 4096 bytes)
            response_data, addr = self.sock.recvfrom(4096)
            decoded_response = response_data.decode()
            
            # Log and publish the response
            rospy.loginfo(f"[HarmonyBridge] Received response from {addr}: {decoded_response}")
            self.pub.publish(decoded_response)
        
        except socket.timeout:
            rospy.logerr("Timed out waiting for a response from Harmony.")
        except Exception as e:
            rospy.logerr(f"Unexpected error while communicating with Harmony: {e}")

if __name__ == '__main__':
    try:
        # Instantiate the node class and keep it alive
        SendJointStates()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

