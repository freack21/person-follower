#!/usr/bin/env python3

import rospy
import socketio
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import numpy as np

class SocketNode :
  def __init__(self) :
    # Initialize ROS node
    rospy.init_node('socket_node', anonymous=True)

    self.initGlobalVars()
    self.initSocketIO()
    self.initROS()



  def initGlobalVars(self) :
    self.myUsername = "Miria"
    # self.alamatIP = "http://10.234.218.255:3000/"
    self.alamatIP = "http://172.16.252.187:3000/"
    # self.alamatIP = "http://localhost:3000/"

    self.isRequestedVision = False


  def initSocketIO(self) :
    # Initialize Socket.IO client
    self.sio = socketio.Client()
    # Listen to Socket.IO event
    self.sio.on("connect", self.on_connect)
    self.sio.on("perintah", self.on_perintah)
    self.sio.on("disconnect", self.on_disconnect)
    # Start the Socket.IO client
    self.sio.connect(self.alamatIP)


  def initROS(self) :
    # Create a ROS publisher to send command to Arduino
    self.command_pub = rospy.Publisher('command_topic', String, queue_size=50)

    # Create a ROS pub and sub to send data to camera_package
    self.vision_pub = rospy.Publisher('vision_topic', String, queue_size=50)
    self.vision_sub = rospy.Subscriber('vision_topic', String, self.vision_callback)

    # Create a ROS subscriber to receive cmd_status data
    self.cmd_status_sub = rospy.Subscriber('cmd_status_topic_', String, self.cmd_status_callback)


  # callback untuk mengirim data ke server
  def vision_callback(self, data):
    rospy.loginfo('[SocketNode] vision: %s', data.data)

    # Extract the vision data from the received message
    vision_data = data.data

    if vision_data == "stop" :
      self.isRequestedVision = False
      self.sio.emit('cmd_status', { 'msg': 'Selesai mengerjakan perintah!', 'isDone': True })
    else :
      self.isRequestedVision = True


  # callback untuk mengirim data ke server
  def cmd_status_callback(self, data):
    rospy.loginfo('cmd_status: %s', data.data)

    # Extract the cmd_status data from the received message
    cmd_status_data = data.data

    if cmd_status_data == "received":
      return self.sio.emit("received_cmd")

    # Access individual elements of the cmd_status data
    [msg, isDone] = cmd_status_data.split("|")

    # Publish the cmd_status data to Basestation
    self.sio.emit('cmd_status', { 'msg': msg, 'isDone': (int(isDone) == 1) })


  def isVisionCommand(self, cmd) :
    if cmd.split("|")[0] == 'vision' :
      return True

    return False



  def on_connect(self):
    rospy.loginfo(f'[SocketNode] "{self.myUsername}" connected to Socket.IO server!')
    self.sio.emit('join', self.myUsername)


  def on_perintah(self, command):
    rospy.loginfo('[SocketNode] perintah: %s', command)
    ros_data = String()

    self.sio.emit("received_perintah", { 'command': command })

    if self.isRequestedVision and command != 'reset' and (not 'stop' in command) :
      msg = "[SocketNode] Sedang menjalankan perintah!"
      self.sio.emit('cmd_status', { 'msg': msg, 'isDone': False })
      return

    if command == "reset" :
      ros_data.data = "stop"
      self.vision_pub.publish(ros_data)
    elif self.isVisionCommand(command) :
      ros_data.data = command.split("|")[1]
      self.vision_pub.publish(ros_data)

    ros_data.data = command
    self.command_pub.publish(ros_data)


  def on_disconnect(self):
    rospy.loginfo(f'[SocketNode] {self.myUsername} disconnected from Socket.IO server!')


  def run(self) :
    # Run the ROS node
    rospy.spin()
    # Disconnect from the Socket.IO server when ROS node is shutdown
    self.sio.disconnect()


if __name__ == '__main__':
  try:
    node = SocketNode()
    node.run()
  except rospy.ROSInterruptException:
    pass