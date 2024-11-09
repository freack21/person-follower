#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import cv2
import numpy as np
import tensorflow as tf
import os
import time

class CameraNode :
  def __init__(self):
    # Initialize ROS node
    rospy.init_node('camera_node', anonymous=True)

    self.initGlobalVars()
    self.initCamAndModel()
    self.initROS()


  def initGlobalVars(self) :
    self.isRequested = False
    self.requestedService = ""
    self.lastCmd = ""


  def initCamAndModel(self) :
    # Open the webcam
    self.cap = cv2.VideoCapture(0)
    if not self.cap.isOpened():
      raise Exception("[CameraNode] No Camera")

    # Load model SSD MobileNet TFLite
    self.interpreter = tf.lite.Interpreter(model_path=os.path.join(os.path.dirname(__file__), "model/model1.tflite"))
    self.interpreter.allocate_tensors()

    # Dapatkan informasi input dan output
    self.input_details = self.interpreter.get_input_details()
    self.output_details = self.interpreter.get_output_details()


  def initROS(self) :
    # Create a ROS publisher to send data to Arduino
    self.command_pub = rospy.Publisher('command_topic', String, queue_size=50)

    # Create a ROS pub and sub to send data to camera_package
    self.vision_pub = rospy.Publisher('vision_topic', String, queue_size=50)
    self.vision_sub = rospy.Subscriber('vision_topic', String, self.vision_callback)



  # callback untuk mengirim data ke server
  def vision_callback(self, data):
    rospy.loginfo('[CameraNode] vision: %s', data.data)

    # Extract the vision data from the received message
    vision_data = data.data

    if vision_data == "stop" :
      self.isRequested = False
      self.requestedService = ""
    else :
      self.isRequested = True
      self.requestedService = vision_data


  def followPerson(self, label, x_margin, y_margin):
    if label != 0 :
      return

    ros_data = String()
    cmd = ""

    if np.abs(x_margin) <= 3 and np.abs(y_margin) <= 4 :
      ros_data.data = "hidupkanPenggiring"
      self.command_pub.publish(ros_data)

      cmd = "berhenti"
      self.lastCmd = ""
      ros_data.data = cmd
      self.command_pub.publish(ros_data)

      ros_data.data = "stop"
      self.vision_pub.publish(ros_data)
      return
    elif np.abs(y_margin) <= 64 :
      if np.abs(x_margin) <= 9 :
        cmd = "majuPelan"
      else :
        if x_margin < 0 :
          cmd = "putarKiriPelan"
        else :
          cmd = "putarKananPelan"

    elif np.abs(x_margin) <= 16 :
      if y_margin < 0 :
        if y_margin >= -80 :
          cmd = "maju"
      elif y_margin > 0 :
        if x_margin < 0 :
          cmd = "putarKiriPelan"
        else :
          cmd = "putarKananPelan"

    elif np.abs(x_margin) <= 32 :
      if y_margin < 0 :
        if y_margin >= -128 :
          cmd = "maju"
      elif y_margin > 0 :
        if x_margin < 0 :
          cmd = "putarKiri"
        else :
          cmd = "putarKanan"

    elif y_margin < -128 :
      if np.abs(x_margin) <= 64 :
        cmd = "maju"
      else :
        if x_margin < 0 :
          cmd = "putarKiri"
        else :
          cmd = "putarKanan"

    elif x_margin < 0 :
      if x_margin >= -48 :
        cmd = "putarKiriPelan"
      else :
        cmd = "putarKiri"
    elif x_margin > 0 :
      if x_margin <= 48 :
        cmd = "putarKananPelan"
      else :
        cmd = "putarKanan"

    if self.lastCmd == cmd :
      return

    self.lastCmd = cmd
    ros_data.data = cmd
    self.command_pub.publish(ros_data)
    if cmd == "majuPelan" :
      ros_data.data = "hidupkanPenggiring"
      self.command_pub.publish(ros_data)


  def run(self) :
    while not rospy.is_shutdown():
      # Capture frame from the webcam
      ret, frame = self.cap.read()

      fps = self.cap.get(cv2.CAP_PROP_FPS)

      if not ret:
        break

      if self.isRequested :
        # Preprocessing gambar (sesuaikan ukuran input sesuai model)
        input_shape = self.input_details[0]['shape']
        img_resized = cv2.resize(frame, (input_shape[2], input_shape[1]))
        img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)

        # Konversi ke tipe data UINT8
        img_uint8 = img_rgb.astype(np.uint8)

        # Set input tensor
        self.interpreter.set_tensor(self.input_details[0]['index'], [img_uint8])    

        _time_mulai = time.time()

        # Jalankan inferensi
        self.interpreter.invoke()

        waktu_proses = time.time() - _time_mulai
        fps = 1 / waktu_proses
        
        # Ambil hasil deteksi
        boxes = self.interpreter.get_tensor(self.output_details[0]['index'])[0]  # Bounding box koordinat
        classes = self.interpreter.get_tensor(self.output_details[1]['index'])[0]  # Kelas objek
        scores = self.interpreter.get_tensor(self.output_details[2]['index'])[0]   # Skor keyakinan

        # Ambil dimensi frame untuk penyesuaian bounding box
        height, width, _ = frame.shape

        heighest_index = 0

        # Visualisasi deteksi pada frame
        for i in range(len(scores)):
          if scores[i] > scores[heighest_index]:  # Hanya tampilkan jika skor > 0.5
              heighest_index = i

        if scores[heighest_index] > 0:  # Hanya tampilkan jika skor > 0.5
          ymin, xmin, ymax, xmax = boxes[heighest_index]
          start_point = (int(xmin * width), int(ymin * height))
          end_point = (int(xmax * width), int(ymax * height))

          # Gambar bounding box dan label pada gambar
          cv2.rectangle(frame, start_point, end_point, (0, 255, 0), 2)

          x1,y1 = start_point
          x2,y2 = end_point

          distance_left = x1
          distance_right = width - x2
          center_y = (y1 + y2) // 2
          
          cv2.line(frame, (x1, center_y), (0, center_y), (255, 0, 0), 2)  # Line to the left edge
          cv2.line(frame, (x2, center_y), (width, center_y), (255, 0, 0), 2)  # Line to the right edge
          # Display the distances at the end of each line
          cv2.putText(frame, f"Left: {distance_left}px", (10, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
          cv2.putText(frame, f"Right: {distance_right}px", (width - 120, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

          # Calculate distances to the left and right frame edges
          distance_up = y1
          distance_down = height - y2  # height is the height of the frame

          # Draw lines from the object's center to the left and right edges of the frame
          center_x = (x1 + x2) // 2
          cv2.line(frame, (center_x, y1), (center_x, 0), (255, 0, 0), 2)  # Line to the left edge
          cv2.line(frame, (center_x, y2), (center_x, height), (255, 0, 0), 2)  # Line to the right edge

          # Display the distances at the end of each line
          cv2.putText(frame, f"Up: {distance_up}px", (center_x + 10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
          cv2.putText(frame, f"Down: {distance_down}px", (center_x + 10, height - 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

          label = f"Class {int(classes[heighest_index])} - {int(scores[heighest_index] * 100)}%"
          cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # fps = cap.get(cv2.CAP_PROP_FPS)
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

      # Tampilkan frame
      cv2.imshow("SSD MobileNet TFLite Detection", frame)

      # Tekan 'q' untuk keluar
      if cv2.waitKey(1) & 0xFF == ord('q'):
          break

    # Release the camera and close windows
    self.cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
  try:
    node = CameraNode()
    node.run()
  except rospy.ROSInterruptException:
    pass