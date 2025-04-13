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
    self.cap = cv2.VideoCapture(2)
    if not self.cap.isOpened():
      raise Exception("[CameraNode] No Camera")

    # Load model SSD MobileNet TFLite
    self.interpreter = tf.lite.Interpreter(model_path=os.path.join(os.path.dirname(__file__), "model/detect.tflite"))
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

      self.lastCmd = ""
      ros_data = String()
      ros_data.data = "berhenti"
      self.command_pub.publish(ros_data)
    else :
      self.isRequested = True
      self.requestedService = vision_data


  def followPerson(self, label, x_margin, distance_up):
    if label != 0 :
      return

    ros_data = String()
    cmd = ""

    if np.abs(distance_up) <= 16 :
      cmd = "berhenti"
    elif np.abs(x_margin) <= 64 :
      cmd = "maju"
    elif x_margin < 0 :
      cmd = "putar_kanan"
    elif x_margin > 0 :
      cmd = "putar_kiri"

    if self.lastCmd == cmd :
      return

    self.lastCmd = cmd
    ros_data.data = cmd
    self.command_pub.publish(ros_data)
    # rospy.loginfo('[CameraNode] followPerson: %s', cmd)


  def run(self) :
    while not rospy.is_shutdown():
      # Capture frame from the webcam
      ret, frame = self.cap.read()

      fps = self.cap.get(cv2.CAP_PROP_FPS)

      if not ret:
        break

      if self.isRequested :
        outname = self.output_details[0]['name']

        if ('StatefulPartitionedCall' in outname): # This is a TF2 model
            boxes_idx, classes_idx, scores_idx = 1, 3, 0
        else: # This is a TF1 model
            boxes_idx, classes_idx, scores_idx = 0, 1, 2

        height = self.input_details[0]['shape'][1]
        width = self.input_details[0]['shape'][2]

        floating_model = (self.input_details[0]['dtype'] == np.float32)

        input_mean = 127.5
        input_std = 127.5

        # Preprocessing gambar (sesuaikan ukuran input sesuai model)
        frame = cv2.flip(frame, 1)
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_resized = cv2.resize(img_rgb, (width, height))

        input_data = np.expand_dims(img_resized, axis=0)

        # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
        if floating_model:
            input_data = (np.float32(input_data) - input_mean) / input_std

        # Set input tensor
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)

        _time_mulai = time.time()

        # Jalankan inferensi
        self.interpreter.invoke()

        waktu_proses = time.time() - _time_mulai
        fps = 1 / waktu_proses
        
        # Ambil hasil deteksi
        boxes = self.interpreter.get_tensor(self.output_details[boxes_idx]['index'])[0]  # Bounding box koordinat
        classes = self.interpreter.get_tensor(self.output_details[classes_idx]['index'])[0]  # Kelas objek
        scores = self.interpreter.get_tensor(self.output_details[scores_idx]['index'])[0]   # Skor keyakinan

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
          cv2.putText(frame, f"Down: {distance_down}px", (center_x + 10, height - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

          label = f"Class {int(classes[heighest_index])} - {int(scores[heighest_index] * 100)}%"
          cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

          x_margin = distance_left - distance_right

          if self.requestedService == "followPerson" :
            self.followPerson(classes[heighest_index], x_margin, distance_up)

        elif self.lastCmd != "berhenti" :
          self.lastCmd = "berhenti"
          ros_data = String()
          ros_data.data = "berhenti"
          self.command_pub.publish(ros_data)


        # fps = cap.get(cv2.CAP_PROP_FPS)
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

      elif self.lastCmd != "" :
        cmd = "berhenti"

        if self.lastCmd != cmd :
          self.lastCmd = ""
          ros_data = String()
          ros_data.data = cmd
          self.command_pub.publish(ros_data)
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