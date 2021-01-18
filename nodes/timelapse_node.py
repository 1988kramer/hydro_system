#!/usr/bin/env python
'''
Node to take photos with a usb webcam at a specified interval 
for timelapse photography
'''

import cv2
import datetime
import rospy
import os

class Timelapse:
  def __init__():
    self.camera = cv2.VideoCapture(0)
    self.interval = rospy.get_param('interval', 600) # default 10 min interval
    self.intensity_thresh = rospy.get_param('thresh', 32)
    self.last_time = -1.0

  def take_image():
    _, image = self.camera.read()

    # check if average intensity is above threshold
    gray_im = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mean = cv2.mean(gray_im)[0]

    stamp = rospy.Time.now().to_sec()

    if mean >= self.intensity_thresh and stamp - self.last_time > self.interval:
      self.last_time = stamp
      filename = '/logs/images/%d.jpg' % int(stamp)
      cv2.imwrite(filename, image) 


if __name__ == '__main__':

  timelapse_node = Timelapse()

  while not rospy.is_shutdown():
    try:
      timelapse_node.take_image()
    except rospy.ROSInterruptException:
      pass