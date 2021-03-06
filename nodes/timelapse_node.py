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
  def __init__(self):
    rospy.init_node('timelapse', anonymous=True)
    self.interval = rospy.get_param('~interval', 600)
    self.intensity_thresh = 32

  def take_image(self):
    start_time = rospy.Time.now()
    camera = cv2.VideoCapture(0)
    _, image = camera.read()
    stamp = rospy.Time.now().to_sec()
    del(camera)

    # check if average intensity is above threshold
    gray_im = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mean = cv2.mean(gray_im)[0]
    rospy.loginfo('mean image intensity: %d' % mean)

    if mean >= self.intensity_thresh:
      rospy.loginfo('saving image')
      filename = '/home/pi/logs/images/%d.jpg' % int(stamp)
      cv2.imwrite(filename, image)
      os.system('rclone copy ' + filename + ' remote_logs:personal\ projects/hydroponics/logs/images/')

    duration = rospy.Time.now() - start_time
    rospy.sleep(self.interval - duration.to_sec())


if __name__ == '__main__':

  timelapse_node = Timelapse()

  while not rospy.is_shutdown():
    try:
      timelapse_node.take_image()
    except rospy.ROSInterruptException:
      pass
