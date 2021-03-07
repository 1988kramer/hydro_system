#!/usr/bin/env python

import rospy
import numpy as np 
from hydro_system.msg import StampedFloatWithVariance

class MedianFilterNode:

  def __init__(self):
    rospy.init_node('median_filter', anonymous=True)
    self.sensor_sub = rospy.Subscriber('/raw_in',
                                       StampedFloatWithVariance,
                                       self.sensor_callback)
    self.est_pub = rospy.Publisher('/filtered_out',
                                   StampedFloatWithVariance,
                                   queue_size=1)
    self.filter_width = rospy.get_param('~width', 30)
    self.arr = np.array([])
    self.idx = 0


  def sensor_callback(self, msg):

    if self.arr.shape[0] < self.filter_width:
      self.arr = np.concatenate((self.arr, [msg.value]), axis=0)
    else:
      self.arr[self.idx] = msg.value
      self.idx += 1
      self.idx %= self.arr.shape[0]

    out_msg = StampedFloatWithVariance()
    out_msg.header = msg.header
    out_msg.value = np.median(self.arr)
    out_msg.variance = np.var(self.arr)
    self.est_pub.publish(out_msg)


if __name__ == '__main__':
  filter_node = MedianFilterNode()
  rospy.spin()