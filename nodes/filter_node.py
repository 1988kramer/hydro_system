#!/usr/bin/env python

import rospy
import numpy as np 
from hydro_system.msg import StampedFloatWithVariance

class FilterNode:

  def __init__(self):
    rospy.init_node('filter', anonymous=True)
    self.sensor_sub = rospy.Subscriber('/raw_in', 
                                       StampedFloatWithVariance,
                                       self.sensor_callback)
    self.est_pub = rospy.Publisher('/filtered_out',
                                   StampedFloatWithVariance,
                                   queue_size=1)
    self.x = -1
    self.q = rospy.get_param('~q')
    self.r = rospy.get_param('~r')
    self.var = rospy.get_param('~var')
    self.chi2 = rospy.get_param('~chi2', 0.0)
    self.last_t = 0.0


  def sensor_callback(self, msg):

    stamp = msg.header.stamp.to_sec()

    if self.x == -1:
      self.x = msg.value
      self.last_t = stamp
    else:

      y_tilde = msg.value - self.x

      # reject outliers given a chi^2 confidence value
      if self.chi2 == 0.0 or (y_tilde**2 / self.var) <= self.chi2:
        dt = stamp - self.last_t
        self.last_t = stamp
        var_hat = self.var + self.q * dt
        S = var_hat + self.r
        K = var_hat / S
        x_hat = self.x + K * y_tilde
        self.var = (1.0 - K) * var_hat
        self.x = x_hat

    out_msg = StampedFloatWithVariance()
    out_msg.header = msg.header
    out_msg.value = self.x
    out_msg.variance = self.var
    self.est_pub.publish(out_msg)


if __name__ == '__main__':
  filter_node = FilterNode()
  rospy.spin()
