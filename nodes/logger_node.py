#!/usr/bin/env python

import rospy
from datetime import datetime
import os
from hydro_system.msg import StampedFloatWithVariance


class Logger:

  def __init__(self):

    rospy.init_node('logger', anonymous=True)
    self.subs = []
    topics_str = rospy.get_param('~topics')
    self.period = rospy.get_param('~period', 60)
    self.dir = rospy.get_param('~directory', '/home/pi/logs/')
    if self.dir[-1] != '/':
      self.dir += '/'
    self.last_times = {}

    if not os.path.isdir(self.dir):
      rospy.logerr(self.dir + ' is not a directory')
      exit()

    topics = topics_str.split()
    for topic in topics:
      self.subs.append(rospy.Subscriber(topic,
                                        StampedFloatWithVariance,
                                        self.callback,
                                        callback_args=topic))
      self.last_times[topic] = 0.0


  def callback(self, msg, topic):

    stamp = msg.header.stamp.to_sec()

    if stamp - self.last_times[topic] > self.period:

      date_str = datetime.today().strftime('_%d_%m_%Y')
      filename = self.dir + topic[1:] + date_str + '.csv'

      with open(filename, 'a') as file:
        file.write('%.4f,%.4f,%.4f\n' % (stamp, msg.value, msg.variance))
      
      self.last_times[topic] = stamp

      #os.system('rclone copy ' + filename + ' remote_logs:personal\ projects/hydroponics/logs/')


if __name__ == '__main__':
  logger = Logger()
  rospy.spin()
