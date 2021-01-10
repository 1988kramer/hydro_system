#!/usr/bin/env python

'''
ROS node for taking readings from the DS18B20 temperature sensor on 
raspberry pi.

Uses code from the example found at https://github.com/adafruit/Adafruit_Learning_System_Guides/blob/master/Raspberry_Pi_DS18B20_Temperature_Sensing/thermometer.py
'''

import rospy
import glob
import numpy as np
import threading
from datetime import datetime
from hydro_system.msg import TempMsg
from std_srvs.srv import Empty, EmptyResponse

class DS18B20_Node:
  def __init__(self):

    rospy.init_node('temp', anonymous=True)
    self.srv = rospy.Service('save_log', Empty, save_log)

    base_dir = '/sys/bus/w1/devices/'
    device_dir = glob.glob(base_dir + '28*')[0]
    self.device_file = device_dir + '/w1_slave'
    self.temp_pub = rospy.Publisher('/temp', TempMsg, queue_size=1)
    self.log = []
    self.lock = threading.Lock()
    self.seq = 0

  def read_temp_raw(self):
    f = open(self.device_file, 'r')
    lines = f.readlines()
    f.close()
    return lines

  def publish_temp(self):
    lines = read_temp_raw()
    while lines[0].strip()[-3:] != 'YES':
      rospy.Rate(5).sleep()
      lines = read_temp_raw()
    equals_pos = lines[1].find('t=')
    if equals_pos != -1:
      temp_string = lines[1][equals_pos+2:]
      temp = float(temp_string) / 1000.0
      stamp = rospy.Time.now()
      temp_msg = TempMsg()
      temp_msg.temperature = temp
      temp_msg.header.stamp = stamp
      temp_msg.header.seq = self.seq
      self.temp_pub.publish(temp_msg)
      self.seq += 1

      # log roughly once per minute
      if len(self.log) == 0 or stamp.to_sec() - self.log[-1][0] > 60.0: 
        with self.lock:
          self.log.append([stamp.to_sec(),temp])

      # dump log to file at least weekly
      if self.log[-1][0] - self.log[0][0] > 604800.0: 
        req = Empty()
        self.save_log(req)


  def save_log(self,req):
    with self.lock:
      log_mat = np.array(self.log)
      log = []
    date_str = datetime.today().strftime('%d_%m_%Y')
    np.save('/home/pi/logs/temperature_' + date_str + '.npy', log_mat)
    return []

if __name__ == '__main__':
  
  node = DS18B20_Node()

  while not rospy.is_shutdown():
    try:
      node.publish_temp()
    except rospy.ROSInterruptException:
      pass
