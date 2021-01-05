'''
ROS node for taking readings from the DS18B20 temperature sensor on 
raspberry pi.

Uses code from the example found at https://github.com/adafruit/Adafruit_Learning_System_Guides/blob/master/Raspberry_Pi_DS18B20_Temperature_Sensing/thermometer.py
'''

#! /usr/bin/env python

import rospy
import glob
from hydro_system.msg import TempMsg

base_dir = '/sys/bus/w1/devices/'
device_dir = glob.glob(base_dir + '28*')[0]
device_file = device_dir + '/w1_slave'
temp_pub = rospy.Publisher('/temp', TempMsg, queue_size=1)
seq = 0

def read_temp_raw():
  f = open(device_file, 'r')
  lines = f.readlines()
  f.close()
  return lines

def publish_temp():
  lines = read_temp_raw()
  while lines[0].strip()[-3:] != 'YES':
    rospy.Rate(5).sleep()
    lines = read_temp_raw()
  equals_pos = lines[1].find('t=')
  if equals_pos != -1:
    temp_string = lines[1][equals_pos+2:]
    temp = float(temp_string) / 1000.0
    temp_msg = TempMsg()
    temp_msg.temperature = temp
    temp_msg.header.stamp = rospy.Time.now() 
    msg.header.seq = seq
    temp_pub.publish(temp)
    seq += 1

if __name__ == '__main__':

  while not rospy.is_shutdown():
    try:
      publish_temp()
    except: rospy.ROSInterruptException:
      pass