#!/usr/bin/env python

'''
ROS node for taking readings from the EZO pH circuit from Atlas Scientific via i2c

borrows code from https://github.com/disaster-robotics-proalertas/atlas_ros
'''

import rospy
import numpy as np
from hydro_system.msg import StampedFloatWithVariance
from hydro_system.srv import CalibratePh, CalibratePhResponse
import smbus
import threading


class pH_Node:

  def __init__(self):
    rospy.init_node('pH', anonymous=True)

    self.seq = 0
    self.bus = smbus.SMBus(1)
    self.temp = 25.0
    self.sensor_address = 0x63 # check and verify this
    self.i2c_lock = threading.Lock()

    self.ph_pub = rospy.Publisher('/pH', 
                                  StampedFloatWithVariance, 
                                  queue_size=1)
    self.temp_sub = rospy.Subscriber('/temp', 
                                     StampedFloatWithVariance, 
                                     self.temp_callback, 
                                     queue_size=1)
    self.calib_srv = rospy.Service('calibrate_ph', 
                                   CalibratePh, 
                                   self.calibrate_ph)
    rospy.sleep(0.1)
    self.send_cmd('L,0')
    rospy.sleep(0.3)
    self.read_line()


  def read_line(self):
    try:
      response = self.bus.read_i2c_block_data(self.sensor_address, 0x00)
    except IOError as e:
      print("Error " + e.strerror + " occurred while reading from address " + str(self.sensor_address))
      return None
    respones = [i for i in response if not i == '\00']
    char_list = list(map(lambda x: chr(x & ~0x80), list(response[1:])))
    char_list = ''.join(char_list).strip('\x00').split(',')
    return [float(x) for x in char_list if x != '']

  def string_to_bytes(self, cmd):
    converted = []
    for b in cmd:
      converted.append(ord(b))
    return converted

  def send_cmd(self, cmd):
    start = ord(cmd[0])
    end = cmd[1:] + '\00'
    end = self.string_to_bytes(end)
    try:
      self.bus.write_i2c_block_data(self.sensor_address, start, end)
      return True
    except IOError as e:
      print('Error ' + e.strerror + ' occurred while writing to address ' + str(self.sensor_address))
      return None

  def get_data(self):
    self.send_cmd('RT,%.2f' % self.temp)
    rospy.sleep(0.9)
    line = self.read_line()

    return line


  def publish_ph(self):
    with self.i2c_lock:
      data = self.get_data()
    if data[0] != '':
      #rospy.loginfo('publishing pH of %.2f and temp of %.2f' % (data[0],self.temp))
      pH_msg = StampedFloatWithVariance()
      stamp = rospy.Time.now()
      pH_msg.header.stamp = stamp
      pH_msg.header.seq = self.seq
      pH_msg.value = data[0]
      self.ph_pub.publish(pH_msg)
      self.seq += 1

    else:
      rospy.loginfo('failed to get data')


  def calibrate_ph(self, req):
    rospy.loginfo('calibrating pH sensor')
    if req.type.lower() == 'clear':
      rospy.loginfo('clearing pH calibration data')
      with self.i2c_lock:
        self.send_cmd('Cal,clear')
        rospy.sleep(0.3)
        self.read_line()
      return CalibratePhResponse('Calibration completed successfully')
    elif req.type.lower() == 'mid':
      rospy.loginfo('Mid point calibration: place sensor probe in pH 7 reference solution and wait for readings to stabilize.')
    elif req.type.lower() == 'low':
      rospy.loginfo('Low point calibration: place sensor probe in pH 4 reference solution and wait for readings to stabilize.')
    elif req.type.lower() == 'high':
      rospy.loginfo('High point calibration: place sensor probe in pH 10 reference solution and wait for readings to stabilize.')

    # ensure last window_size measurements are stable
    window_size = 5
    meas = np.zeros(window_size)
    meas_idx = 0
    num_meas = 0
    with self.i2c_lock:
      while num_meas < window_size and abs(meas[meas_idx] - meas[meas > 0.0].mean()) > 0.05:

        if num_meas >= 10:
          rospy.loginfo('Calibration failed, pH reading failed to stabilize.')
          return CalibratePhResponse('Calibration failed')
        meas[meas_idx] = self.get_data()[0]
        rospy.loginfo('current: %.3f, mean: %.3f' % (meas[meas_idx], meas[meas > 0.0].mean()))
        num_meas += 1
        meas_idx = (meas_idx + 1) % window_size

      self.send_cmd('Cal,%s,%d' % (req.type,req.point))
      rospy.sleep(0.9)
      self.read_line()
    return CalibratePhResponse('Calibration completed successfully')


  def temp_callback(self, msg):

    with self.i2c_lock:
      self.temp = msg.value


if __name__ == '__main__':

  pH_node = pH_Node()

  while not rospy.is_shutdown():
    try:
      pH_node.publish_ph()
    except rospy.ROSInterruptException:
      pass
