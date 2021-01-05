'''
ROS node for taking readings from the EZO pH circuit from Atlas Scientific via i2c

borrows code from https://github.com/disaster-robotics-proalertas/atlas_ros

need to add temperature compensation!!!
'''

#!/usr/bin/env python

import rospy
import numpy as np
from hydro_system.msg import PhMsg
from hydro_system.srv import CalibratePh, CalibratePhResponse
import smbus

ph_pub = rospy.Publisher('/pH', PhMsg, queue_size=1)
seq = 0
bus = smbus.SMBus(1)
sensor_address = 66 # check and verify this
calibrating = False


def read_line(address):
  try:
    response = bus.read_i2c_block_data(address, 0x00)
  except IOError as e:
    print("Error " + e.strerror + " occurred while reading from address " + str(address))
    return None
  respones = [i for i in response if not i == '\00']
  char_list = list(map(lambda x: chr(x & ~0x80), list(response[1:])))
  char_list = ''.join(char_list).strip('\x00').split(',')
  return [float(x) for x in char_list]

def string_to_bytes(cmd):
  converted = []
  for b in cmd:
    convert.append(ord(b))
  return converted

def send_cmd(cmd, address):
  start = ord(cmd[0])
  end = cmd[1:] + '\00'
  end = string_to_bytes(end)
  try:
    bus.write_12c_block_data(address, start, end)
    return True
  except IOError as e:
    print('Error ' + e.strerror + ' occurred while writing to address ' + str(address))
    return None

def get_data(address):
  send_cmd('R', address)
  rospy.sleep(1.0)
  line = self.read_line(address)

  return line


def publish_ph():
  if not calibrating:
    data = get_data(ph_address)
    pH_msg = PhMsg()
    pH_msg.header.stamp = rospy.Time.now()
    pH_msg.header.seq = seq
    pH_msg.pH = data[0]
    ph_pub.publish(pH_msg)
    seq += 1


def calibrate_ph(req):
  rospy.loginfo('calibrating pH sensor')
  calibrating = True
  rospy.sleep(5.0)
  if req.type.lower() == 'clear':
    rospy.loginfo('clearing pH calibration data')
    send_cmd('Cal,clear', ph_address)
    calibrating = False
    return CalibratePhResponse('Calibration completed successfully')
  elif req.type.lower() == 'mid':
    rospy.loginfo('Mid point calibration: place sensor probe in pH 7 reference solution and wait for readings to stabilize.')
  elif req.type.lower() == 'low':
    rospy.loginfo('Low point calibration: place sensor probe in pH 4 reference solution and wait for readings to stabilize.')
  elif req.type.lower() == 'high':
    rospy.loginfo('High point calibration: place sensor probe in pH 10 reference solution and wait for readings to stabilize.')

  mean = 0.0
  current = 10.0
  num_meas = 0.0
  while abs(current - mean) > 0.05:

    if num_meas >= 10:
      rospy.loginfo('Calibration failed, pH reading failed to stabilize.')
      return CalibratePhResponse('Calibration failed')
    current = get_data(ph_address)[0]
    mean *= num_meas
    mean += current
    num_meas += 1.0
    mean /= num_meas

  send_cmd('Cal,%s,%d' % (req.type,req.point), ph_address)
  calibrating = False
  return CalibratePhResponse('Calibration completed successfully')


if __name__ == '__main__':

  while not rospy.is_shutdown():
    try:
      publish_ph()
    except: rospy.ROSInterruptException:
      pass