#!/usr/bin/env python

'''
ROS node for taking readings from the SHT31D temperature and humidity sensor.
borrows from https://github.com/ralf1070/Adafruit_Python_SHT31/blob/master/Adafruit_SHT31.py
'''

import rospy
from hydro_system.msg import StampedFloatWithVariance
import smbus

SHT31_MEAS_HIGHREP_STRETCH = 0x2C06
SHT31_MEAS_MEDREP_STRETCH = 0x2C0D
SHT31_MEAS_LOWREP_STRETCH = 0x2C10
SHT31_MEAS_HIGHREP = 0x2400
SHT31_MEAS_MEDREP = 0x240B
SHT31_MEAS_LOWREP = 0x2416
SHT31_READSTATUS = 0xF32D
SHT31_CLEARSTATUS = 0x3041
SHT31_SOFTRESET = 0x30A2
SHT31_HEATER_ON = 0x306D
SHT31_HEATER_OFF = 0x3066

SHT31_STATUS_DATA_CRC_ERROR = 0x0001
SHT31_STATUS_COMMAND_ERROR = 0x0002
SHT31_STATUS_RESET_DETECTED = 0x0010
SHT31_STATUS_TEMPERATURE_ALERT = 0x0400
SHT31_STATUS_HUMIDITY_ALERT = 0x0800
SHT31_STATUS_HEATER_ACTIVE = 0x2000
SHT31_STATUS_ALERT_PENDING = 0x8000

class SHT31D_Node:

  def __init__(self):
    rospy.init_node('sht31d', anonymous=True)
    self.seq = 0

    self.temp_pub = rospy.Publisher('/temp',
                                    StampedFloatWithVariance,
                                    queue_size=1)
    self.humidity_pub = rospy.Publisher('/humidity',
                                        StampedFloatWithVariance,
                                        queue_size=1)

    self.device_address = 0x44
    self.i2c = smbus.SMBus(1)
    self.reset()


  def write_command(self, command):
    try:
      self.i2c.write_i2c_block_data(self.device_address, command >> 8, [command & 0xFF])
      return True
    except IOError as e:
      print('Error ' + e.strerror + ' occurred while writing to address ' + str(self.device_address))
      return False

  def reset(self):
    self.write_command(SHT31_SOFTRESET)
    rospy.sleep(0.01)

  def set_heater(self, enable = True):
    if enable:
      self.write_command(SHT31_HEATER_ON)
    else:
      self.write_command(SHT31_HEATER_OFF)

  def crc8(self, buf):
    polynomial = 0x31
    crc = 0xFF

    for index in range(len(buf)):
      crc ^= buf[index]
      for i in range(8,0,-1):
        if crc & 0x80:
          crc = (crc << 1) ^ polynomial
        else:
          crc = (crc << 1)

    return crc & 0xFF

  def read_temperature_humidity(self):
    self.write_command(SHT31_MEAS_HIGHREP)
    rospy.sleep(0.015)
    
    try:
        buf = self.i2c.read_i2c_block_data(self.device_address, 0x00, 6)
    except IOError as e:
        print('Error ' + e.strerror + ' occurred while reading from address ' + str(self.device_address))
        return None
    
    if buf[2] != self.crc8(buf[0:2]):
      return (float('nan'), float('nan'))

    raw_temp = buf[0] << 8 | buf[1]
    temp = 175.0 * raw_temp / 0xFFFF - 45.0

    if buf[5] != self.crc8(buf[3:5]):
      return (temp, float('nan'))

    raw_humidity = buf[3] << 8 | buf[4]
    humidity = 100.0 * raw_humidity / 0xFFFF

    return temp, humidity

  def publish_data(self):

    temp, humidity = self.read_temperature_humidity()

    temp_msg = StampedFloatWithVariance()
    temp_msg.header.seq = self.seq
    temp_msg.header.stamp = rospy.Time.now()
    temp_msg.value = temp

    humidity_msg = StampedFloatWithVariance()
    humidity_msg.header = temp_msg.header
    humidity_msg.value = humidity

    self.temp_pub.publish(temp_msg)
    self.humidity_pub.publish(humidity_msg)

    if self.seq % 10 == 0:
      rospy.sleep(0.5)
      self.set_heater(True)
      rospy.sleep(1.0)
      self.set_heater(False)
      rospy.sleep(0.5)
    else:
      rospy.sleep(2.0)


if __name__ == '__main__':
  
  node = SHT31D_Node()

  while not rospy.is_shutdown():
    try:
      node.publish_data()
    except rospy.ROSInterruptException:
      pass
