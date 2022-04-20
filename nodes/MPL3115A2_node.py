#!/usr/bin/env python

'''
ROS node for taking readings from the MPL3115A2 barometric pressure sensor
and converting them to water depth.

Uses code from the example found at https://gist.github.com/pepijndevos/c3646ef6652e0f0342dd
'''

import rospy
import smbus
import requests
import json
import yaml
from hydro_system.msg import StampedFloatWithVariance


#REGISTERS

MPL3115A2_REGISTER_STATUS = (0x00)
MPL3115A2_REGISTER_STATUS_TDR = 0x02
MPL3115A2_REGISTER_STATUS_PDR = 0x04
MPL3115A2_REGISTER_STATUS_PTDR = 0x08

MPL3115A2_REGISTER_PRESSURE_MSB = (0x01)
MPL3115A2_REGISTER_PRESSURE_CSB = (0x02)
MPL3115A2_REGISTER_PRESSURE_LSB = (0x03)

MPL3115A2_REGISTER_TEMP_MSB = (0x04)
MPL3115A2_REGISTER_TEMP_LSB = (0x05)

MPL3115A2_REGISTER_DR_STATUS = (0x06)

MPL3115A2_OUT_P_DELTA_MSB = (0x07)
MPL3115A2_OUT_P_DELTA_CSB = (0x08)
MPL3115A2_OUT_P_DELTA_LSB = (0x09)

MPL3115A2_OUT_T_DELTA_MSB = (0x0A)
MPL3115A2_OUT_T_DELTA_LSB = (0x0B)

MPL3115A2_BAR_IN_MSB = (0x14)

MPL3115A2_WHOAMI = (0x0C)

#BITS

MPL3115A2_PT_DATA_CFG = 0x13
MPL3115A2_PT_DATA_CFG_TDEFE = 0x01
MPL3115A2_PT_DATA_CFG_PDEFE = 0x02
MPL3115A2_PT_DATA_CFG_DREM = 0x04

MPL3115A2_CTRL_REG1 = (0x26)
MPL3115A2_CTRL_REG1_SBYB = 0x01
MPL3115A2_CTRL_REG1_OST = 0x02
MPL3115A2_CTRL_REG1_RST = 0x04
MPL3115A2_CTRL_REG1_OS1 = 0x00
MPL3115A2_CTRL_REG1_OS2 = 0x08
MPL3115A2_CTRL_REG1_OS4 = 0x10
MPL3115A2_CTRL_REG1_OS8 = 0x18
MPL3115A2_CTRL_REG1_OS16 = 0x20
MPL3115A2_CTRL_REG1_OS32 = 0x28
MPL3115A2_CTRL_REG1_OS64 = 0x30
MPL3115A2_CTRL_REG1_OS128 = 0x38
MPL3115A2_CTRL_REG1_RAW = 0x40
MPL3115A2_CTRL_REG1_ALT = 0x80
MPL3115A2_CTRL_REG1_BAR = 0x00
MPL3115A2_CTRL_REG2 = (0x27)
MPL3115A2_CTRL_REG3 = (0x28)
MPL3115A2_CTRL_REG4 = (0x29)
MPL3115A2_CTRL_REG5 = (0x2A)

MPL3115A2_REGISTER_STARTCONVERSION = (0x12)


class MPL3115A2_Node:

  def __init__(self):
    rospy.init_node('mpl3115a2', anonymous=True)
    self.seq = 0
    self.depth_pub = rospy.Publisher('/depth',
                                     StampedFloatWithVariance,
                                     queue_size=1)

    self.barometric_pressure_kPa = 101.0
    self.altitude_offset_kPa = -0.8052
    self.in_h2o_per_kPa = 4.01865
    self.device_address = 0x60
    self.multiplexer_address = 0x70
    self.depth_sensor_idx = 0b00000001
    self.barometric_sensor_idx = 0b00000010
    self.i2c = smbus.SMBus(1)

    self.set_multiplexer_index(self.depth_sensor_idx)
    self.setup_sensor()
    self.set_multiplexer_index(self.barometric_sensor_idx)
    self.setup_sensor()
    

  def setup_sensor(self):
    whoami = self.i2c.read_byte_data(self.device_address, MPL3115A2_WHOAMI)
    if whoami != 0xc4:
      print('Unable to connect to MPL3115A2 device')

    self.i2c.write_byte_data(self.device_address,
                             MPL3115A2_CTRL_REG1,
                             MPL3115A2_CTRL_REG1_SBYB |
                             MPL3115A2_CTRL_REG1_OS128 |
                             MPL3115A2_CTRL_REG1_ALT)

    self.i2c.write_byte_data(self.device_address,
                             MPL3115A2_PT_DATA_CFG,
                             MPL3115A2_PT_DATA_CFG_TDEFE |
                             MPL3115A2_PT_DATA_CFG_PDEFE |
                             MPL3115A2_PT_DATA_CFG_DREM)


  def set_multiplexer_index(self, index):
    self.i2c.write_byte(self.multiplexer_address, index)
    rospy.sleep(0.01)


  def poll(self):
    sta = 0
    while not (sta & MPL3115A2_REGISTER_STATUS_PDR):
      sta = self.i2c.read_byte_data(self.device_address, 
                                    MPL3115A2_REGISTER_STATUS)

  def read_water_pressure(self):
    self.set_multiplexer_index(self.depth_sensor_idx)
    return self.read_pressure()


  def read_barometric_pressure(self):
    self.set_multiplexer_index(self.barometric_sensor_idx)
    return self.read_pressure()


  def read_pressure(self):
    self.i2c.write_byte_data(self.device_address,
                             MPL3115A2_CTRL_REG1,
                             MPL3115A2_CTRL_REG1_SBYB | 
                             MPL3115A2_CTRL_REG1_OS128 |
                             MPL3115A2_CTRL_REG1_BAR)
    self.poll()

    msb, csb, lsb = self.i2c.read_i2c_block_data(self.device_address, 
                                                 MPL3115A2_REGISTER_PRESSURE_MSB, 
                                                 3)
    return float((msb << 16) | (csb << 8) | lsb) / 64.0


  def publish_data(self):
    sensor_pressure_Pa = self.read_water_pressure()
    sensor_pressure_kPa = sensor_pressure_Pa / 1.0e3

    barometric_pressure_Pa = self.read_barometric_pressure()
    barometric_pressure_kPa = barometric_pressure_Pa / 1.0e3

    pressure_diff_kPa = sensor_pressure_kPa - barometric_pressure_kPa
    pressure_diff_in_h2o = pressure_diff_kPa * self.in_h2o_per_kPa
    
    pressure_msg = StampedFloatWithVariance()
    pressure_msg.header.seq = self.seq
    pressure_msg.header.stamp = rospy.Time.now()
    pressure_msg.value = pressure_diff_in_h2o

    self.depth_pub.publish(pressure_msg)

    self.seq += 1

    rospy.sleep(2.0)
     

if __name__ == '__main__':
  
  node = MPL3115A2_Node()

  while not rospy.is_shutdown():
    try:
      node.publish_data()
    except rospy.ROSInterruptException:
      pass

