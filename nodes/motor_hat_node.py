#!/usr/bin/env python

'''
ROS node for controlling dc motors via the adafruit motor hat
'''

import rospy
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
from hydro_system.msg import MotorHatCmd
import sys
import signal

class MotorHatNode:

  def __init__(self):
    rospy.init_node('motor_hat', anonymous=True, disable_signals=True)
    self.cmd_sub = rospy.Subscriber('/motor_hat_cmd', MotorHatCmd, self.cmd_callback, queue_size=1)
    self.motor_hat = Adafruit_MotorHAT(addr=0x61)
    self.num_motors = 4

  def shutdown(self, sig, frame):
    for i in range(self.num_motors):
      self.motor_hat.getMotor(i+1).run(Adafruit_MotorHAT.RELEASE)
      self.motor_hat.getMotor(i+1).setSpeed(0)
    rospy.loginfo('motor hat node shutting down')
    rospy.signal_shutdown('SIGINT received (CTRL+C)')
    sys.exit()

  def cmd_callback(self, msg):
    motor_num = msg.motor + 1
    if msg.command == 'forward':
      self.motor_hat.getMotor(motor_num).run(Adafruit_MotorHAT.FORWARD)
    elif msg.command == 'backward':
      self.motor_hat.getMotor(motor_num).run(Adafruit_MotorHAT.BACKWARD)
    elif msg.command == 'release':
      self.motor_hat.getMotor(motor_num).run(Adafruit_MotorHAT.RELEASE)
    self.motor_hat.getMotor(motor_num).setSpeed(msg.speed)

if __name__ == '__main__':
  motor_hat_node = MotorHatNode()
  signal.signal(signal.SIGINT, motor_hat_node.shutdown)
  rospy.spin()
