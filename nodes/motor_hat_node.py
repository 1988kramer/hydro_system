#!/usr/bin/env python

'''
ROS node for controlling dc motors via the adafruit motor hat
'''

import rospy
from Adafruit_MotorHAT import Adafruit_MotorHat, Adafruit_DCMotor
from hydro_system import MotorHatCmd
import sys
import signal

class MotorHatNode:

  def __init__(self):
    rospy.init_node('motor_hat', anonymous=true, disable_signals=True)
    self.cmd_sub = rospy.Subscriber('/motor_hat_cmd', MotorHatCmd, cmd_callback, queue_size=1)
    self.motor_hat = Adafruit_MotorHAT(addr=0x60)
    self.num_motors = 4

  def shutdown(self, sig):
    for i in range(self.num_motors):
      motor_hat.getMotor(i+1).run(Adafruit_MotorHAT.RELEASE)
    rospy.loginfo('motor hat node shutting down')
    rospy.signal_shutdown('SIGINT received (CTRL+C)')
    sys.exit()

  def cmd_callback(self, msg):
    for i in range(self.num_motors):
      if msg.commands[i] == 'forward':
        motor_hat.getMotor(i+1).run(Adafruit_MotorHAT.FORWARD)
      elif msg.commands[i] == 'backward':
        motor_hat.getMotor(i+1).run(Adafruit_MotorHAT.BACKWARD)
      elif msg.commands[i] == 'release':
        motor_hat.getMotor(i+1).run(Adafruit_MotorHAT.RELEASE)
      motor_hat.getMotor(i+1).setSpeed(msg.speeds[i])

if __name__ == '__main__':
  motor_hat_node = MotorHatNode()
  signal.signal(signal.SIGINT, motor_hat_node.shutdown())
  rospy.spin()