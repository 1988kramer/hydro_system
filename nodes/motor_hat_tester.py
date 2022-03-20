#!/usr/bin/env python

'''
ROS node for testing the motor hat node
'''

import rospy
from hydro_system.msg import MotorHatCmd


def command_publisher():
  global motor
  global command_pub
  '''
  if motor == 0:
    motor = 1
  else:
    motor = 0
  '''
  msg = MotorHatCmd()
  msg.command = 'forward'
  msg.speed = 128
  msg.motor = motor

  command_pub.publish(msg)

  rospy.sleep(8.0)

  retract_msg = MotorHatCmd()
  retract_msg.command = 'backward'
  retract_msg.speed = 128
  retract_msg.motor = motor

  command_pub.publish(retract_msg)

  rospy.sleep(5.0)

  stop_msg = MotorHatCmd()
  stop_msg.command = 'release'
  stop_msg.speed = 0
  stop_msg.motor = motor

  command_pub.publish(stop_msg)
  rospy.sleep(0.1)




if __name__ == '__main__':

  rospy.init_node('motor_tester', anonymous=True)
  command_pub = rospy.Publisher('/motor_hat_cmd', MotorHatCmd, queue_size=1)

  motor = 1

  while not rospy.is_shutdown():
    try:
      command_publisher()
    except rospy.ROSInterruptException:
      pass
