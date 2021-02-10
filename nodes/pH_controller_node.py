#!/usr/bin/env python

''' 
ROS node for controlling pH in a hydroponics system
'''

import rospy
from hydro_system.msg import StampedFloatWithVariance, MotorHatCmd
from hydro_system.srv import ChangeSetPoint, ChangeSetPointResponse
from std_srvs.srv import Empty
import threading
from datetime import datetime

class pH_ControllerNode:

  def __init__(self):
    rospy.init_node('pH_controller', anonymous=True)

    self.motor_cmd_pub = rospy.Publisher('/motor_hat_cmd', 
                                         MotorHatCmd, 
                                         queue_size=1)
    self.pH_sub = rospy.Subscriber('/pH_filtered', 
                                   StampedFloatWithVariance, 
                                   self.ph_callback,
                                   queue_size=1)
    self.set_point_srv = rospy.Service('change_set_point', 
                                       ChangeSetPoint, 
                                       self.change_set_point)

    self.set_point = rospy.get_param('~set_point', 5.9)
    self.range = rospy.get_param('~range', 0.2)

    self.lock = threading.Lock()
    self.last_adjust_time = 0.0
    self.adjust_duration = 7200.0

    self.up_motor = 1
    self.down_motor = 0

    self.start_msg = MotorHatCmd()
    self.start_msg.motor = 0
    self.start_msg.speed = 128
    self.start_msg.command = 'forward'

    self.retract_msg = MotorHatCmd()
    self.retract_msg.motor = 0
    self.retract_msg.speed = 128
    self.retract_msg.command = 'backward'

    self.stop_msg = MotorHatCmd()
    self.stop_msg.motor = 0
    self.stop_msg.speed = 0
    self.stop_msg.command = 'release'


  def ph_callback(self, msg):

    stamp = rospy.Time.now()

    # if not waiting for a previous adjustment to take effect
    if stamp - self.last_adjust_time > self.adjust_duration:

      with self.lock:
        diff = msg.value - self.set_point
        # if pH estimate outside the range
        if abs(diff) > self.range:
          # if pH estimate is higher than the range add pH down
          if diff > 0.0:
            rospy.logerr('adjusting down')
            self.adjust(self.down_motor)
            self.log(stamp,-1.0)
          # if the pH estimate is significantly lower than the set point add pH up
          else:
            rospy.logerr('adjusting up')
            self.adjust(self.up_motor)
            self.log(stamp,1.0)

      self.last_adjust_time = msg.header.stamp.to_sec()
    


  # add ~1ml of pH up or down
  def adjust(self, motor):

    self.start_msg.motor = motor
    self.stop_msg.motor = motor
    self.rectract_msg = motor

    if direction == 1.0:
      rospy.logerr('motor up command')
    else:
      rospy.logerr('motor down command')

    self.motor_cmd_pub.publish(self.start_msg)
    rospy.sleep(8.0)
    self.motor_cmd_pub.publish(self.retract_msg)
    rospy.sleep(5.0)
    self.motor_cmd_pub.publish(self.stop_msg)


  def change_set_point(self, req):
    if req.set_point > 14.0 or req.set_point < 0.0:
      raise rospy.ServiceException('invalid value for pH set point')
    elif req.set_point < 5.5 or req.set_point > 6.5:
      rospy.logerr('pH value of %.1f is not a good idea for hydroponics' % req.set_point)
    with self.lock:
      self.set_point = req.set_point
    return ChangeSetPointResponse('set point changed successfully')


  def log(self, stamp, direction):
    date_str = datetime.today().strftime('%d_%m_%Y')
    filename = '/home/pi/logs/pH_command_' + date_str + '.csv'
    with open(filename, 'a') as file:
      file.write('%.4f,%d' % (stamp, direction))
    os.system('rclone copy ' + filename + ' remote_logs:personal\ projects/hydroponics/logs/')
    return []


if __name__ == '__main__':

  pH_controller_node = pH_ControllerNode()
  rospy.spin()
