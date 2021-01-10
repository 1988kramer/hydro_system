#!/usr/bin/env python

''' 
ROS node for controlling pH in a hydroponics system
'''

import rospy
import numpy as np 
from hydro_system.msg import PhMsg, MotorHatCmd
from hydro_system.srv import ChangeSetPoint, ChangeSetPointResponse
from std_srv import Empty
import threading

class pH_ControllerNode():

  def __init__(self):
    rospy.init_node('pH_controller', anonymous=True)

    self.motor_cmd_pub = rospy.Publisher('/motor_hat_cmd', 
                                         MotorHatCmd, 
                                         queue=1)
    self.pH_sub = rospy.Subscriber('/pH', 
                                   PhMsg, 
                                   self.ph_callback)
    self.set_point_srv = rospy.Service('change_set_point', 
                                       ChangeSetPoint, 
                                       self.change_set_point)
    self.log_srv = rospy.Service('save_log',
                                 Empty,
                                 self.log_commands)

    self.set_point = 6.0
    self.range = 0.2
    self.pH = -1.0
    self.q = 0.5
    self.r = 0.5
    self.pH_var = 0.1
    self.last_t = 0.0
    self.log = []
    self.lock = threading.Lock()
    self.last_adjust_time = 0.0
    self.adjust_duration = 7200.0

    self.stop_msg = MotorHatCmd()
    self.stop_msg.commands = ['release'] * 4
    self.stop_msg.speeds = [0] * 4

    self.up_msg = MotorHatCmd()
    self.up_msg.speeds = [128, 0, 0, 0]
    self.up_msg.commands = ['forward', 'release', 'release', 'release']

    self.down_msg = MotorHatCmd()
    self.down_msg.speeds = [0, 128, 0, 0]
    self.down_msg.commands = ['release', 'forward', 'release', 'release']


  def ph_callback(self, msg):
    stamp = msg.header.stamp.to_sec()
    if self.pH == -1.0:
      self.pH = msg.pH
      self.last_t = stamp
    else:
      dt = stamp - self.last_t
      y_tilde = msg.pH - self.pH
      self.last_t = stamp
      pH_var_hat = pH_var + self.q * dt
      S = pH_var_hat + self.r
      K = pH_var_hat / S
      self.pH = self.pH + K * y_tilde
      self.pH_var = (1.0 - K) * pH_var

    # if not waiting for a previous adjustment to take effect
    if stamp - self.last_adjust_time > self.adjust_duration:

      # if pH estimate is significantly higher than the set point add pH down
      with lock:
        if self.pH - self.set_point > self.range:
          self.adjust(-1.0)
          self.log.append([stamp,-1.0])
        # if the pH estimate is significantly lower than the set point add pH up
        else:
          self.adjust(1.0)
          self.log.append([stamp,1.0])

        if len(self.log) > 0 and self.log[-1][0] - self.log[-2][0] > 604800.0:
          req = Empty()
          self.save_log(req)

      self.last_adjust_time = msg.header.stamp.to_sec()


  # add ~1ml of pH up or down
  def adjust(self, direction):

    if direction == 1.0:
      self.motor_cmd_pub(self.up_msg)
    else:
      self.motor_cmd_pub(self.down_msg)

    rospy.sleep(2.0)

    self.motor_cmd_pub(self.stop_msg)


  def change_set_point(self, req):
    if req.set_point > 14.0 or req.set_point < 0.0:
      raise rospy.ServiceException('invalid value for pH set point')
    elif req.set_point < 5.5 or req.set_point > 6.5:
      rospy.logerr('pH value of %.1f is not a good idea for hydroponics' % req.set_point)
    with self.lock:
      self.set_point = req.set_point
    return ChangeSetPointResponse('set point changed successfully')


  def save_log(self, req):
    log_mat = np.array(self.log)
    self.log = []
    date_str = datetime.today().strftime('%d_%m_%Y')
    np.save('/home/pi/logs/pH_command_' + date_str + '.npy', log_mat)
    return []


if __name__ == '__main__':

  pH_controller_node = pH_ControllerNode()
  rospy.spin()