#!/usr/bin/env python

''' 
ROS node for controlling pH in a hydroponics system
'''

import rospy
import numpy as np 
from hydro_system.msg import PhMsg, MotorHatCmd
from hydro_system.srv import ChangeSetPoint, ChangeSetPointResponse
from std_srvs.srv import Empty
import threading
from datetime import datetime

class pH_ControllerNode():

  def __init__(self):
    rospy.init_node('pH_controller', anonymous=True)

    self.motor_cmd_pub = rospy.Publisher('/motor_hat_cmd', 
                                         MotorHatCmd, 
                                         queue_size=1)
    self.pH_sub = rospy.Subscriber('/pH', 
                                   PhMsg, 
                                   self.ph_callback,
                                   queue_size=1)
    self.set_point_srv = rospy.Service('change_set_point', 
                                       ChangeSetPoint, 
                                       self.change_set_point)
    self.log_srv = rospy.Service('save_controller_log',
                                 Empty,
                                 self.save_log)

    self.set_point = 6.0
    self.range = 0.2
    self.pH = -1.0
    self.q = 0.03
    self.r = 0.15
    self.pH_var = 0.1
    self.last_t = 0.0
    self.log = []
    self.lock = threading.Lock()
    self.last_adjust_time = 0.0
    self.adjust_duration = 60.0 # 7200.0

    self.stop_msg = MotorHatCmd()
    self.stop_msg.commands = ['release'] * 4
    self.stop_msg.speeds = [0] * 4

    self.down_msg = MotorHatCmd()
    self.down_msg.speeds = [128, 0, 0, 0]
    self.down_msg.commands = ['forward', 'release', 'release', 'release']

    self.up_msg = MotorHatCmd()
    self.up_msg.speeds = [0, 128, 0, 0]
    self.up_msg.commands = ['release', 'forward', 'release', 'release']


  def ph_callback(self, msg):
    stamp = msg.header.stamp.to_sec()
    if self.pH == -1.0:
      self.pH = msg.pH
      self.last_t = stamp
    else:
      dt = stamp - self.last_t
      y_tilde = msg.pH - self.pH
      self.last_t = stamp
      pH_var_hat = self.pH_var + self.q * dt
      S = pH_var_hat + self.r
      K = pH_var_hat / S
      self.pH = self.pH + K * y_tilde
      self.pH_var = (1.0 - K) * self.pH_var

    # if not waiting for a previous adjustment to take effect
    if stamp - self.last_adjust_time > self.adjust_duration:

      with self.lock:
        diff = self.pH - self.set_point
        # if pH estimate outside the range
        if abs(diff) > self.range:
          # if pH estimate is higher than the range add pH down
          if diff > 0.0:
            rospy.logerr('adjusting down')
            self.adjust(-1.0)
            self.log.append([stamp,-1.0])
          # if the pH estimate is significantly lower than the set point add pH up
          else:
            rospy.logerr('adjusting up')
            self.adjust(1.0)
            self.log.append([stamp,1.0])

        if len(self.log) > 0 and self.log[-1][0] - self.log[0][0] > 86400.0:
          req = Empty()
          self.save_log(req)

      self.last_adjust_time = msg.header.stamp.to_sec()
    


  # add ~1ml of pH up or down
  def adjust(self, direction):

    if direction == 1.0:
      rospy.logerr('motor up command')
      self.motor_cmd_pub.publish(self.up_msg)
    else:
      rospy.logerr('motor down command')
      self.motor_cmd_pub.publish(self.down_msg)

    rospy.sleep(2.0)

    self.motor_cmd_pub.publish(self.stop_msg)


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
    date_str = datetime.today().strftime('%d_%m_%Y_%H_%M')
    filename = '/home/pi/logs/pH_command_' date_str + '.npy'
    np.save(filename, log_mat)
    os.system('rclone copy ' + filename + ' remote_logs:personal\ projects/hydroponics/logs/')
    return []


if __name__ == '__main__':

  pH_controller_node = pH_ControllerNode()
  rospy.spin()
