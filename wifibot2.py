#coding:utf-8
'''
'''
import os
from time import ctime
import pigpio
import time

import threading
import cv2
import numpy as np
from scipy import interpolate

########################
# Constants
#
### fro LEDs ###
LED0 = 10
LED1 = 9
LED2 = 25

### for Motors ###
ENA = 13  #//L298 A
ENB = 20  #//L298 B
IN1 = 19  #//1
IN2 = 16  #//2
IN3 = 21  #//3
IN4 = 26  #//4

### for Sonar ###
ECHO = 4  
TRIG = 17  

### fro IR Sensors ###
IR_R = 18  
IR_L = 27
IR_M = 22  
IRF_R = 23  
IRF_L = 24  


### Pan,Tilt
PAN=6
TILT=5

########################
#  Class: WifiBot
#
class WifiBot(object):
  #
  #
  def __init__(self):
    # IN1:19, IN2:16, IN3:21, IN4:26
    self.motor_io=[IN1, IN2, IN3, IN4]

    # PAN:16, TILT:5
    self.head_io=[PAN, TILT]

    # IR_R:18, IR_L:27, IR_M:22, IRF_R:23, IRF_L:24
    self.ir_io=[IR_R, IR_L, IR_M, IRF_R, IRF_L]

    # ECHO:4, TRIG:17
    self.sonar_io=[ECHO, TRIG]

    self.left_speed=200
    self.right_speed=200
    self.moving_dir=None
    self.vel2sp=None
    self.servo=None
    self.limit=int(255*1.5)
    self.init_robot()
    self.move_dir_io=(0,0,0,0)
    self.head=[1500,1500]

  #
  #
  def init_robot(self):
    #
    #  GPIO Mode
    self.pi=pigpio.pi()

    #########Setup Hardware##########
    self.setup_leds()
    self.setup_motors()
    self.setup_head()
    self.setup_ir_sensors()
    self.setup_sonar()

    print '....WIFIROBOTS START!!!...'
    self.motor_on()
    return

  #
  #
  def setup_leds(self):
    for x in [LED0, LED1, LED2]:
      self.pi.set_mode(x, pigpio.OUTPUT)
      self.pi.write(x, 1)
    return

  #
  #
  def setup_motors(self):
    for x in [IN1, IN2, IN3, IN4]:
      self.pi.set_mode(x, pigpio.OUTPUT)
      self.pi.write(x, 0)

    self.pi.set_PWM_frequency(ENA, 1000)
    self.pi.set_PWM_frequency(ENB, 1000)
    return

  #
  #
  def setup_head(self):
    self.pi.set_servo_pulsewidth(PAN, 1500)
    self.pi.set_servo_pulsewidth(TILT, 1500)
    return

  #
  #
  def setup_ir_sensors(self):
    for x in self.ir_io:
      self.pi.set_mode(x, pigpio.INPUT)
      self.pi.set_pull_up_down(x, pigpio.PUD_UP)
    return

  #
  #
  def setup_sonar(self):
    self.pi.set_mode(TRIG, pigpio.OUTPUT)
    self.pi.write(TRIG, 0)

    self.pi.set_mode(ECHO, pigpio.INPUT)
    self.pi.set_pull_up_down(ECHO, pigpio.PUD_UP)
    return

  def make_lookup_table_v(self, y, mval=10):
    t=np.linspace(0, mval, 11)
    self.vel2sp=interpolate.interp1d(t, y, kind="cubic")
    return

  def move(self, v, w, d=0.1):
    wd=w*d
    right_v=v+wd
    left_v=v-wd
    rsp=self.vel2sp(right_v)
    lsp=self.vel2sp(left_v)
    self.set_speed(right_v, left_v)
    return

  def move_head2(self, dp, dt):
    if dp :
      self.head[0]=min(max(self.head[0]+dp, 500), 2500)
      self.pi.set_servo_pulsewidth(PAN, self.head[0])
    if dt :
      self.head[1]=min(max(self.head[1]+dt, 500), 2500)
      self.pi.set_servo_pulsewidth(TILT, self.head[1])
    return

  ####################################################
  ## Open_Light()
  ####################################################
  def  Open_Light(self):
    self.pi.write(LED0, 0)
    time.sleep(1)

  ####################################################
  ##Close_Light()
  ####################################################
  def  Close_Light(self):
    self.pi.write(LED0, 1)
    time.sleep(1)
  
  ####################################################
  #
  #
  def demo_leds(self):
    for i in range(1, 5):
      self.set_leds(0,0,0)
      time.sleep(0.5)
      self.set_leds(1,0,0)
      time.sleep(0.5)
      self.set_leds(0,1,0)
      time.sleep(0.5)
      self.set_leds(0,0,1)
      time.sleep(0.5)
      self.set_leds(0,0,0)
      time.sleep(0.5)
      self.set_leds(1,1,1)

  #
  #
  def set_leds(self, x1, x2, x3):
    if x1 is not None:
      self.pi.write(LED0,x1)
    if x2 is not None:
      self.pi.write(LED1,x2)
    if x3 is not None:
      self.pi.write(LED2,x3)

  #####################################
  #
  #
  def motor_on(self):
    self.pi.set_PWM_dutycycle(ENA, self.right_speed)
    self.pi.set_PWM_dutycycle(ENB, self.left_speed)
    return

  #
  #
  def motor_off(self):
    self.pi.set_PWM_dutycycle(ENA, 0)
    self.pi.set_PWM_dutycycle(ENB, 0)
    return

  #
  #
  def set_motor_io(self, io, skip=0):
    for x in range(len(io)):
      self.pi.write(self.motor_io[x+skip], io[x])

    return

  #
  #
  def stop(self):
    self.move_dir_io=(0,0,0,0)
    self.set_motor_io(self.move_dir_io)
    self.moving_dir=None
    time.sleep(0.3)
    return


  #
  #
  def get_io(self, sp):
    if sp>0:
      return (1,0)
    elif sp<0:
      return (0,1)
    else:
      return (0,0)
  #
  #
  def set_speed(self, rsp, lsp):
    r_io=self.get_io(rsp)
    l_io=self.get_io(lsp)
    move_io=r_io+l_io

    drsp = np.abs(self.right_speed - rsp)
    dlsp = np.abs(self.left_speed - lsp)

    if (drsp > self.limit or dlsp > self.limit) and self.move_dir_io != move_io:
      self.set_motor_io((0,0,0,0))
      time.sleep(0.5)
      self.move_dir_io = move_io

    self.set_right_speed(np.abs(rsp))
    self.set_left_speed(np.abs(lsp))
    self.set_motor_io(move_io)

    return 
     
  def get_speed_from_v(self, v):
    return int(self.vel2sp(v * 10))

  #
  #
  def Forward(self, sp, timeout=-1):
    print 'motor forward'
    if self.moving_dir and self.moving_dir != "forward":
      self.stop()

    
    self.set_speed(sp, sp)
    self.moving_dir = "forward"
    self.set_leds(None, 0, 0)
    if timeout > 0:
      time.sleep(timeout)
      self.Stop()

    return 
  
  #
  #
  def Backward(self, sp, timeout=-1):
    print 'motor_backward'
    if self.moving_dir and self.moving_dir != "backward":
      self.stop()

    self.set_speed(-sp, -sp)

    self.moving_dir = "backward"
    self.set_leds(None, 1, 0)

    if timeout > 0:
      time.sleep(timeout)
      self.Stop()

    return 

  #
  #
  def TurnLeft(self, sp, timeout=-1):
    print 'motor_turnleft'
    if self.moving_dir and self.moving_dir != "trunleft":
      self.stop()

    self.set_speed(sp, -sp)

    self.moving_dir = "trunleft"
    self.set_leds(None, 0, 1)
    if timeout > 0:
      time.sleep(timeout)
      self.Stop()

    return 

  #
  #
  def TurnRight(self,sp, timeout=-1):
    print 'motor_turnright'
    if self.moving_dir and self.moving_dir != "trunright":
      self.stop()

    self.set_speed(-sp, sp)

    self.moving_dir = "trunright"
    self.set_leds(None, 0, 1)
    if timeout > 0:
      time.sleep(timeout)
      self.Stop()

    return 

  #
  #
  def Stop(self):
    print 'motor_stop'
    self.stop()
    self.set_leds(None, 1, 1)
    return 
  

  #####################################
  #
  #  0< sp < 255, 0:stop, 255:full
  #
  def set_right_speed(self, num):
    if num <= 0: num=0
    elif num >= 255: num=255
    self.right_speed=num
    self.pi.set_PWM_dutycycle(ENA,num)

  #
  #
  def set_left_speed(self, num):
    if num <= 0: num=0
    elif num >= 255: num=255
    self.left_speed=num
    self.pi.set_PWM_dutycycle(ENB,num)

  ####################################################
  #
  #
  def Angle_cal(self, angle):
    if angle > 160:
      angle=160
    elif angle < 20:
      angle=20
    return angle
  
  #
  #
  def SetServoAngle(self, ServoNum,angle):
    self.set_leds(0, 1, 0)
    time.sleep(0.01)
    self.set_leds(1, 1, 1)
    try:
      self.servo.XiaoRGEEK_SetServo(self.servo_no(ServoNum),self.Angle_cal(angle))
    except:
      print("Error in servo")
    return

  #
  #
  def servo_no(self, n):
    if n < 1: return 0x01
    elif n > 8: return 0x08
    return int(n)

  #
  #  500< ah,av < 2500, 500: right(90), 1500: center , 2500: left(-90)
  def move_head(self, ah, av):
    self.pi.set_servo_pulsewidth(PAN,ah)
    self.pi.set_servo_pulsewidth(TILT,av)
    self.head[0]=ah
    self.head[1]=av
    return

  def move_head_pwm(self, ah, av):
    self.SetServoAngle(7,av)
    self.SetServoAngle(8,ah)
    return

  #
  #
  def close(self):
    self.pi.stop()

  ########################
  #
  def Avoiding(self):
    if self.pi.read(IR_M) == 0:
      self.stop()
      time.sleep(0.1)
    return

  def get_ir_sensor(self):
    res=[]
    for x in self.ir_io:
      res.append(self.read(x))
    return res

  def get_distance(self):
    time.sleep(0.1)
    self.pi.write(TRIG, 1)
    time.sleep(0.000015)
    self.pi.write(TRIG, 0)
    while not self.pi.read(ECHO) :
      pass
    t1=time.time()
    while self.pi.read(ECHO):
      pass
    t2=time.time()
    time.sleep(0.1)
    return (t2 - t1) * 17000 # 340/2*100

    
####################################################

if __name__ == '__main__':
  robot=WifiBot()

