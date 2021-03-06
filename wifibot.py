#coding:utf-8
'''
'''
import os
from time import ctime
import RPi.GPIO as GPIO
import time
#from smbus import SMBus

import threading
import cv2
import numpy as np

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
    self.left_speed=80
    self.right_speed=80
    self.moving_dir=None
    self.servo=None
    self.init_robot()
    self.move_dir_io=(0,0,0,0)

  #
  #
  def init_robot(self):
    #self.servo = SMBus(1)
    #
    #  GPIO Mode
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

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
    GPIO.setup([LED0, LED1, LED2],GPIO.OUT,initial=GPIO.HIGH)

  #
  #
  def setup_motors(self):
    # right motor
    GPIO.setup(ENA,GPIO.OUT,initial=GPIO.LOW)
    self.ENA_pwm=GPIO.PWM(ENA,1000) 
    GPIO.setup([IN1,IN2],GPIO.OUT,initial=GPIO.LOW)

    # left motor
    GPIO.setup(ENB,GPIO.OUT,initial=GPIO.LOW)
    self.ENB_pwm=GPIO.PWM(ENB,1000) 
    GPIO.setup([IN3,IN4],GPIO.OUT,initial=GPIO.LOW)

  #
  #
  def setup_head(self):
    GPIO.setup(PAN,GPIO.OUT,initial=GPIO.LOW)
    self.PAN_pwm=GPIO.PWM(PAN,50) 
    self.PAN_pwm.start(0)
    GPIO.setup(TILT,GPIO.OUT,initial=GPIO.LOW)
    self.TILT_pwm=GPIO.PWM(TILT,50) 
    self.TILT_pwm.start(0)


  #
  #
  def setup_ir_sensors(self):
    GPIO.setup([IR_R, IR_L, IR_M, IRF_R, IRF_L] ,GPIO.IN,pull_up_down=GPIO.PUD_UP)

  #
  #
  def setup_sonar(self):
    GPIO.setup(TRIG,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(ECHO,GPIO.IN,pull_up_down=GPIO.PUD_UP)


  ####################################################
  ## Open_Light()
  ####################################################
  def  Open_Light(self):
    GPIO.output(LED0,False)
    time.sleep(1)

  ####################################################
  ##Close_Light()
  ####################################################
  def  Close_Light(self):
    GPIO.output(LED0,True)
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
        GPIO.output(LED0,x1)
      if x2 is not None:
        GPIO.output(LED1,x2)
      if x3 is not None:
        GPIO.output(LED2,x3)

  #####################################
  #
  #
  def motor_on(self):
    self.ENA_pwm.start(self.right_speed)
    self.ENB_pwm.start(self.left_speed)
    return

  #
  #
  def motor_off(self):
    self.ENA_pwm.stop()
    self.ENB_pwm.stop()
    return

  #
  #
  def stop(self):
    GPIO.output([IN1, IN2, IN3, IN4], (0, 0, 0, 0))
    self.moving_dir=None
    self.move_dir_io=(0,0,0,0)
    time.sleep(0.3)
    return

  #
  #
  def set_right_motor_dir(self, rev):
    if rev :
      GPIO.output([IN1,IN2], (0, 1))
    else:
      GPIO.output([IN1,IN2], (1, 0))
    return 

  #
  #
  def set_left_motor_dir(self, rev):
    if rev :
      GPIO.output([IN3,IN4], (0, 1))
    else:
      GPIO.output([IN3,IN4], (1, 0))
    return 

  #
  #
  def right_motor(self, sp, reverse):
    self.set_right_speed(sp)
    self.set_right_motor_dir(reverse)
    return 

  #
  #
  def left_motor(self, sp, reverse):
    self.set_left_speed(sp)
    self.set_left_motor_dir(reverse)
    return 

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

    if self.move_dir_io != move_io:
      GPIO.output([IN1, IN2, IN3,IN4], (0, 0, 0, 0))
      time.sleep(0.3)
      self.move_dir_io = move_io

    self.set_right_speed(np.abs(rsp))
    self.set_left_speed(np.abs(lsp))
    GPIO.output([IN1, IN2, IN3,IN4], move_io)

    return 
     
  #
  #
  def Forward(self, sp):
    print 'motor forward'
    if self.moving_dir and self.moving_dir != "forward":
      self.stop()

    self.set_speed(sp, sp)

    GPIO.output([IN1, IN3], 1)
    self.moving_dir = "forward"

    self.set_leds(None, 0, 0)
    return 
  
  #
  #
  def Backward(self, sp):
    print 'motor_backward'
    if self.moving_dir and self.moving_dir != "backward":
      self.stop()

    self.set_speed(sp, sp)

    GPIO.output([IN2, IN4], 1)

    self.moving_dir = "backward"
    self.set_leds(None, 1, 0)
    return 

  #
  #
  def TurnLeft(self, sp):
    print 'motor_turnleft'
    if self.moving_dir and self.moving_dir != "trunleft":
      self.stop()

    self.set_speed(sp, sp)

    GPIO.output([IN1,IN4], 1)

    self.moving_dir = "trunleft"
    self.set_leds(None, 0, 1)
    return 

  #
  #
  def TurnRight(self,sp):
    print 'motor_turnright'
    if self.moving_dir and self.moving_dir != "trunright":
      self.stop()

    self.set_speed(sp, sp)

    GPIO.output([IN2, IN3], 1)

    self.moving_dir = "trunright"
    self.set_leds(None, 0, 1)
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
  #
  def set_right_speed(self, num):
    if num <= 0: num=1
    elif num >= 100: num=99
    self.right_speed=num
    self.ENA_pwm.ChangeDutyCycle(num)

  #
  #
  def set_left_speed(self, num):
    if num <= 0: num=1
    elif num >= 100: num=99
    self.left_speed=num
    self.ENB_pwm.ChangeDutyCycle(num)

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

    #self.servo.XiaoRGEEK_SetServo(self.servo_no(ServoNum),self.Angle_cal(angle))
    return

  #
  #
  def servo_no(self, n):
    if n < 1: return 0x01
    elif n > 8: return 0x08
    return int(n)

  #
  #  5< ah,av < 23
  def move_head_gpio(self, ah, av):
    self.PAN_pwm.ChangeDutyCycle(ah)
    self.TILT_pwm.ChangeDutyCycle(av)
    return

  def move_head(self, ah, av):
    self.SetServoAngle(7,av)
    self.SetServoAngle(8,ah)
    return

  #
  #
  def close(self):
    self.ENA_pwm.stop()
    self.ENB_pwm.stop()
    GPIO.cleanup()


####################################################

if __name__ == '__main__':
  robot=WifiBot()

