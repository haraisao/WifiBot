#coding:utf-8
'''
'''
import os
from time import ctime
import RPi.GPIO as GPIO
import time
from smbus import SMBus

import threading
#import cv2
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

########################
# Variables
#



########################
#  Class: WifiBot
#
class WifiBot(object):
  def __init__(self):
    self.left_speed=100
    self.right_speed=100
    self.moving_dir=None
    self.servo=None
    self.init_robot()

  #
  #
  def init_robot(self):
    self.servo = SMBus(1)

    #
    #  GPIO Mode
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    #########Setup Hardware##########
    self.setup_leds()
    self.setup_motors()
    self.setup_ir_sensors()
    self.setup_sonar()

    print '....WIFIROBOTS START!!!...'
    #self.stop_motor()

  #
  #
  def setup_leds(self):
    GPIO.setup(LED0,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(LED1,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(LED2,GPIO.OUT,initial=GPIO.HIGH)

  #
  #
  def setup_motors(self):
    # right motor
    GPIO.setup(ENA,GPIO.OUT,initial=GPIO.LOW)
    self.ENA_pwm=GPIO.PWM(ENA,1000) 
    self.ENA_pwm.start(0) 
    self.ENA_pwm.ChangeDutyCycle(100)
    GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)

    # left motor
    GPIO.setup(ENB,GPIO.OUT,initial=GPIO.LOW)
    self.ENB_pwm=GPIO.PWM(ENB,1000) 
    self.ENB_pwm.start(0) 
    self.ENB_pwm.ChangeDutyCycle(100)
    GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)

  #
  #
  def setup_ir_sensors(self):
    GPIO.setup(IR_R ,GPIO.IN,pull_up_down=GPIO.PUD_UP)
    GPIO.setup(IR_L ,GPIO.IN,pull_up_down=GPIO.PUD_UP)
    GPIO.setup(IR_M ,GPIO.IN,pull_up_down=GPIO.PUD_UP)
    GPIO.setup(IRF_R,GPIO.IN,pull_up_down=GPIO.PUD_UP)
    GPIO.setup(IRF_L,GPIO.IN,pull_up_down=GPIO.PUD_UP)

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
  def test_leds(self):
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

  def set_leds(self, x1, x2, x3):
      if x1 is not None:
        GPIO.output(LED0,(x1 == 1))
      if x2 is not None:
        GPIO.output(LED1,(x2 == 1))
      if x3 is not None:
        GPIO.output(LED2,(x3 == 1))

  #####################################
  def stop_motor (self):
    GPIO.output(ENA, 0)
    GPIO.output(ENB, 0)
    GPIO.output(IN1, 0)
    GPIO.output(IN2, 0)
    GPIO.output(IN3, 0)
    GPIO.output(IN4, 0)
    self.moving_dir=None
    time.sleep(0.3)
    return

  def set_right_motor_dir(self, rev):
    if rev :
      GPIO.output(IN1, 0)
      time.sleep(0.3)
      GPIO.output(IN2, 1)
    else:
      GPIO.output(IN2, 0)
      time.sleep(0.3)
      GPIO.output(IN1, 1)
    return 

  def set_left_motor_dir(self, rev):
    if rev :
      GPIO.output(IN3, 0)
      time.sleep(0.3)
      GPIO.output(IN4, 1)
    else:
      GPIO.output(IN4, 0)
      time.sleep(0.3)
      GPIO.output(IN3, 1)
    return 


  def right_motor(self, sp, reverse):
    self.set_right_speed(sp)
    self.set_right_motor_dir(reverse)
    return 

  def left_motor(self, sp, reverse):
    self.set_left_speed(sp)
    self.set_left_motor_dir(reverse)
    return 

  def set_speed(self, rsp, lsp):
    self.set_right_speed(rsp)
    self.set_left_speed(lsp)

  def Forward(self, sp):
    print 'motor forward'
    if self.moving_dir and self.moving_dir != "forward":
      self.stop_motor()

    self.set_speed(sp, sp)

    GPIO.output(IN1, 1)
    GPIO.output(IN3, 1)
    self.moving_dir = "forward"

    self.set_leds(None, 0, 0)
    return 
  
  def Backward(self, sp):
    print 'motor_backward'
    if self.moving_dir and self.moving_dir != "backward":
      self.stop_motor()

    self.set_speed(sp, sp)

    GPIO.output(IN2, 1)
    GPIO.output(IN4, 1)

    self.moving_dir = "backward"
    self.set_leds(None, 1, 0)
    return 

  def TurnLeft(self, sp):
    print 'motor_turnleft'
    if self.moving_dir and self.moving_dir != "trunleft":
      self.stop_motor()

    self.set_speed(sp, sp)

    GPIO.output(IN1, 1)
    GPIO.output(IN4, 1)

    self.moving_dir = "trunleft"
    self.set_leds(None, 0, 1)
    return 

  def TurnRight(self,sp):
    print 'motor_turnright'
    if self.moving_dir and self.moving_dir != "trunright":
      self.stop_motor()

    self.set_speed(sp, sp)

    GPIO.output(IN2, 1)
    GPIO.output(IN3, 1)

    self.moving_dir = "trunright"
    self.set_leds(None, 0, 1)
    return 

  def Stop(self):
    print 'motor_stop'
    self.stop_motor()
    self.set_leds(None, 1, 1)
    return 
  

  #####################################
  def set_right_speed(self, num):
    if num <= 0: num=1
    elif num > 100: num=100
    self.right_speed=num
    if num ==0:
      GPIO.output(IN1,0)
      GPIO.output(IN2,0)
    self.ENA_pwm.ChangeDutyCycle(num)

  #
  #
  def set_left_speed(self, num):
    if num <= 0: num=1
    elif num > 100: num=100
    self.left_speed=num
    if num==0:
      GPIO.output(IN3,0)
      GPIO.output(IN4,0)
    self.ENB_pwm.ChangeDutyCycle(num)

  ####################################################
  def Angle_cal(self, angle):
    if angle > 160:
      angle=160
    elif angle < 15:
      angle=15
    return angle
  
  #
  #
  def SetServoAngle(self, ServoNum,angle):
    self.set_leds(0, 1, 0)
    time.sleep(0.01)
    self.set_leds(1, 1, 1)

    self.servo.XiaoRGEEK_SetServo(self.servo_no(ServoNum),self.Angle_cal(angle))
    return

  #
  #
  def servo_no(self, n):
    if n < 1: return 0x01
    elif n > 8: return 0x08
    return int(n)

  def move_head(self, ah, av):
    self.SetServoAngle(7,av)
    self.SetServoAngle(8,ah)
    return

  ####################################################
  def  Avoiding(self): 
    if GPIO.input(IR_M) == False:
      self.Stop()
      time.sleep(0.1)
      return

  ####################################################
  def  Get_Distence(self):
    time.sleep(0.1)
    GPIO.output(TRIG,GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(TRIG,GPIO.LOW)
    while not GPIO.input(ECHO):
        pass
    t1 = time.time()
    while GPIO.input(ECHO):
        pass
    t2 = time.time()
    time.sleep(0.1)
    return (t2-t1)*340/2*100

  ####################################################
  def  AvoidByRadar(self, distance):
    dis = int(self.Get_Distence())
    if(distance<20):
      distance = 20
    if((dis>1)&(dis < distance)):
      self.Stop()
  
    
  def Avoid_wave(self):
    dis = self.Get_Distence()
    if dis<20:
      self.Stop()
    else:
      self.Forward()


####################################################

if __name__ == '__main__':
  robot=WifiBot()

