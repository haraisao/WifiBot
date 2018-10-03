#! /usr/bin/env python
# -*- coding: utf-8 -*-
import pygame
from pygame.locals import *

from wifibot2 import *

def setup_joystick():
  pygame.joystick.init()
  try:
    j = pygame.joystick.Joystick(0) # create a joystick instance
    j.init() # init instance
    print 'Joystick name: ' + j.get_name()
    print 'Num of buttons : ' + str(j.get_numbuttons())
    print 'Num of sticks  : ' + str(j.get_numaxes())
    return j

  except pygame.error:
    print 'Joystickã not found.'
    return None
    

#
# main function
def main(joy):
  loop_flag=True
  robot=WifiBot()

  pygame.init()

  while loop_flag:
    pygame.time.wait(50)

    for e in pygame.event.get():
      if e.type == pygame.locals.JOYAXISMOTION: # Analogue Stick
        v=joy.get_axis(1) * 200
        if abs(v) < 50 : rv=lv=0
        elif abs(v) < 150 : rv=lv= -75*np.sign(v)
        else : rv=lv= -150*np.sign(v)

        tsp=joy.get_axis(3) 
        if tsp < 0 :
          if rv > 0:
            rv += tsp*abs(rv)
          elif rv <0:
            rv -= tsp*abs(rv)
        if tsp > 0 :
          if lv > 0:
            lv -= tsp*abs(lv)
          elif lv <0:
            lv += tsp*abs(lv)

        #print (v,tsp, lv, rv)
        robot.set_speed( int(lv), int(rv) )
        pass


      elif e.type == pygame.locals.JOYHATMOTION: # cross key 
        hatval=joy.get_hat(0)
        if hatval == (0,0):
          pygame.time.set_timer(e.type, 0)
        else:
          pygame.time.set_timer(e.type, 60)
        robot.move_head2( hatval[0]*30, -hatval[1]*30 )
        

      elif e.type == pygame.locals.JOYBUTTONDOWN: # push a button
        print str(e.button)+' down'
        if e.button == 0: loop_flag=False

      elif e.type == pygame.locals.JOYBUTTONUP: # release a button
        print str(e.button)+' up'
        if e.button == 10:
          robot.move_head(1500,1500)

      else:
        print e

    #
    #
#    hatval=joy.get_hat(0)
#    if hatval != (0,0):
#      print "MOVE Camera"
#      robot.move_head2( hatval[0]*10, hatval[1]*10 )

if __name__ == '__main__':
  j=setup_joystick()
  if j :
    main(j)
