#
#
# Copyright (C) 2018 Isao Hara, All Right Reserved.
# Released under the MIT license
#
from __future__ import print_function
import sys
import os

#
#
def daemonize(pidfname="daemon.pid"):
  try:
    pid=os.fork()
  except:
    print( "ERROR in fork1" )

  if pid > 0:
    os._exit(0)

  try:
    os.setsid()
  except:
    print( "ERROR in setsid" )

  try:
    pid=os.fork()
  except:
    print( "ERROR in fork2" )

  if pid > 0:
    try:
      with open(pidfname, "w") as f:
        print( pid, file=f )
    except:
      pass
    os._exit(0)
