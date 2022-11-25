#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

msg = """
Control robot
---------------------------
Moving around:
   q    w    e
   a    s    d
        x

w/x : increase/decrease linear X velocity 
q/e : increase/decrease angular velocity 
a/d : increase/decreace linear Y velocity
space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_y_vel, target_angular_vel):
    return "currently:\tlinear vel %s \t y vel: %s \t angular vel %s " % (target_linear_vel,target_y_vel, target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -0.5, 0.5)
    return vel

def checkYLimitVelocity(vel):
    vel = constrain(vel, -0.5, 0.5)
    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -1, 1)
    return vel

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('AR_TB4WD_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0
    target_y_vel = 0.0
    control_y_vel = 0.0
    try:
        print(msg)
        while(1):
            key = getKey()
            if key == 'w' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + 0.05)
                status = status + 1
                print(vels(target_linear_vel, target_y_vel,target_angular_vel))
            elif key == 'x' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - 0.05)
                status = status + 1
                print (vels(target_linear_vel, target_y_vel, target_angular_vel))
            elif key == 'q' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + 0.25)
                status = status + 1
                print (vels(target_linear_vel, target_y_vel,target_angular_vel))
            elif key == 'e' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - 0.25)
                status = status + 1
                print (vels(target_linear_vel,target_y_vel,target_angular_vel))
            elif key == 'a':
            	target_y_vel = checkYLimitVelocity(target_y_vel + 0.05)
            	status = status+1
            	print (vels(target_linear_vel,target_y_vel, target_angular_vel))
            elif key == 'd':
            	target_y_vel = checkYLimitVelocity(target_y_vel - 0.05)
            	status = status+1
            	print (vels(target_linear_vel,target_y_vel, target_angular_vel))
            elif key == ' ' or key == 's' :
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                target_y_vel = 0.0
                control_y_vel = 0.0
                print (vels(target_linear_vel,target_y_vel, target_angular_vel))
            else:
                if (key == '\x03'):
                    break
            if status == 20 :
                print(msg)
                status = 0

            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (0.04/0.25))
            twist.linear.y = 0.0; twist.linear.x = target_linear_vel; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (0.06/0.25))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = target_angular_vel

            control_y_vel = makeSimpleProfile(control_y_vel, target_y_vel, (0.04/0.25))
            twist.linear.y = target_y_vel
            pub.publish(twist)

    except Exception as eq:
        print(eq)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
