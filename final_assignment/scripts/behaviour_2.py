#!/usr/bin/env python

"""
To make this node in order to operate the keyboard for manual control of the robot, I have taken
the code directly from the ros-teleop/teleop_twist_keyboard. The reason to do so was that this
was introduced during the lesson in RT1 for the purposes of teleoperation. The changes I have
made to this are as follows:
1. Anything with respect to windows os has been taken away and only those with respect to 
   termios has been left. Termios module provides an interface to the Unix terminal control 
   facilities. It can be used to control most ascpects of the terminal communication ports. 
2. Include a function inside the PublishingThread class that will stop the robot as well. I
   shall call it stop_robot_fnc() with a self arguement.
3. Removed the part of code that involves stamped from the def run(self)
4. Removed RestoreTerminalSettings()
   
"""

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

import sys
from select import select

import termios
"""
The tty module defines functions for putting the tty into cbreak and raw modes.
Because it requires termios module, it will work only on Unix.
The tty module defines the following functions:
tty.setraw(fd, when=termios.TCSAFLUSH)
    Change the mode of the file descriptor fd to raw. If when is omitted, it defaults to
    termios. TCSAFLUSH, and is passed to termios.tcsetattr().
tty.setcbreak(fd, when=termio.TCSAFLUSH)
    Change the mode of the file descriptor fd to cbreak. If when is omitted, it defaults to
    termios.TCSAFLUSH, and is passed to termios.tcsetattr().
"""
import tty


msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
t : up (+z)
b : down (-z)
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        'k':(-1,0,0,0),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()
        
    def stop_robot_fnc(self):
    # Copy paste of the end part of the run function as it publishes to stop message when
    # thread exits.
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        
        self.publisher.publish(twist)
        

    def run(self):
        twist = Twist()

        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)


def getKey(timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
       key = sys.stdin.read(1)
    else:
       key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":

    flag = 1

    rospy.init_node('behaviour_2')
    
    
    settings = saveTerminalSettings()
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.5)
    if key_timeout == 0.0:
        key_timeout = None
        
    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0
    
    # Here I will use rospy.Rate as it is a convenience class that makes a best effort at
    # maintaining a particular rate for a root.
    r = rospy.Rate(10) #10Hz
    # After defining the rate I will wait for subscribers and update the values of x, y, z, th,
    # speed and turn
    pub_thread.wait_for_subscribers()
    pub_thread.update(x, y, z, th, speed, turn)
    
    print(msg)
    print(vels(speed,turn))
    
    while not rospy.is_shutdown():
            
        active_ = rospy.get_param("/active")
            
        if active_ == 2:
            if flag == 0:
                print("Behaviour 2 ready to use\n")
                flag = 1
                
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = (speed * speedBindings[key][0])
                turn = (turn * speedBindings[key][1])
      
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            pub_thread.update(x, y, z, th, speed, turn)
        else:
            # For Idle State
            if flag == 1:
                pub_thread.stop_robot_fnc()
                flag = 0
        r.sleep()
                
            
