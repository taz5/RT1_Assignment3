#!/usr/bin/env python

"""
This part of the assignment uses will use the teleop code when the robot is near an obstacle.
For detecting an obstacle I will be using a laser function which will give the necessary data in
order to let the user know that the robot is near the obstacle.
To start off I will take all the code from the ros-teleop/teleop_twist_keyboard.py file available 
on github. The code structure will be very much similar to that of behaviour_2.py. So let's begin!
"""
#!/usr/bin/env python

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

import sys
from select import select

from sensor_msgs.msg import LaserScan

import termios
import tty

left = True
right = True
front = True
msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
        i    
   j    k    l

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'k':(-1,0,0,0),
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

def Laser_clbk(msg):
    """
    There are 720 elements present in range vectors which we are going to divide into 5 equal
    parts, so each region will consist of 720/5 = 144 elements. But we only need to get data
    for the front, left and right sides.
    """
    global left
    global right
    global front
    
    r = min(msg.ranges[0:143])
    f = min(msg.ranges[288:431])
    l = min(msg.ranges[576:713])
    
    if r < 1.0:
        right = False
    else:
        right = True
    
    if f < 1.0:
        front = False
    else:
        front = True
    
    if l < 1.0:
        left = False
    else:
        left = True
        
"""
One extra function that needs to included here is a function that will remove the last data
given by the Laser so we don't need to worry about driving into the obstacle. For that we can use a pop command as it removes an item at the specified index from the list. If no index value
is mentioned, the pop() function will remove the last element from the list. The pop function
requires a dictionary in order to understand what to remove. So for those reasons, we can
create a copy of the MoveBindings. I will call my dictionary here as MB.
"""

def pop_fnc(MB):
   
   global left
   global right
   global front
   
   """
   Let us first try to understand what all cases are possible:
   1. When the obstacle is only in front of the robot
   2. When the obstacle is on the left of the robot
   3. When the obstacle is on the right of the robot
   4. When the obstacle is on the left and right of the robot
   5. When the obstacle is on the left and front of the robot
   6. When the obstacle is on the right and front of the robot
   7. When the obstacle is on the left, right and front of the robot
   Let's write a series of if conditions to solve this issue!
   """
   
   if not front and left and right:
       elem_removed1 = MB.pop('i')
       print("Command i has been removed!")
       
   elif front and not left and right:
       elem_removed1 = MB.pop('j')
       print("Command j has been removed!")
   
   elif front and left and not right:
       elem_removed1 = MB.pop('l')
       print("Command l has been removed!")
       
   elif front and not left and not right:
       elem_removed1 = MB.pop('j')
       elem_removed2 = MB.pop('l')
       print("Commands j and l have been removed!")
       
   elif not front and not left and right:
       elem_removed1 = MB.pop('i')
       elem_removed2 = MB.pop('j')
       print("Commands i and j have been removed!")
       
   elif not front and left and not right:
       elem_removed1 = MB.pop('i')
       elem_removed2 = MB.pop('l')
       print("Commands i and l have been removed!")
       
   elif not front and not left and not right:
       elem_removed1 = MB.pop('i')
       elem_removed2 = MB.pop('j')
       elem_removed3 = MB.pop('l')
       print("Command i, j and l have been removed!")



if __name__=="__main__":

    flag = 1

    rospy.init_node('behaviour_3')
    
    
    settings = saveTerminalSettings()
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.5)
    subs = rospy.Subscriber('/scan', LaserScan, Laser_clbk)
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
    
    # Creating the Dictionary
    copy_MoveBindings = {}
    print(msg)
    print(vels(speed,turn))
    
    while not rospy.is_shutdown():
            
        active_ = rospy.get_param("/active")
        copy_MoveBindings = moveBindings.copy()
            
        if active_ == 3:
            if flag == 0:
                print("Behaviour 3 ready to use\n")
                flag = 1
                
            key = getKey(key_timeout)
            pop_fnc(copy_MoveBindings)
            
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
