#!/usr/bin/env python3

"""
This node is the user interface node where the user will be able to choose the behaviour
it wants the robot to follow. There are 3 behaviours of the robot, they are as follows:
1. Autonomously reach a x,y coordinate inserted by the user.
2. Let the user drive the robot with the keyboard.
3. Let the user drive the robot assisting them to avoid collisions.
In the user interface front, I will create a function that will take necessary inputs from
the user.
Let's begin!!!
"""

# I first have to import all the libraries that we will be necessary.
import rospy
from std_srvs.srv import * # The star indicates all


# I will initialize a flag to this node which when True will cancel the operation.   
flag = False


def MODE():

    global flag
    
    if flag == True:
        print("Press 0 to cancel the operation you have set")
        flag = False
    
    input_behaviour = input("Based on the instructions provided above, choose a behaviour!\n")
    
    if (input_behaviour == "1"):
        # When the user inserts 1, we need to make sure that the params set before the exec of
        # behaviour 1 reset's the robot's state
        rospy.set_param('active',0)
        print("Behaviour Activated: Behaviour 1 a.k.a autonomous\n")
        active_ = rospy.get_param("/active")
        """
        Now we can follow the same code that we used while performing the exercises in class
        with respect to inserting new position and reaching it. This make the robot move
        autonomously in the environment.
        """
        # Takes input from user to get the x and y coordinates of the goal
        print("Enter the desired coordinates!")
        x = float(input('x:'))
        y = float(input('y:'))
        rospy.set_param("des_pos_x", x)
        rospy.set_param("des_pos_y", y)
        rospy.set_param('active', 1)
        flag = True         
    
    elif (input_behaviour == "2"):
        rospy.set_param('active', 2)
        print("Behaviour Activated: Behaviour 2 a.k.a Driving Robot in map using Keyboard\n")    
        active_ = rospy.get_param("/active")
    
    elif (input_behaviour == "3"):
        rospy.set_param('active', 3)
        print("Behaviour Activated: Behaviour 3 a.k.a Assistive Driving when about to collide\n")    
        active_ = rospy.get_param("/active")
    
    elif (input_behaviour == "0"):
        
        rospy.set_param('active',0)
        print("You have set the robot to an idle state")
        active_ = rospy.get_param("/active")
        
    else:
        print("Wrong Input!")
    




user_information = """
    Here are the Instructions indicating the robot behaviours:
    1. Press 1 to make the robot move autonomously inside the map.
    2. Press 2 to take control of the robot's movements using the keyboard.
    3. Press 3 to avoid collisions. In this case, the robot will seek assistance when near an obstacle.
    4. Press 0 if you want to cancel the target and get the robot to an idle state.
    5. A default is given where an error message gets printed.
"""

def main():
    print(user_information)
    while not rospy.is_shutdown():
        MODE()

if __name__ == "__main__":
    main()
    
