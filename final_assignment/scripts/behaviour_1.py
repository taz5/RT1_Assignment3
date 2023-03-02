#!/usr/bin/env python3
"""
This particular node is responsible for making the robot drive autonomously.
I will be using actionlib in order to perform this part as there are advantages to using
action clients over services. It is similar to services in the sense that it sends a request
to an action server in order to achieve some goal and will get a result. But unlike services,
while the action is being performed an action server sends progress feedback to the client. 
Actions are useful when a response may take a significant length of time.
"""

# I first have to import all the libraries that we will be necessary.

import rospy
from std_srvs.srv import *
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf import transformations

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point

"""
Before making the necessary function for this node, I will initialize variable to be used in
this node.
"""
msg = """
You have activated the autonomous mode of the robot! This node will run as soon as you put
the x and y coordinates in the user interface.
"""
"""
For making the robot reach the goal, I will be using an action client MoveBaseGoal() as the
message and will consist of all the behaviours of the robot whilst reaching its desired goal.
"""
goal = MoveBaseGoal()
# Initializing the desired x and y coordinates
desired_position_x = 0
desired_position_y = 0

# Initializing a Flag to see if the robot reached the goal.
flag_goal = False

active_ = 0

# Necessary Function definitions

def clbk_odom(msg):
    # The callback function will get the present position of the robot in the environment.
    global position_  
    
    #position
    position_ = msg.pose.pose.position

def goal_status(term_state,result):
    
    global flag_goal
    global client
    
    """
    I have taken this part from the ros documentation of the actionlib_msgs/GoalStatus Message
    Here's what each goal ID indicates:
    0. The goal has yet to be processed by the action server.
    1. The goal is currently being processed by the action server.
    2. The goal received a cancel request after it started executing and has since completed its
       execution.
    3. The goal was achieved successfully by the action server.
    4. The goal was aborted during execution by the action server due to some failure.
    5. The goal was rejected by the action server without being processed, because the goal was
       unattainable or invalid.
    6. The goal received a cancel request after it started executing and has not yet completed
       execution.
    7. The goal received a cancel request before it started executing, but the action server has
       not yet confirmed that the goal is canceled.
    8. The goal received a cancel request before it started executing and was successfully
       canceled.
    9. An action client can determine that a goal is LOST. This should not be sent over the
       wire by an action server.
       
    I require my flag to be true when it comes to the 3rd terminal State. So, here comes an if
    condition.
    """
    
    if term_state == 0:
        print("Goal yet to be processed!\n")
        return
    if term_state == 1:
        print("Goal being processed!\n")
        return
    if term_state == 2:
        print("Goal received a cancel request after it started executing!\n")
        return
    if term_state == 3:
        print("Goal was achieved successfully!\n")
        # In this case flag_goal = True
        flag_goal = True
        return
    if term_state == 4:
        print("Goal was aborted!\n")
        return
    if term_state == 5:
        print("Goal was rejected!\n")
        return
    if term_state == 6:
        print("Goal received a cancel request before it started executing and not yet completed execution!\n")
        return
    if term_state == 7:
        print("The goal received a cancel request before it started executing, but the action server has not yet confirmed that the goal is canceled.\n")
        return
    if term_state == 8:
        print("The goal received a cancel request before it started executing and was successfully canceled.\n")
        return
    if term_state == 9:
        print("Goal is LOST!!\n")
        return    
    
    
    

def goalsetto_action_client(des_pos_x, des_pos_y):
    # 2 necessary variables
    global goal
    global client
    # goal has been defined as MoveBaseGoal(). I will be using that to get the position of the x
    # and y coordinates.
    goal.target_pose.pose.position.x = des_pos_x
    goal.target_pose.pose.position.y = des_pos_y
    """
    The send goal function has the following parameters:
    1. self
    2. goal
    3. done_cb which is None by Default ->
       Callback that gets called on transitions to Done. The callback should take 2 parameters:
       the terminal state as an integer from the actionlib_msgs/GoalStatus and the result.
    4. active_cb which is None by Default ->
       No-parameter callback that gets called on transitions to Active.
    5. feedback_cb which is None by Default ->
       Callback that gets called whenever feedback for this goal is received. Takes one
       parameter: the feedback. 
    So when the send_goal function is being used I am going to use a callback function that
    gives me a status as to whether the goal has been achieved or no. I have written the
    function above. 
    """
    client.send_goal(goal,goal_status)

def my_callback(event):
    print ("Time exceeded" + str(event.current_real))
    print("Failed to reach target in 60 seconds\n")
    rospy.set_param('active',0)
    

def main():

    """
    Things to do in the main function:
    1. Get necessary variables globally.
    2. Initialize the node. Keep the name of the node the same the one mentioned in the launch
       file.
    3. Subscribe to the Odometry.
    4. Initialize the action client and wait for the server.
    5. Things to remember:
       i. set the field goal.target_pose.header.frame_id to map
       ii. set the field goal.target_pose.pose.orientation.w to 1
    """
    
    flag = 0
        
    global goal
    global flag_goal
    global desired_position_x, desired_position_y
    global active_
    global client
    
    
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    
    rospy.init_node('behaviour_1')
    print(msg)
    
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    client.wait_for_server()
    
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.orientation.w = 1.0
    
    while(1):
        
        """
        One thing I can do here is to constantly keep updating my variables that include the
        desired x and y coordinates along with the active_. The active_ is a parameter that
        takes either a 0 or 1 to ensure if a mode is inactive or active respectively. So
        basically works with a blocking and unblocking feature.
        """
        active_ = rospy.get_param("/active")
        # Making sure that the target position of the x, y coordinates are communicated.
        desired_position_x = rospy.get_param('des_pos_x')
        desired_position_y = rospy.get_param('des_pos_y')
        """
        active_ = 1 will make the mode active whereas if it is 0 the mode will be inactive
        """
        if active_ == 1:
            if flag == 1:
                print("Moving towards desired goal\n")
                goalsetto_action_client(desired_position_x,desired_position_y)
                rospy.Timer(rospy.Duration(60), my_callback)
                flag = 0
                
                
        elif active_ == 0:
            if flag == 0 and flag_goal == False:
                print("Idle state!\n")
                client.cancel_goal()
                flag = 1
            if flag_goal == True:
                print("Back to Idle Mode\n")
                flag = 1
                flag_goal = False
    
    

if __name__ == "__main__":
    main()
    
