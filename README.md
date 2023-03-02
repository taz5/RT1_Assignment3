# INTRODUCTION

The final assignment is focused on developing a software architecture for the control of the robot in the environment. The software will rely on the move_base and mapping packages for localizing the robot and plan the motion.

The architecture requests an input from the User, and lets the robot execute one of the following behaviours:

1. Autonomously reach a x,y coordinate inserted by the user.
2. Lets the user drive the robot with the keyboard
3. Let's the user drive the robot assisting them to avoid collisions.

All the nodes created in this assignment were made in python.

## INITIAL SETUP BEFORE STARTING TO CODE

The steps to be carried prior to creating the nodes would be as follows:

1. Create your workspace. In my case I called it RT1_Assignment3
2. Then create a src folder with RT1_Assignment3
3. Perform catkin_make

```bash
mkdir RT1_Assignment3
cd RT1_Assignment3
mkdir src
catkin_make
```
**Remember to source your workspace. You can do it either by using the .bashrc file or just source it within the terminal itself. The best practice it to source it in the bashrc file.**

To carry out this assignment, the following packages are required:

1. [final_assigment](https://github.com/CarmineD8/final_assignment.git)
2. [slam_gmapping](https://github.com/CarmineD8/slam_gmapping.git)

To perform this assignment, I used the docker created by Prof. Carmine. So the dependencies I needed to initially start off with this assignment were already included in it. All that is needed to be done is to go to the src folder of the workspace and clone the repositories containing the final_assignment and the slam_gmapping.
```bash
cd src
git clone https://github.com/CarmineD8/final_assignment.git
git clone https://github.com/CarmineD8/slam_gmapping.git
```
**Remember:** Once the repositories are cloned, we must check out to noetic.
```bash
git checkout noetic
```
That's all that is needed for the initial setup! Now to utilizing the Final_Assignment folder that was cloned.
### FINAL_ASSIGNMENT
The final assignment folder contains a series of folders that have to do with the creation of the map, the robot, configurations, parameters, and a CMakeLists.txt and package.xml. For this assignment, the focus remained only on the launch and scripts folders.

The launch file contains 4 launch files, 2 of which was created by me to launch nodes and all launch files in the folder.

**nodes_four.launch**

This launch file contains parameters and nodes.
```bash
<?xml version="1.0"?>
<launch>

<param name="active" type="int" value="0"/>
<param name="des_pos_x" type="double" value="1"/>
<param name="des_pos_y" type="double" value="1"/>

<node pkg="final_assignment" type="user_interface.py" name="user_interface" required="true" launch-prefix="xterm -bg black -fg green -e"/>
<node pkg="final_assignment" type="behaviour_1.py" name="behaviour_1" required="true" launch-prefix="xterm -bg black -fg green -e"/>
<node pkg="final_assignment" type="behaviour_2.py" name="behaviour_2" required="true" launch-prefix="xterm -bg black -fg green -e"/>
<node pkg="final_assignment" type="behaviour_3.py" name="behaviour_3" required="true" launch-prefix="xterm -bg black -fg green -e"/>
</launch>
```
**final.launch**
```bash
<?xml version="1.0"?>
<launch>
    <include file="$(find final_assignment)/launch/simulation_gmapping.launch"/>
    <include file="$(find final_assignment)/launch/move_base.launch"/>
    <include file="$(find final_assignment)/launch/nodes_four.launch"/>
</launch>
```

The scripts folder contains 4 nodes to perform the 3 behaviours mentioned at the beginning of this file. They are as follows:

1. user_interface
2. behaviour_1
3. behaviour_2
4. behaviour_3

**Remember:** In order to have these nodes running, it is extrememly important to make the respect files executable. This can be done with the command:
```bash
chmod +x user_interface.py
chmod +x behaviour_1.py
chmod +x behaviour_2.py
chmod +x behaviour_3.py
```

In order to work on this assignment, I have used **actionlib** instead of services. It is similar to services in teh sense that it sends a request to an action server in order to achieve some goal and will get a result. But unlike servies, while the action is being performed, an action server sends progress feedback to the client. Actions are useful when a response may take a significant length of time.

I have also used the **move_base** package as it provides an implementation of an action that, given a goal in the world, will attempt to reach it with a mobile base. The move_base node links together a global and local planner to accomplish its global navigation task.

Also, I have imported the **tf** package as it lets the user keep track of multiple coordinate frames over time. tf maintains the relationship between coordinate frames in a tree structure buffered in time, and lets the user transform points, vectors, etc between any two coordinate frames at any desired point in time.

