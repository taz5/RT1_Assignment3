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
