# Quadruped

## Kinematics

Kinematics following DH standard here: https://github.com/users/Tuna-on-a-bagel/projects/3?pane=issue&itemId=33689584

Leg joint angles are given in [ab/ad, shoulder, knee].T

foot positions are give in [x, y, z].T in **LEG BASE FRAME**

The robot does not haveunique closed form inverse kinematics solutions, this is solved via newtons method. To speed up solve time, a LUT is built offline with grid solutions and stored in roboConf.json which is then loaded into Leg().IK_solutions for each leg independently. while online, the robot can query the LUT looking for $min(norm(LUT_i - XYZ_{des, i}))$
