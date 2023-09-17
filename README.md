# Quadruped

## Kinematics

Kinematics following DH standard here: https://github.com/users/Tuna-on-a-bagel/projects/3?pane=issue&itemId=33689584

Leg joint angles are given in [ab/ad, shoulder, knee].T

Leg frames have been assigned and can be seen here:

<img src="https://user-images.githubusercontent.com/51982197/268480703-fc66cad1-2a8c-4a43-9396-b6a2c6a0dee0.png" alt="Image" width="400">
<img src="https://user-images.githubusercontent.com/51982197/268480706-da3b65ca-fc4b-4c03-9838-1087ae24d1ad.png" alt="Image" width="400">


foot positions are give in [x, y, z].T in **LEG BASE FRAME**

The robot does not haveunique closed form inverse kinematics solutions, this is solved via newtons method. To speed up solve time, a LUT is built offline with grid solutions and stored in roboConf.json which is then loaded into Leg().IK_solutions for each leg independently. while online, the robot can query the LUT looking for $min(norm(LUT_i - XYZ_{des, i}))$, which can then be used as the initial guess for newtons method.




## Dynamics
