[![forthebadge](https://forthebadge.com/images/badges/built-with-love.svg)](https://forthebadge.com) [![forthebadge](https://forthebadge.com/images/badges/for-robots.svg)](https://forthebadge.com)

# Solution-to-Inverse-Kinematics-problem-using-Numerical-methods

### Abstract
Inverse Kinematics (IK) problem is defined as the problem of determining a set of appropriate joint configurations for
which the end effectors move to desired positions as smoothly, rapidly, and as accurately as possible. A few of the iterative
methods that solve tries to solve the problem have been described here. Further, we have prepared a MATLAB code that
uses one of the methods. We have also simulated the working of the methods in Unity Engine to help better visualize it.

### Demo
https://pushpendra.itch.io/robot-arm

![demoimage](https://github.com/pps-19012/Solution-to-Inverse-Kinematics-problem-using-Numerical-methods/blob/main/robotarm.png)

| File names | Function of those files: |  
| --- | --- |
| end_pos.m	| Calculates the position of the end effector using angles between the arms. |
Jacobian.m	             |   Consists of the function which calculates the Jacobian of the robotic system|  
Jacobian_Convergence.m	|    Calculates the position of end effector at each iteration of the transpose method and displays the converging and diverging case.|  
make_smatrix.m	   |       Converts linear path into n sub intervals and returns all the target points.  |
mid_pos.m         |       Calculates the position of the mid joint.  |
theta_Calculation.m |	    Calculates set of joint angles for the target position of end effector. |  
Path_tracking.m | 	          Main file that calculates and combines all the base functions and performs simulation of the results |




