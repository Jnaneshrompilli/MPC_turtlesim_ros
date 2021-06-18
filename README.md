# MPC Path Planner

Implementing Model Predictive Control in Path Planning of turtlebot in ROS using Ipopt, a C++ library

Equations
----------
xt+1 = x[t] + v_l*cos(theta)
y[t+1] = y[t] + v_l*sin(theta)
theta[t+1] = theta[t] + 
