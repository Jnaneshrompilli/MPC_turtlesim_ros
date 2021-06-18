# MPC Path Planner

Implementing Model Predictive Control in Path Planning of turtlebot in ROS using Ipopt, a C++ library

Equations
----------
x_(t+1)= x_t+v_l*cos⁡(θ)*dt
y_(t+1)=y_t+v_l*sin⁡(θ)*dt
θ_(t+1)= θ_t+ v_a*dt
v_l^(t+1)=v_l^t+a_l*dt
v_a^(t+1)=v_a^t+a_a*dt

