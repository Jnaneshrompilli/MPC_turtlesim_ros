#include <cppad/ipopt/solve.hpp>
#include <cppad/cppad.hpp>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "turtlesim/Pose.h"
#include <iostream>
#include <math.h>
#include "mpc_controller.h"

using namespace std;

// to store data of turtle1 and turtle2
turtlesim::Pose turtle1;
turtlesim::Pose turtle2;
geometry_msgs::Point goal;
geometry_msgs::Twist msg_vel;

// Limits to the constraints
int N=5;
float dt=0.1;
float x_lower, x_upper, y_lower, y_upper, theta_upper, theta_lower, v_lin_lower, v_lin_upper;
float v_ang_lower, v_ang_upper, a_lin_lower, a_lin_upper, a_ang_lower, a_ang_upper;

// Indexes for constraints
size_t x_start = 0;
size_t y_start = x_start + N;
size_t theta_start = y_start + N;
size_t vel_l_start = theta_start + N;
size_t vel_a_start = vel_l_start + N;
size_t ac_l_start = vel_a_start + N;
size_t ac_a_start = ac_l_start + N - 1;

namespace
{
    using CppAD::AD;

    class FG_eval
    {
    public:
        typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
        void operator()(ADvector &fg, const ADvector &x)
        {

            fg[0] = 0;

            for (int t = 0; t < N - 1; t++)
            {

                AD<double> x1 = x[x_start + t + 1];
                AD<double> y1 = x[y_start + t + 1];
                AD<double> theta1 = x[theta_start + t + 1];
                AD<double> v_l1 = x[vel_l_start + t + 1];
                AD<double> v_a1 = x[vel_a_start + t + 1];

                AD<double> x0 = x[x_start + t];
                AD<double> y0 = x[y_start + t];
                AD<double> theta0 = x[theta_start + t];
                AD<double> v_l0 = x[vel_l_start + t];
                AD<double> v_a0 = x[vel_a_start + t];

                AD<double> ac_l = x[ac_l_start + t];
                AD<double> ac_a = x[ac_a_start + t];

                fg[0] += (abs(CppAD::pow(goal.x - x1, 2) + CppAD::pow(goal.y - y1, 2))) / N;

                fg[1 + x_start + t] = x1 - (x0 + (v_l0 * CppAD::cos(theta0) * dt));
                fg[1 + y_start + t - 1] = y1 - (y0 + (v_l0 * CppAD::sin(theta0) * dt));
                fg[1 + theta_start + t - 2] = theta1 - (theta0 + v_a0 * dt);
                fg[1 + vel_l_start + t - 3] = (abs(CppAD::pow(turtle1.x - x1, 2) + CppAD::pow(turtle1.y - y1, 2)));
                fg[1 + vel_a_start + t - 4] = v_l1 - (v_l0 + ac_l * dt);
                fg[1 + ac_l_start + t - 5] = v_a1 - (v_a0 + ac_a * dt);
            }
            return;
        }
    };
}

// To load parmaters
void load_params()
{
    ros::NodeHandle nh;
    nh.getParam("/N", N);
    nh.getParam("/dt", dt);
    nh.getParam("/x_lower", x_lower);
    nh.getParam("/x_upper", x_upper);
    nh.getParam("/y_lower", y_lower);
    nh.getParam("/y_upper", y_upper);
    nh.getParam("/v_lin_lower", v_lin_lower);
    nh.getParam("/v_lin_upper", v_lin_upper);
    nh.getParam("/v_ang_upper", v_ang_upper);
    nh.getParam("/v_ang_lower", v_ang_lower);
    nh.getParam("/a_lin_lower", a_lin_lower);
    nh.getParam("/a_lin_upper", a_lin_upper);
    nh.getParam("/a_ang_lower", a_ang_lower);
    nh.getParam("/a_ang_upper", a_ang_upper);
    nh.getParam("/theta_lower", theta_lower);
    nh.getParam("/theta_upper", theta_upper);
}

geometry_msgs::Twist get_started(turtlesim::Pose turtle1_arg, turtlesim::Pose turtle2_arg, geometry_msgs::Point goal_arg)
{

    // Storing passed data of turtle1 and turtle2
    turtle1.x = turtle1_arg.x ;
    turtle1.y = turtle1_arg.y;
    turtle1.theta = turtle1_arg.theta;
    turtle1.angular_velocity = turtle1_arg.angular_velocity;
    turtle1.linear_velocity = turtle1_arg.linear_velocity;

    turtle2.x = turtle2_arg.x;
    turtle2.y = turtle2_arg.y;
    turtle2.theta = turtle2_arg.theta;
    turtle2.angular_velocity = turtle2_arg.angular_velocity;
    turtle2.linear_velocity = turtle2_arg.linear_velocity;
  
    goal.x = goal_arg.x;
    goal.y = goal_arg.y;

    bool ok ;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    // number of independent variables (domain dimension for f and g)
    size_t nx = ac_a_start + N - 1;
    // number of constraints (range dimension for g)
    size_t ng = ac_l_start + N - 6;
    // initial value of the independent variables
    Dvector xi(nx);
    Dvector xl(nx), xu(nx);

    // Lower and upper bounds of variables
    xi[x_start] = turtle2.x;
    xl[x_start] = turtle2.x;
    xu[x_start] = turtle2.x;
    for (int i = x_start + 1; i < y_start; i++)
    {
        xi[i] = turtle2.x;
        xl[i] = -15 ; //x_lower;
        xu[i] = 15  ; //x_upper;
    }

    xi[y_start] = turtle2.y;
    xl[y_start] = turtle2.y;
    xu[y_start] = turtle2.y;
    for (int i = y_start + 1; i < theta_start; i++)
    {
        xi[i] = turtle2.y;
        xl[i] = -15 ; //y_lower;
        xu[i] = 15 ; //y_upper;
    }

    xi[theta_start] = turtle2.theta;
    xl[theta_start] = turtle2.theta;
    xu[theta_start] = turtle2.theta;
    for (int i = theta_start + 1; i < vel_l_start; i++)
    {
        xi[i] = turtle2.theta;
        xl[i] = -3.14 ; //theta_lower;
        xu[i] = 3.14 ; //theta_upper;
    }

    xi[vel_l_start] = turtle2.linear_velocity;
    xl[vel_l_start] = turtle2.linear_velocity;
    xu[vel_l_start] = turtle2.linear_velocity;
    for (int i = vel_l_start + 1; i < vel_a_start; i++)
    {
        xi[i] = turtle2.linear_velocity;
        xl[i] = 0.0 ; //v_lin_lower;
        xu[i] = 3.5 ; //v_lin_upper;
    }

    xi[vel_a_start] = turtle2.angular_velocity;
    xl[vel_a_start] = turtle2.angular_velocity;
    xu[vel_a_start] = turtle2.angular_velocity;
    for (int i = vel_a_start + 1; i < ac_l_start; i++)
    {
        xi[i] = turtle2.angular_velocity;
        xl[i] = -1.57; // v_ang_lower;
        xu[i] = 1.57; //v_ang_upper;
    }
    for (int i = ac_l_start; i < ac_a_start; i++)
    {
        xi[i] = 0.1;
        xl[i] = -5 ; //a_lin_lower;
        xu[i] = 5; //a_lin_upper;
    }
    for (int i = ac_a_start; i < ac_a_start + N - 1; i++)
    {
        xi[i] = 0.1;
        xl[i] = -5 ; //a_ang_lower;
        xu[i] = 5 ; //a_ang_upper;
    }

    // lower and upper limits for x

    // lower and upper limits for g
    Dvector gl(ng), gu(ng);

    // Lower and upper limits for constraints
    for (int i = x_start; i < y_start - 1; i++)
    {
        gl[i] = -0.05;
        gu[i] = 0.05;
    }
    for (int i = y_start - 1; i < theta_start - 2; i++)
    {
        gl[i] = -0.05;
        gu[i] = 0.05;
    }
    for (int i = theta_start - 2; i < vel_l_start - 3; i++)
    {
        gl[i] = -0.05;
        gu[i] = 0.05;
    }
    for (int i = vel_l_start - 3; i < vel_a_start - 4; i++)
    {
        gl[i] = 5;
        gu[i] = 1000;
    }
    for (int i = vel_a_start - 4; i < ac_l_start - 5; i++)
    {
        gl[i] = -0.5;
        gu[i] = 0.5;
    }
    for (int i = ac_l_start - 5; i <= ac_a_start - 6; i++)
    {
        gl[i] = -0.5;
        gu[i] = 0.5;
    }
    // object that computes objective and constraints
    FG_eval fg_eval;

    // options
    std::string options;

    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";

    // Disables printing IPOPT creator banner
    options += "String  sb          yes\n";

    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";

    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, xi, xl, xu, gl, gu, fg_eval, solution);

    //cout<<"Solution x: "<<solution.x<<endl ;
    //cout<<"f(x): "<<solution.obj_value<<endl ;
    // Check some of the solution values
    //
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    cout << "\n\nStatus: " << solution.status;
    cout << "\nDistance: " << solution.obj_value;
    cout << "\n    Linear Velocity: " << solution.x[vel_l_start + 1] << "\n    Angular Velocity: " << solution.x[vel_a_start + 1];
    //cout << "1" << solution.g << endl;
    msg_vel.linear.x = solution.x[vel_l_start + 1];
    msg_vel.angular.z = solution.x[vel_a_start + 1];

    return msg_vel;
}