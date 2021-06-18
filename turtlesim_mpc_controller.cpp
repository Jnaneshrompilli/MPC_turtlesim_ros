#include <cppad/ipopt/solve.hpp>
#include <cppad/cppad.hpp>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "turtlesim/Pose.h"
#include <iostream>
#include <math.h>

using namespace std;

turtlesim::Pose turtle1_pose;
turtlesim::Pose turtle2_pose;
geometry_msgs::Point goal_pose;
geometry_msgs::Twist msg;
ros::Publisher vel_pub;

float distance_t1_t2 = 0;

int N = 5;
float dt = 0.1;

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

                    // Fortran style indexing

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

                    fg[0] += (abs(CppAD::pow(goal_pose.x - x1, 2) + CppAD::pow(goal_pose.y - y1, 2))) / N;

               

                    fg[1 + x_start + t] = x1 - (x0 + (v_l0 * CppAD::cos(theta0) * dt));
                    fg[1 + y_start + t - 1] = y1 - (y0 + (v_l0 * CppAD::sin(theta0) * dt));
                    fg[1 + theta_start + t - 2] = theta1 - (theta0 + v_a0 * dt);
                    fg[1 + vel_l_start + t - 3] = (abs(CppAD::pow(turtle1_pose.x - x1, 2) + CppAD::pow(turtle1_pose.y - y1, 2)));
                    fg[1 + vel_a_start + t - 4] = v_l1 - (v_l0 + ac_l * dt);
                    fg[1 + ac_l_start + t - 5] = v_a1 - (v_a0 + ac_a * dt);
               }
               return;
          }
     };
}

bool get_started(void)
{
     bool ok = true;
     float velocities[2];
     size_t i;
     typedef CPPAD_TESTVECTOR(double) Dvector;

     // number of independent variables (domain dimension for f and g)
     size_t nx = ac_a_start + N - 1;
     // number of constraints (range dimension for g)
     size_t ng = ac_l_start + N - 6;
     // initial value of the independent variables
     Dvector xi(nx);
     Dvector xl(nx), xu(nx);

     xi[x_start] = turtle2_pose.x;
     xl[x_start] = turtle2_pose.x;
     xu[x_start] = turtle2_pose.x;
     for (int i = x_start + 1; i < y_start; i++)
     {
          xi[i] = turtle2_pose.x;
          xl[i] = -15;
          xu[i] = 15;
     }

     xi[y_start] = turtle2_pose.y;
     xl[y_start] = turtle2_pose.y;
     xu[y_start] = turtle2_pose.y;
     for (int i = y_start + 1; i < theta_start; i++)
     {
          xi[i] = turtle2_pose.y;
          xl[i] = -15;
          xu[i] = 15;
     }

     xi[theta_start] = turtle2_pose.theta;
     xl[theta_start] = turtle2_pose.theta;
     xu[theta_start] = turtle2_pose.theta;
     for (int i = theta_start + 1; i < vel_l_start; i++)
     {
          xi[i] = turtle2_pose.theta;
          xl[i] = -3.14;
          xu[i] = +3.14;
     }

     xi[vel_l_start] = turtle2_pose.linear_velocity;
     xl[vel_l_start] = turtle2_pose.linear_velocity;
     xu[vel_l_start] = turtle2_pose.linear_velocity;
     for (int i = vel_l_start + 1; i < vel_a_start; i++)
     {
          xi[i] = turtle2_pose.linear_velocity;
          xl[i] = 0.0;
          xu[i] = 3.5;
     }

     xi[vel_a_start] = turtle2_pose.angular_velocity;
     xl[vel_a_start] = turtle2_pose.angular_velocity;
     xu[vel_a_start] = turtle2_pose.angular_velocity;
     for (int i = vel_a_start + 1; i < ac_l_start; i++)
     {
          xi[i] = turtle2_pose.angular_velocity;
          xl[i] = -3.14 / 2;
          xu[i] = 3.14 / 2;
     }
     for (int i = ac_l_start; i < ac_a_start; i++)
     {
          xi[i] = 0.1;
          xl[i] = -1;
          xu[i] = 1;
     }
     for (int i = ac_a_start; i < ac_a_start + N - 1; i++)
     {
          xi[i] = 0.1;
          xl[i] = -1;
          xu[i] = 1;
     }

     // lower and upper limits for x

     // lower and upper limits for g
     Dvector gl(ng), gu(ng);
     for (int i = x_start; i < y_start - 1; i++)
     {
          gl[i] = -0.15;
          gu[i] = 0.15;
     }
     for (int i = y_start - 1; i < theta_start - 2; i++)
     {
          gl[i] = -0.15;
          gu[i] = 0.15;
     }
     for (int i = theta_start - 2; i < vel_l_start - 3; i++)
     {
          gl[i] = -0.15;
          gu[i] = 0.15;
     }
     for (int i = vel_l_start - 3; i < vel_a_start - 4; i++)
     {
          gl[i] = 5;
          gu[i] = 1000;
     }
     for (int i = vel_a_start - 4; i < ac_l_start - 5; i++)
     {
          gl[i] = -5;
          gu[i] = 5;
     }
     for (int i = ac_l_start - 5; i <= ac_a_start - 6; i++)
     {
          gl[i] = -5;
          gu[i] = 5;
     }
     // object that computes objective and constraints
     FG_eval fg_eval;

     // options
     std::string options;
     // turn off any printing
     options += "Integer print_level  0\n";
     options += "String  sb           yes\n";
     // maximum number of iterations
     options += "Integer max_iter     5000\n";
     // approximate accuracy in first order necessary conditions;
     // see Mathematical Programming, Volume 106, Number 1,
     // Pages 25-57, Equation (6)
     options += "Numeric tol          1e-6\n";
     // derivative testing
     options += "String  derivative_test            second-order\n";
     // maximum amount of random pertubation; e.g.,
     // when evaluation finite diff
     options += "Numeric point_perturbation_radius  0.\n";

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
     msg.linear.x = solution.x[vel_l_start + 1];
     msg.angular.z = solution.x[vel_a_start + 1];
     vel_pub.publish(msg);

     return ok;
}

void mySigintHandler(int sig)
{
     ros::shutdown();
}

// Update the position of turtle 1
void turtle1_update(const turtlesim::Pose::ConstPtr &pose_message)
{
     turtle1_pose.x = pose_message->x;
     turtle1_pose.y = pose_message->y;
     turtle1_pose.theta = pose_message->theta;
     turtle1_pose.linear_velocity = pose_message->linear_velocity;
     turtle1_pose.angular_velocity = pose_message->angular_velocity;
}

// Update the positio of turtle 2
void turtle2_update(const turtlesim::Pose::ConstPtr &pose_message)
{
     turtle2_pose.x = pose_message->x;
     turtle2_pose.y = pose_message->y;
     turtle2_pose.theta = pose_message->theta;
     turtle2_pose.linear_velocity = pose_message->linear_velocity;
     turtle2_pose.angular_velocity = pose_message->angular_velocity;
}

// Distance between two points
float distance_calc(float x1, float x2, float y1, float y2)
{
     return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

void goal_input()
{
     cout << "\nEnter goal x(>1): ";
     cin >> goal_pose.x;
     cout << "\nEnter goal y(>1): ";
     cin >> goal_pose.y;
}

int main(int argc, char **argv)
{

     char choice;
     goal_input();
     ros::init(argc, argv, "turtlesim_mpc_controller");

     ros::NodeHandle n;

     vel_pub = n.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 1000);
     ros::Subscriber pose1_sub = n.subscribe("turtle1/pose", 10, turtle1_update);
     ros::Subscriber pose2_sub = n.subscribe("turtle2/pose", 10, turtle2_update);

     ros::Rate loop_rate(10);

     //float distance_t1_goal = distance_calc(goal_pose.x, turtle1_pose.x, goal_pose.y, turtle1_pose.y);

     msg.linear.x = 0.2;
     msg.angular.z = 0.0;
     vel_pub.publish(msg);
     while (ros::ok())
     {
          float distance_t1_goal = distance_calc(goal_pose.x, turtle2_pose.x, goal_pose.y, turtle2_pose.y);
          if (abs(distance_t1_goal) < 0.60)
          {
               msg.linear.x = 0.0;
               msg.angular.z = 0.0;
               vel_pub.publish(msg);
               break;
          }
          get_started();
          //vel_pub.publish(msg);
          ros::spinOnce();
          loop_rate.sleep();
     }
     cout << "\nTry Again (Y/N): ";
     cin >> choice;
     if (choice == 'y' || choice == 'Y')
     {
          return main(argc, argv);
     }

     cout << "\n\n*****Mission Completed*****" << endl;

     return 0;
}