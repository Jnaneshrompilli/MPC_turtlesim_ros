#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "turtlesim/Pose.h"
#include <turtlesim/Spawn.h>
#include <iostream>
#include <math.h>
#include "mpc_controller.h"



using namespace std;

turtlesim::Pose turtle1_pose;
turtlesim::Pose turtle2_pose;
geometry_msgs::Point goal_pose;
geometry_msgs::Twist msg;
ros::Publisher vel_pub;


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

// Input from user
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
     
     ros::init(argc, argv, "turtlesim_mpc_controller");
     ros::NodeHandle n;

     // Calling service to spawn turtle2
     ros::ServiceClient spawnclient = n.serviceClient<turtlesim::Spawn>("spawn") ;
     turtlesim::Spawn::Request req ;
     turtlesim::Spawn::Response resp ;
     req.x = 0 ;
     req.y = 0 ;
     req.theta = 0.78 ;
     req.name = "turtle2" ;
     spawnclient.waitForExistence();
     bool success_call = spawnclient.call(req,resp); 

     goal_input();

     vel_pub = n.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 1000);
     ros::Subscriber pose1_sub = n.subscribe("turtle1/pose", 10, turtle1_update);
     ros::Subscriber pose2_sub = n.subscribe("turtle2/pose", 10, turtle2_update);

     ros::Rate loop_rate(10);

     load_params() ;

     msg.linear.x = 0.2;
     msg.angular.z = 0.0;
     vel_pub.publish(msg);

     // calling mpc_controller
     while (ros::ok())
     {
          float distance_t1_goal = distance_calc(goal_pose.x, turtle2_pose.x, goal_pose.y, turtle2_pose.y);

          // If goal reached
          if (abs(distance_t1_goal) < 0.60)
          {
               msg.linear.x = 0.0;
               msg.angular.z = 0.0;
               vel_pub.publish(msg);
               break;
          }
          msg = get_started(turtle1_pose,turtle2_pose,goal_pose) ;
          vel_pub.publish(msg);
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