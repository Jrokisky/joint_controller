#include <ros/ros.h>
#include <ros/console.h>
#include <dynamixel_workbench_msgs/JointCommand.h>
#include <string>
#include <cmath>

int main(int argc, char** argv) {
  ros::init(argc, argv, "joint_controller");
  ros::NodeHandle nh("joint_controller");

  ros::ServiceClient joint_command_client = 
	  nh.serviceClient<dynamixel_workbench_msgs::JointCommand>("/joint_command");
  joint_command_client.waitForExistence();
 
  float goal_position = 0.0;

  // Dynamixel unit.
  std::string unit = nh.param<std::string>("unit", "rad");
  // Rotation increment.
  float goal_pos_inc = nh.param<float>("goal_pos_inc", 0.0);
  // Maximum rotation.
  float max_rad = nh.param<float>("max_rad", 0.0);
  // Dynamixel Id.
  int id = nh.param<int>("dynamixel_id", 0);

  ros::Rate r(1);
  while (ros::ok()) 
  {
    dynamixel_workbench_msgs::JointCommand joint_command;
    joint_command.request.unit = unit;
    joint_command.request.id = id;
    joint_command.request.goal_position = goal_position;

    if (joint_command_client.call(joint_command))
    {
      if (!joint_command.response.result)
        ROS_INFO("Failed to write goal_position: %f", goal_position);
      else
        ROS_INFO("Wrote goal_position: %f", goal_position);
    }
    else
    {
      ROS_INFO("Failed to call service");
    }

    // Head in the other direction.
    float tmp = goal_position + goal_pos_inc;
    if (std::abs(tmp) > max_rad) 
      goal_pos_inc *= -1.0;

    goal_position += goal_pos_inc;
    r.sleep();
    ros::spinOnce();
  }
}

