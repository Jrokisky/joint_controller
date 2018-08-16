#include <ros/ros.h>
#include <ros/console.h>
#include <dynamixel_workbench_msgs/JointCommand.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "joint_controller");
  ros::NodeHandle nh;

  ros::ServiceClient joint_command_client = 
	  nh.serviceClient<dynamixel_workbench_msgs::JointCommand>("joint_command");
 
  float goal_position = 5.0;

  ros::Rate r(1);
  while (ros::ok()) 
  {
    dynamixel_workbench_msgs::JointCommand joint_command;
    joint_command.request.unit = "AX-12A"; //TODO: make param. 
    joint_command.request.id = 1;
    joint_command.request.goal_position = goal_position;

    if (joint_command_client.call(joint_command))
    {
      if (!joint_command.response.result)
        ROS_INFO("Failed to write goal_position");
    }
    else
    {
      ROS_INFO("Failed to call service");
    }

    goal_position += 5.0;
    r.sleep();
    ros::spinOnce();
  }
}
