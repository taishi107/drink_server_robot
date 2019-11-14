#include <ros/package.h>
#include <drink_server_hw/dynamixel_hw.h>
#include <iostream>
#include <math.h>


BeerRobo::BeerRobo(){

  hardware_interface::JointStateHandle state_handle_right("right_wheel", &pos_[0], &vel_[0], &eff_[0]);
  joint_state_interface.registerHandle(state_handle_right);

  hardware_interface::JointStateHandle state_handle_left("left_wheel", &pos_[1], &vel_[1], &eff_[1]);
  joint_state_interface.registerHandle(state_handle_left);


  registerInterface(&joint_state_interface);


  hardware_interface::JointHandle vel_handle_right(joint_state_interface.getHandle("right_wheel"), &cmd_[0]);
  vel_joint_interface.registerHandle(vel_handle_right);

  hardware_interface::JointHandle vel_handle_left(joint_state_interface.getHandle("left_wheel"), &cmd_[1]);
  vel_joint_interface.registerHandle(vel_handle_left);


  registerInterface(&vel_joint_interface);
}

void BeerRobo::read(ros::Time time, ros::Duration period){

}

void BeerRobo::write(ros::Time time, ros::Duration period){

  ROS_ERROR("cmd0:%f cmd1:%f", cmd_[0], cmd_[1]);

}



