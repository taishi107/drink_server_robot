#include <ros/package.h>
#include <drink_server_hw/dynamixel_hw.h>
#include <iostream>
#include <math.h>
#include <algorithm>


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


  //////////////dynamixel_setting/////////
  if ((dev = DX_OpenPort (COMPORT, BAUDRATE))) {
    ROS_ERROR("Open success");
    int i;
    uint32_t num=20;
    TDxAlarmStatus stat[20];
    if (DX_Ping2 (dev, &num, stat, &err)) {
      for (i = 0; i < num; i++)
        ROS_ERROR("Found ID=%d %02X", stat[i].id, stat[i].Status);
    }

    ////ENdlessTurn_Mode On/////////
    DX_WriteWordData(dev, right_wheel_id, CCW_Angle_Limit, 0, &err);
    DX_WriteWordData(dev, right_wheel_id, CW_Angle_Limit, 0, &err);
    DX_WriteWordData(dev, left_wheel_id, CCW_Angle_Limit, 0, &err);
    DX_WriteWordData(dev, left_wheel_id, CW_Angle_Limit, 0, &err);

    ROS_INFO("EndlessTurn_Mode_ON");	
	
  }else{
    ROS_ERROR("Open_error");
  }



}

void BeerRobo::read(ros::Time time, ros::Duration period){


}

void BeerRobo::write(ros::Time time, ros::Duration period){

  //ROS_ERROR("cmd0:%f cmd1:%f", cmd_[0], cmd_[1]);
  if(cmd_[0] < 0){
    cmd_[0] = -cmd_[0] * cmd_to_d;
    right_cmd_vel = std::min(1023.0, cmd_[0]);

  }else{
    cmd_[0] = cmd_[0] * cmd_to_d;
    cmd_[0] = std::min(1023.0, cmd_[0]);
    right_cmd_vel = cmd_[0] + 1024;
  }

  if(cmd_[1] < 0){
    cmd_[1] = -cmd_[1] * cmd_to_d;
    cmd_[1] = std::min(1023.0, cmd_[1]);
    left_cmd_vel = cmd_[1] + 1024;

  }else{
    cmd_[1] = cmd_[1] * cmd_to_d ;
    left_cmd_vel = std::min(1023.0, cmd_[1]);
  }
  //ROS_ERROR("right_vel:%d left_vel:%d", right_cmd_vel, left_cmd_vel);


  cmd_param[0] = Moving_Speed;
  cmd_param[1] = 2;
  cmd_param[2] = right_wheel_id;
  cmd_param[3] = right_cmd_vel & 0xff; 
  cmd_param[4] = right_cmd_vel >> 8;
  cmd_param[5] = left_wheel_id;
  cmd_param[6] = left_cmd_vel & 0xff;
  cmd_param[7] = left_cmd_vel >> 8;

  //DX_WriteWordData(dev, right_wheel_id, Moving_Speed, right_cmd_vel, &err);
  //DX_WriteWordData(dev, left_wheel_id, Moving_Speed, left_cmd_vel, &err);


  DX_WriteSyncData(dev, cmd_param, 8, &err);


}



