#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <drink_server_hw/dynamixel_hw.h>


int main(int argc, char **argv){


  ros::init(argc, argv, "locomotion_control");

  ros::NodeHandle nh;

  BeerRobo beerrobo(nh);

  controller_manager::ControllerManager cm(&beerrobo, nh);


  ros::Rate rate(1/beerrobo.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);

  spinner.start();

  while(ros::ok()){

    ros::Time now = beerrobo.getTime();

    ros::Duration dt = beerrobo.getPeriod();

    beerrobo.read(now, dt);
    cm.update(now, dt);
    beerrobo.write(now, dt);
    rate.sleep();

  }

}
