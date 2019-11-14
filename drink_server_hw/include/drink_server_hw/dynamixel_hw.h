#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <drink_server_hw/dxlib.h>

#define CW_Angle_Limit 6
#define CCW_Angle_Limit 8
#define Moving_Speed 32
#define COMPORT "/dev/ttyACM0"
#define BAUDRATE (500000)


class BeerRobo : public hardware_interface::RobotHW
{
  public:
    BeerRobo();
    ros::Time getTime() const {return ros::Time::now();}
    ros::Duration getPeriod() const {return ros::Duration(0.01);}

    void read(ros::Time, ros::Duration);

    void write(ros::Time, ros::Duration);


  private:

    ros::Publisher pub;
    ros::Subscriber sub;
    hardware_interface::VelocityJointInterface vel_joint_interface;
    hardware_interface::JointStateInterface joint_state_interface;
    double cmd_[2]={0,0};
    double pos_[2];
    double vel_[2];
    double eff_[2];
    uint16_t right_cmd_vel;
    uint16_t left_cmd_vel;

    TDeviceID dev;
    TErrorCode err;
    const int right_wheel_id = 2;
    const int left_wheel_id = 7;
    const double cmd_to_d = 57;

    uint8_t cmd_param[8];

};

