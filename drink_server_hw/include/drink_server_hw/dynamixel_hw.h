#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <drink_server_hw/dxlib.h>
#include <std_msgs/Bool.h>

#define CW_Angle_Limit 6
#define CCW_Angle_Limit 8
#define Moving_Speed 32
#define Goal_Position 30
#define Torque_Limit 34
#define Max_Torque 14
#define COMPORT "/dev/ttyACM1"
#define BAUDRATE (500000)


class BeerRobo : public hardware_interface::RobotHW
{
  public:
    BeerRobo(ros::NodeHandle nh);
    ros::Time getTime() const {return ros::Time::now();}
    ros::Duration getPeriod() const {return ros::Duration(0.01);}
    ros::NodeHandle node_handle;

    void read(ros::Time, ros::Duration);

    void write(ros::Time, ros::Duration);

    void right_pusherCallback(const std_msgs::Bool& msg);
    void left_pusherCallback(const std_msgs::Bool& msg);
    void right_stopperCallback(const std_msgs::Bool& msg);
    void left_stopperCallback(const std_msgs::Bool& msg);

  private:

    ros::Publisher pub;
    ros::Subscriber sub;
    ros::Subscriber r_p_sub;
    ros::Subscriber l_p_sub;
    ros::Subscriber r_s_sub;
    ros::Subscriber l_s_sub;
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
    const int right_stopper_id = 8;
    const int left_stopper_id = 1;
    const int right_pusher_id = 3;
    const int left_pusher_id = 6;
    const double cmd_to_d = 57;

    uint16_t right_push_on;
    uint16_t left_push_on;
    uint16_t right_stop_on;
    uint16_t left_stop_on;
    uint16_t right_push_off;
    uint16_t left_push_off;
    uint16_t right_stop_off;
    uint16_t left_stop_off;
    uint16_t stopper_lim;
    uint16_t pusher_lim;
    

    uint8_t cmd_param[8];

};

