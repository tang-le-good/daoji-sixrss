#include "ros/ros.h"
#include "std_msgs/Float64.h"
// #include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
// #include "zpmc_cv_control.h"
#include <tf/transform_datatypes.h>
#include "math.h"
class XianDuojiStewartControl {
public:
    XianDuojiStewartControl() 
    {
        ros::NodeHandle nh;

        // Publishers for the wheel velocity controllers
        arm1_position_pub = nh.advertise<std_msgs::Float64>("/arm_joint1_position_controller/command", 10);
        arm2_position_pub = nh.advertise<std_msgs::Float64>("/arm_joint2_position_controller/command", 10);
        arm3_position_pub = nh.advertise<std_msgs::Float64>("/arm_joint3_position_controller/command", 10);
        arm4_position_pub = nh.advertise<std_msgs::Float64>("/arm_joint4_position_controller/command", 10);
        arm5_position_pub = nh.advertise<std_msgs::Float64>("/arm_joint5_position_controller/command", 10);
        arm6_position_pub = nh.advertise<std_msgs::Float64>("/arm_joint6_position_controller/command", 10);
        
        // 订阅电机反馈的消息
        joint_state_sub = nh.subscribe("/joint_states", 10, &XianDuojiStewartControl::jointStateCallback, this);
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) 
    {
        // Assuming that the joint names are "left_wheel" and "right_wheel"
        // Find the indices of the left and right wheel joints
        
        arm1_position_cmd.data = 3.14;
        arm2_position_cmd.data = 3.14;
        arm3_position_cmd.data = 3.14;
        arm4_position_cmd.data = 3.14;
        arm5_position_cmd.data = 3.14;
        arm6_position_cmd.data = 3.14;
        arm1_position_pub.publish(arm1_position_cmd);
        arm2_position_pub.publish(arm2_position_cmd);
        arm3_position_pub.publish(arm3_position_cmd);
        arm4_position_pub.publish(arm4_position_cmd);
        arm5_position_pub.publish(arm5_position_cmd);
        arm6_position_pub.publish(arm6_position_cmd);

        ROS_INFO("Command - Left: %.3f, Right: %.3f", arm1_position_cmd.data, arm2_position_cmd.data);

    }


private:
    ros::Publisher arm1_position_pub;
    ros::Publisher arm2_position_pub;
    ros::Publisher arm3_position_pub;
    ros::Publisher arm4_position_pub;
    ros::Publisher arm5_position_pub;
    ros::Publisher arm6_position_pub;
    ros::Subscriber joint_state_sub;
    // Example command: set wheel velocities
    std_msgs::Float64  arm1_position_cmd;
    std_msgs::Float64  arm2_position_cmd;
    std_msgs::Float64  arm3_position_cmd;
    std_msgs::Float64  arm4_position_cmd;
    std_msgs::Float64  arm5_position_cmd;
    std_msgs::Float64  arm6_position_cmd;
    

    // 车轮反馈速度
    double v1 = 0;
    double v2 = 0;

    

    



};

int main(int argc, char **argv) {
    ros::init(argc, argv, "xian_duoji_stewart_control");

    XianDuojiStewartControl node;
    ros::spin();  // Keep the node running and processing callbacks
    return 0;
}