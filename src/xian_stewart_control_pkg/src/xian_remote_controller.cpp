#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>


using namespace std;

class Turtle 
{
    ros::NodeHandle n;
    ros::Subscriber sub;
    double input_alpha = 0;
    double input_beta = 0;
    double input_gamma = 0;
    double input_x = 0;
    double input_y = 0;
    double input_z = 168;  //杆长为160时，动平台水平对应的Z高度
    double step_length = 0.25 ;

    int front_left_mode_button_pre = 0;
    int front_left_mode_button_cur = 0;
    int front_right_mode_button_pre = 0;
    int front_right_mode_button_cur = 0;
    int controller_mode = 0;
public:


    // void callback(const sensor_msgs::Joy::ConstPtr &Joy);

    Turtle() 
    {
        
        sub = n.subscribe<sensor_msgs::Joy>("joy", 10, &Turtle::callback, this);
    }

    void callback(const sensor_msgs::Joy::ConstPtr &Joy) 
    {
        front_left_mode_button_pre = front_left_mode_button_cur;
        front_left_mode_button_cur = Joy->buttons[6];

        if(front_left_mode_button_pre == 0 && front_left_mode_button_cur==1)
        {   
            controller_mode+=1;
            if(controller_mode > 4)
            {
                controller_mode = 0;
            }
            
        }
        printf("controller_mode: %d \n", controller_mode);

        switch (controller_mode) 
        {
            case 0:
                // 当 expression 等于 constant1 时执行的代码
                break;
            case 1:
                // 当 expression 等于 constant2 时执行的代码
                break;
            case 2:
                // 当 expression 等于 constant2 时执行的代码
                break;
            case 3: // stewart平台x，y，z平移运动模式
                if(Joy->axes[6] == 1)
                {
                    input_x += step_length;
                }
                if(Joy->axes[6] == -1)
                {
                    input_x -= step_length;
                }
                ros::param::set("/xian_stewart_control_params_server/input_x", input_x);

                if(Joy->axes[7] == 1)
                {
                    input_y += step_length;
                }
                if(Joy->axes[7] == -1)
                {
                    input_y -= step_length;
                }
                ros::param::set("/xian_stewart_control_params_server/input_y", input_y);

                if(Joy->buttons[4] == 1)
                {
                    input_z += step_length;
                }
                if(Joy->buttons[0] == 1)
                {
                    input_z -= step_length;
                }
                ros::param::set("/xian_stewart_control_params_server/input_z", input_z);

                printf("input_x: %0.2f, input_y: %0.2f, input_z: %0.2f \n", input_x, input_y, input_z);

                break;
            case 4: // stewart平台x，y，z旋转运动模式
                if(Joy->axes[6] == 1)
                {
                    input_alpha += step_length;
                }
                if(Joy->axes[6] == -1)
                {
                    input_alpha -= step_length;
                }
                ros::param::set("/xian_stewart_control_params_server/input_alpha", input_alpha);

                if(Joy->axes[7] == 1)
                {
                    input_beta += step_length;
                }
                if(Joy->axes[7] == -1)
                {
                    input_beta -= step_length;
                }
                ros::param::set("/xian_stewart_control_params_server/input_beta", input_beta);

                if(Joy->buttons[4] == 1)
                {
                    input_gamma += step_length;
                }
                if(Joy->buttons[0] == 1)
                {
                    input_gamma -= step_length;
                }
                ros::param::set("/xian_stewart_control_params_server/input_gamma", input_gamma);

                printf("input_alpha: %0.2f, input_beta: %0.2f, input_gamma: %0.2f \n", input_alpha, input_beta, input_gamma);
                break;
            // 更多 case...
            default:
                // 当 expression 不匹配任何 case 时执行的代码（可选）
                break;
        }
        // 复位键
        front_right_mode_button_pre = front_right_mode_button_cur;
        front_right_mode_button_cur = Joy->buttons[7];
        if(front_right_mode_button_pre == 0 && front_right_mode_button_cur==1)
        {   
            input_alpha = 0;
            input_beta = 0;
            input_gamma = 0;
            input_x = 0;
            input_y = 0;
            input_z = 168;  
            ros::param::set("/xian_stewart_control_params_server/input_x", input_x);
            ros::param::set("/xian_stewart_control_params_server/input_y", input_y);
            ros::param::set("/xian_stewart_control_params_server/input_z", input_z);
            ros::param::set("/xian_stewart_control_params_server/input_alpha", input_alpha);
            ros::param::set("/xian_stewart_control_params_server/input_beta", input_beta);
            ros::param::set("/xian_stewart_control_params_server/input_gamma", input_gamma);
            printf("input_x: %0.2f, input_y: %0.2f, input_z: %0.2f \n", input_x, input_y, input_z);
            printf("input_alpha: %0.2f, input_beta: %0.2f, input_gamma: %0.2f \n", input_alpha, input_beta, input_gamma);    
            printf("完成复位\n");            
        }
        

        // input_alpha = Joy->axes[0] * 20;
        // input_beta = Joy->axes[1] * 20;
        // input_gamma = Joy->axes[2] * 20;

        // input_alpha = Joy->buttons[0];
        // input_beta = Joy->buttons[1];
        // input_gamma = Joy->buttons[2];


        // ros::param::set("/xian_stewart_control_params_server/input_alpha", input_alpha);
        // ros::param::set("/xian_stewart_control_params_server/input_beta", input_beta);
        // ros::param::set("/xian_stewart_control_params_server/input_gamma", input_gamma);

        // ROS_INFO("input_alpha: %.3f input_beta: %.3f, input_gamma: %.3f", input_alpha, input_beta, input_gamma);
    }
};




int main(int argc, char **argv) {
    ros::init(argc, argv, "turtle");
    Turtle turtle;
    ros::spin();
    return 0;
}


