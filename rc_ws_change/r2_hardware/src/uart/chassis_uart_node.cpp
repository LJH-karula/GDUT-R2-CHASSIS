#include "ros/ros.h"
#include "r2_hardware/communica.h"
#include "thread"

ros::Subscriber sub;
control_base::CONTROL_BASE base;
r2_msgs::controller_cmd msgs;

void doMsg(r2_msgs::controller_cmd::ConstPtr Msg_p)
{
    msgs.cmd_vel.linear.x = Msg_p->cmd_vel.linear.x;
    msgs.cmd_vel.linear.y = Msg_p->cmd_vel.linear.y;
    msgs.cmd_vel.angular.z = Msg_p->cmd_vel.angular.z;
    msgs.chassis_ctrl_flag = Msg_p->chassis_ctrl_flag;
    msgs.next_controller_state = Msg_p->next_controller_state;
    msgs.take_ball_flag = Msg_p->take_ball_flag;
    // ROS_INFO("Send!!!");
}

void node3Function()
{
    ros::Rate rate(200);
    ros::NodeHandle nh;
    while(ros::ok())
    {
        base.CHASSIS_TO_STM32(msgs.cmd_vel.linear.x,msgs.cmd_vel.linear.y,
                                msgs.cmd_vel.angular.z,2,msgs.chassis_ctrl_flag,1);
        // ROS_INFO("Chassis!!!");
        rate.sleep();
    }
   
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"chassis");
    ros::NodeHandle nh;

    std::string chassis_serial_port_;
    sub = nh.subscribe<r2_msgs::controller_cmd>("/r2_cmd",1,doMsg); 
    nh.param<std::string>("chassis_serial_port",chassis_serial_port_,"/dev/chassis");
    base.ChassisSerialInit(chassis_serial_port_.data());
   
    std::thread node3Thread(node3Function);         //chassis
    ROS_INFO("open uart with chassis successfully!\n");
    ros::spin();
    return 0;
}
