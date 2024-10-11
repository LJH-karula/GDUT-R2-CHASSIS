#include "ros/ros.h"
#include "r2_hardware/communica.h"
#include "thread"

r2_msgs::controller_cmd msgs;
control_base::CONTROL_BASE base;
r2_msgs::stm32 upper;
ros::Subscriber sub;
ros::Publisher upper_pub;


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


// void node1Function()
// {
//     ros::Rate rate(50);
//     ros::NodeHandle nh;
//     while(ros::ok())
//     {
//         // ROS_INFO("Send!!!");
        
//         rate.sleep();
//         ros::spinOnce();
//     }
   
// }

// void node2Function()
// {
//     ros::Rate rate(50);
//     while (ros::ok())
//     {
        
//         upper_pub.publish(upper);
//          rate.sleep();
//          ros::spinOnce();
//     }
    
// }

void send_timer_callback(const ros::TimerEvent&)
{
    base.UPPER_TO_STM32(msgs.next_controller_state,msgs.take_ball_flag,3.14,0);
}

void read_timer_callback(const ros::TimerEvent&)
{
    base.ROS_READ_FROM_UPPER(upper.now_controller_state,upper.ball_state,
                                 upper.Put_ball_arrive,upper.lock_flag,upper.touch_flag,upper.touch_flag,
                                 upper.start_flag,upper.red_blue_flag,upper.restart_flag,upper.test_cnt);
    upper_pub.publish(upper);
}

int main(int argc, char *argv[])
{
    /* code */
    ros::init(argc,argv,"test");
    ros::NodeHandle nh;
    
    std::string upper_serial_port_;
    std::string chassis_serial_port_;
    sub = nh.subscribe<r2_msgs::controller_cmd>("/r2_cmd",1,doMsg); //use to acieve all cmd from ros
    upper_pub = nh.advertise<r2_msgs::stm32>("/stm32",1);
    nh.param<std::string>("stm32_serial_port", upper_serial_port_, "/dev/upper");
    // nh.param<std::string>("chassis_serial_port", chassis_serial_port_, "/dev/chassis");
    ros::Timer read_timer = nh.createTimer(ros::Duration(0.02),read_timer_callback);
    ros::Timer send_timer = nh.createTimer(ros::Duration(0.05),send_timer_callback);

    //base.ChassisSerialInit(chassis_serial_port_.data());
    base.SerialInit(upper_serial_port_.data());

    // std::thread node1Thread(node1Function);         //upper
    // std::thread node2Thread(node2Function);         //read
    ROS_INFO("open uart with stm32 successfully!\n");

    ros::spin();
    return 0;
}

