#include "r2_hardware/communica.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "r2_hardware/pub_process.h"

std::string field_direction;
int mirror_flag;



int main(int argc, char * argv[])
{
    ros::init(argc, argv, "ros_write_to_stm32");
    setlocale(LC_ALL, "");
    ros::NodeHandle nh;
    uart_stm32 uart_stm32;
    ROS_INFO("ros to stm32 information process start");
    if(!nh.param<std::string>("field_direction",field_direction,"right"))
        ROS_ERROR("GET FIELD DIRECTION FAILD");
    if(field_direction=="left")
    {
        mirror_flag=-1;
    }
    else
    {
        mirror_flag=1;
    }
    ros::Rate loop_rate(100);
    
    while (ros::ok())
    {
        uart_stm32.run();
        loop_rate.sleep();
        ros::spinOnce();    
    }

    return 0;
}

void uart_stm32::run()
{
    //publish all commands
    cmd_pub.publish(controller);
}

uart_stm32::uart_stm32()
{
    // if(!nh.param<int>("/if_use_gazebo",if_use_gazebo,1))
    // {
    //     ROS_ERROR("Failed to get param 'if_use_gazebo', auto to use gazebo");
    // }
    
    //use to accieve chassis cmd
    chassis_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel",1,&uart_stm32::do_vel_Msg,this);

    //use to acieve controller cmd without chassis
    upper_sub = nh.subscribe<r2_msgs::controller_cmd>("r2_control",10,&uart_stm32::control_callback,this);   
    timer = nh.createTimer(ros::Duration(0.5), &uart_stm32::timer_callback,this);// 创建定时器,触发的时间间隔为0.5秒
    //use to send all control to hardware
    cmd_pub = nh.advertise<r2_msgs::controller_cmd>("r2_cmd",10);
    
    controller.next_controller_state = CONTROLLER_OFF;
    controller.take_ball_flag = INVERSE_BALL;
    controller.chassis_ctrl_flag = 0;
    
}

uart_stm32::~uart_stm32()
{
}

void uart_stm32::do_vel_Msg(const geometry_msgs::Twist::ConstPtr &msg_p)
{
    // if(if_use_gazebo==0)
    // {
    if(mirror_flag==1)
    {
        controller.cmd_vel.linear.x = msg_p->linear.y;
        controller.cmd_vel.linear.y = msg_p->linear.x;
    }
    else
    {
        controller.cmd_vel.linear.x = -msg_p->linear.y;
        controller.cmd_vel.linear.y = -msg_p->linear.x;
    }
        
        controller.cmd_vel.angular.z = msg_p->angular.z;
        // ROS_INFO("Send chassis cmd successful!\r\n");
    //}
}


void uart_stm32::control_callback(const r2_msgs::controller_cmd::ConstPtr &msg_p)
{
    controller.chassis_ctrl_flag = msg_p->chassis_ctrl_flag;
    controller.next_controller_state = msg_p->next_controller_state;
    controller.take_ball_flag = msg_p->take_ball_flag;
    last_msg_time = ros::Time::now();   //获取更新上层控制命令的时间戳
}


void uart_stm32::timer_callback(const ros::TimerEvent&)
{
    //超时1s，把上层控制命令置0
    if((ros::Time::now() - last_msg_time).toSec() >= 1.0f)
    {
        controller.next_controller_state = CONTROLLER_OFF;
        // ROS_INFO("Reset the shangceng Controller state\r\n");
    }
}

