/*
 * Copyright (c) 2020, Cooneo Robot, Inc
 * All rights reserved.
 * Author: Zhenghao Li
 * Date: 2020.7.10
 */

#include <ros/ros.h>
#include <queue>
#include <thread>
#include <serial/serial.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <unistd.h>

#define PI 3.1415926 //535
static double Angle;
static bool recv_nav_tag = false;

struct Vel {
        double linear = 0.0;
        double angle = 0.0;
};

class MotorAdaptor {
public:
ros::NodeHandle n;
ros::Subscriber sub_nav_;
ros::Publisher odom_pub_;


std::queue<Vel> nav_queue_;
serial::Serial ser_car;

tf::TransformBroadcaster tb_;
ros::Time lastest_response_time_;
ros::Time lastest_send_time_;

double lastest_linear_ = 0;
double lastest_angle_ = 0;
double odom_linear_x_ = 0;
double odom_linear_y_ = 0;
double odom_angle_ = 0;

double vehicle_linear;
double vehicle_angle;
int16_t Encoder_right = 0;
int16_t Encoder_left = 0;
int16_t Steer_angle = 0;
double diff_send;

/******************************  参数 ********************************/
double max_angle_ = 3.1415926/4.0;                                                  //最大角速度 0.78，
double car_length_ = 0.355;                                                                   //后轴中心 与 前轮连杆中心之间的距离: 0.355 m

/****************************************************************************************************************/

void nav_callback(const geometry_msgs::TwistConstPtr &msg);
bool calcVelocity(uint8_t *, double &, double &, double &Yaw,double diff);
uint8_t calcByteToWrite(int16_t &, int16_t &, int16_t &, Vel & );
uint8_t check_data(uint8_t cmd[],int begin,int end);
void send_thread();
void rec_encoders();
void execute();

MotorAdaptor()
{
        sub_nav_ = n.subscribe("cmd_vel", 5, &MotorAdaptor::nav_callback, this);  

        odom_pub_ = n.advertise<nav_msgs::Odometry>("wheel_odom", 10);

        ser_car.setPort("/dev/ttyUSB0");         // 这里的端口需要自己实际环境 调整。
        ser_car.setBaudrate(115200);
        serial::Timeout timeout1 = serial::Timeout::simpleTimeout(10);
        ser_car.setTimeout(timeout1);
        ser_car.open();


        lastest_response_time_ = ros::Time::now();
        lastest_send_time_ = ros::Time::now();

        //打开两个线程，分别执行两个任务函数
        std::thread s(&MotorAdaptor::send_thread, this);
        s.detach();

        std::thread e(&MotorAdaptor::rec_encoders, this);
        e.detach();

        ROS_INFO("Initial successfully!");
}
};


/********************************************************************************************************************************************************
* 当下位机收到 ”0xff,0xfe,'s'“指令后，会发送imu，编码器的数据
* 帧头  帧头  标志位	 tx[0]	  tx[1]	     tx[2] tx[3]	 tx[4]   tx[5]	             tx[6]	               tx[7]	                 tx[8]	            tx[9]                tx[10]	       tx[11]             校验	    共16字节
* 0xff	0xfe  ‘e’	   0	     0  	     0           0	      0	     motorA_H8   motorA_L8	motorB_H8	motorB_L8  方向     舵机角度_H8	  舵机角度_L8 异或校验
*
********************************************************************************************************************************************************/
void MotorAdaptor::rec_encoders()
{
        uint8_t resp[16];
        uint8_t encoder_cmd[3]={0xff,0xfe,'e'};
        double vel_x;
        double vel_y;
        double diff;
        double Yaw;
        double average_linear;
        double average_angle;
        nav_msgs::Odometry odom_data;
        geometry_msgs::Quaternion q;
        tf::StampedTransform transform_send;

        odom_data.header.frame_id = "wheel_odom";
        odom_data.child_frame_id = "base_link";

        transform_send.child_frame_id_ = "base_link";
        transform_send.frame_id_ = "wheel_odom";

        ros::Rate r(200);
        ROS_INFO("Start read encoder");
        while(ros::ok())
        {
                ser_car.write(encoder_cmd,3);                                             //发送获取编码值标志位
                if(ser_car.read(resp, 16) == 16)                                          //读取有效数据位
                {
                        // ROS_INFO("encoder recv: %x, %x, %x, %x ,%x, %x, %x, %x,%x, %x, %x, %x ,%x, %x, %x, %x", resp[0], resp[1], resp[2],resp[3], resp[4], resp[5],resp[6],resp[7],resp[8],resp[9],resp[10],resp[11],resp[12],resp[13],resp[14],resp[15]);

                        ros::Time now = ros::Time::now();
                        diff = (now - lastest_response_time_).toSec();
                        Yaw  =0;
                        if(!calcVelocity(resp, vehicle_linear, vehicle_angle, Yaw,diff))
                                continue;

                        average_linear = (lastest_linear_ + vehicle_linear) / 2;
                        average_angle  = (lastest_angle_ + vehicle_angle) / 2;

                        vel_x = vehicle_linear * std::cos(average_angle);
                        vel_y = vehicle_linear * std::sin(average_angle);

                        odom_angle_ += average_angle * diff;
                        odom_linear_x_ += (average_linear * diff) * std::cos(odom_angle_);
                        odom_linear_y_ += (average_linear * diff) * std::sin(odom_angle_);

                        lastest_linear_ = vehicle_linear;                                        //保留上一次的速度值，与下一次接收的速度做均值计算
                        lastest_angle_ = vehicle_angle;                                          //保留上一次的速度值，与下一次接收的速度做均值计算

                        odom_data.pose.pose.position.x = odom_linear_x_;
                        odom_data.pose.pose.position.y = odom_linear_y_;
                        odom_data.pose.pose.position.z = 0;
                        odom_data.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_angle_);
                        odom_data.twist.twist.linear.x = vel_x;
                        odom_data.twist.twist.linear.y = vel_y;
                        odom_data.twist.twist.linear.z = 0;
                        odom_data.twist.twist.angular.z = vehicle_angle;
                        odom_data.header.stamp = ros::Time::now();

                        lastest_response_time_ = now;
                        transform_send.stamp_ = ros::Time::now();
                        tf::Vector3 v(odom_linear_x_, odom_linear_y_, 0);
                        transform_send.setOrigin(v);
                        transform_send.setRotation(tf::createQuaternionFromYaw(odom_angle_));
                        // tb_.sendTransform(transform_send);   //由于robot_pose_ekf功能包自带base_link-->odom的tf转换，注释掉避免多重接收。
                        odom_pub_.publish(odom_data);
                        // ROS_INFO("recv:  x %f,y %f,angle %f", odom_linear_x_, odom_linear_y_, (odom_angle_*(180.0/3.1415726)));
                }
                else
                {
                        ROS_WARN(" not recv encoder data");
                }
                r.sleep();
        }
}

/***************************************************************************************************************************************
 * 上位机(rasp)------->下位机(stm32)
 * tx[0]  tx[1]	 tx[2]	    tx[3]	    tx[4]	      tx[5]	      tx[6]	      tx[7]          tx[8]          tx[9]   tx[10]
 * oxff	  0xfe   tag	   motorA_H8   motorA_L8	 motorB_H8	 motorB_L8	 Servo_angle_H8	 Servo_angle_L8 方向	 校验
 * ************************************************************************************************************************************/
void MotorAdaptor::send_thread()
{
        uint8_t cmd[] = {0xff, 0xfe, 's', 0, 0, 0, 0, 0, 0, 0, 0};
        ros::Rate r(5);
        uint8_t direction_tag;
        uint16_t left_encoder_temp;
        uint16_t right_encoder_temp;
        uint16_t steer_angle_temp;

        while(ros::ok())
        {
                ros::Time now = ros::Time::now();
                diff_send = (now - lastest_send_time_).toSec();

                if((nav_queue_.size() != 0) && (recv_nav_tag==true))
                {
                        Vel send = nav_queue_.front();
                        direction_tag      = calcByteToWrite(Encoder_right, Encoder_left, Steer_angle, send);
                        nav_queue_.pop();

                        left_encoder_temp  = (uint16_t)Encoder_left;
                        right_encoder_temp = (uint16_t)Encoder_right;
                        steer_angle_temp = (uint16_t)Steer_angle;

                        cmd[3] =  (uint8_t)(left_encoder_temp>>8); // target_a 高八位
                        cmd[4] =  (uint8_t)(left_encoder_temp); // target_a 低八位
                        cmd[5] =  (uint8_t)(right_encoder_temp>>8); // target_b 高八位
                        cmd[6] =  (uint8_t)right_encoder_temp; // target_b 低八位
                        cmd[7] =  (uint8_t)(steer_angle_temp>>8); // 前轮转向角度 atan2(y,x) = 求 y/x 的反正切  高八位
                        cmd[8] =  (uint8_t)(steer_angle_temp); // 前轮转向角度 atan2(y,x) = 求 y/x 的反正切  低八位
                        cmd[9] =  direction_tag;             // 后轮方向控制位 若(0 0 0 0 0 1 0 0) 左轮向前 ；若(0 0 0 0 0 0 1 0) 右轮向前
                        cmd[10] =  check_data(cmd,3,10);
                        ser_car.write(cmd, 11);

                        recv_nav_tag = false;
                }
                else
                {
                        cmd[3] = 0x00;                       // target_a 高八位
                        cmd[4] = 0x00;                       // target_a 低八位
                        cmd[5] = 0x00;                       // target_b 高八位
                        cmd[6] = 0x00;                       // target_b 低八位
                        cmd[7] = (uint8_t)(steer_angle_temp>>8); // 前轮转向角度 5a
                        cmd[8] = (uint8_t)(steer_angle_temp); // 前轮转向角度 5a
                        cmd[9] = direction_tag;              // 后轮方向控制位 若0x0F:停车 ； 0x05 ：前进 ; 0x0A:倒车
                        cmd[10] = check_data(cmd,3,10);
                        ser_car.write(cmd, 11);
                }

                direction_tag      = 0x0F;
                steer_angle_temp   = 9000;
                left_encoder_temp  = 0.0;
                right_encoder_temp = 0.0;

                // ROS_INFO("nav send: %x, %x, %x, %x ,%x, %x, %x", cmd[3], cmd[4],cmd[5], cmd[6], cmd[7], cmd[8], cmd[9]);
                r.sleep();
        }
}

/***************************************************************************************************************************************************************************
 * 帧头  帧头  标志位	 tx[0]    tx[1]	 tx[2]  tx[3]	 tx[4]      tx[5]	                 tx[6]	              tx[7]	                tx[8]	              tx[9]            tx[10]	              tx[11]                    校验	    共16字节
 * 0xff	0xfe  ‘e’	  0	     0          0         0          0         motorA_H8	motorA_L8	motorB_H8	motorB_L8    方向     舵机角度_H8	  舵机角度_L8    异或校验
 * 检验位：只校验数据域rx[0~10],校验方式为：异或校验
 * 编码器方向位：
 *             tx[9] = 0x0F (A、B=0);
 *             tx[9] = 0x05 (A、B>0);
 *             tx[9] = 0x0A (A、B<0);
 *****************************************************************************************************************************************************************************/
bool MotorAdaptor::calcVelocity(uint8_t *rx, double &vehicle_linear, double &vehicle_angle, double &Yaw,double diff)
{
        if(rx[15]==check_data(rx,3,15))                                        //数据帧头正确且校验正确
        {
                uint16_t U_Velocity_A = 0;
                uint16_t U_Velocity_B = 0;
                uint16_t UServo_Ang = 0;
                int16_t Velocity_A = 0x00;
                int16_t Velocity_B = 0x00;

                double v_left;
                double v_right;
                double Servo_Ang;
                double ICR;

                UServo_Ang = ((UServo_Ang|rx[13])<<8)|rx[14]; // 下位机上传舵机角度的范围是[45,135]度，表示的是从左到右

                Servo_Ang = (double)( PI/2.0 - ( ( UServo_Ang / 100.0 ) * (PI/180.0) ) ); // 舵机角度转换成弧度 ，左偏右偏的角度的弧度值
                ICR = car_length_ / tan( Servo_Ang );                   // 向右偏转为负的，向左偏转为正的。 计算整个小车的转弯半径

                U_Velocity_A = ((U_Velocity_A|rx[8])<<8)|rx[9]; //存储在无符号16位中间变量里面
                U_Velocity_B = ((U_Velocity_B|rx[10])<<8)|rx[11];

                Velocity_A = (int16_t)U_Velocity_A;          //存储在有符号16位中间变量里面
                Velocity_B = (int16_t)U_Velocity_B;

                if(rx[12]== 0x05)      // 小车前进
                {
                        v_left = (double)(Velocity_A / 1000.0); //计算左右轮的速度 m/s
                        v_right = (double)(Velocity_B / 1000.0); //计算左右轮的速度 m/s
                }
                else if(rx[12] == 0x0A) // 小车后退
                {
                        v_left = -(double)(Velocity_A / 1000.0); //计算左右轮的速度 m/s
                        v_right = -(double)(Velocity_B / 1000.0); //计算左右轮的速度 m/s
                }
                else                 // 小车停止
                {
                        v_left = 0.0;
                        v_right= 0.0;
                }

                vehicle_linear = (v_left + v_right) / 2;     //计算mini小车后两轮中点处的线速度
                if(UServo_Ang == 90)
                        vehicle_angle = 0;
                else
                        vehicle_angle = vehicle_linear / ICR;  //计算mini小车后两轮中点处的角速度

                //  std::cout<<" Recv------> Left encoder: "<<Velocity_A <<" Right encoder: "<<Velocity_B<<" Servo_Ang : "<<Servo_Ang*(180/PI)<<std::endl;
                return 1;
        }
        else     //检验不正确
        {
                // ROS_INFO("Check error!");
                return 0;
        }
}

uint8_t MotorAdaptor::calcByteToWrite(int16_t &Encoder_right, int16_t &Encoder_left, int16_t &Steer_angle, Vel &v )
{
        double V_left = 0.0;
        double V_right = 0.0;
        double V_angle = 0.0;

        if(v.angle!=0)                                                  //********* 以一定的角度前进或倒退 ************
        {
                V_angle = std::atan2((v.angle*car_length_),fabs(v.linear)); // 前轮转向角度 atan2(y,x) = 求 y/x 的反正切
                ROS_INFO("Calculate Steer Angle: %lf",V_angle);
                if(V_angle>max_angle_)
                {
                        V_angle = max_angle_;
                        ROS_INFO("Steer angle out range");
                }
                else if(V_angle<-max_angle_)
                {
                        V_angle = -max_angle_;
                        ROS_INFO("Steer angle out range");
                }
                if(v.linear<0)
                {
                        V_left = v.linear*(1+tan(V_angle)/2.0);         // 小车倒车的时候，解算相反带角速度的运动学解算，是结合 teb 算法发现的.
                        V_right = v.linear*(1-tan(V_angle)/2.0);        // m/s
                        V_angle = -V_angle;
                }
                else
                {
                        V_left = v.linear*(1-tan(V_angle)/2.0);         // 带角速度的运动学解算
                        V_right = v.linear*(1+tan(V_angle)/2.0);        // m/s
                }
        }
        else                                                      // *********  角度为0 直行 *********
        {
                V_angle = 0.0;
                V_left = v.linear;
                V_right = v.linear;                               // m/s
        }


        Encoder_left = (int16_t)fabs(V_left * 1000);              // 此处除以 下位机 30ms读取一次 编码值，然后执行PI
        Encoder_right = (int16_t)fabs(V_right * 1000);             // 计算下位机30ms内应有的脉冲数
        Steer_angle = (int16_t)( 9000 - (V_angle * (180/PI))*100.0);

        // std::cout<<"Send----> Encoder_left: "<<Encoder_left<<"  Encoder_right:  "<<Encoder_right<<" V_angle: "<<V_angle<<std::endl;

        uint8_t motor_direction_tag = 0x00;
        if(v.linear<0)
                motor_direction_tag = 0x0A;                       // 倒车
        else if(v.linear>0)
                motor_direction_tag = 0x05;                       // 前进
        else
                motor_direction_tag = 0x0F;

        return motor_direction_tag;
}
void MotorAdaptor::execute()
{
        ros::spin();
}

void MotorAdaptor::nav_callback(const geometry_msgs::TwistConstPtr &msg)
{
        recv_nav_tag = true;
        Vel push;
        push.linear = msg->linear.x;
        push.angle = msg->angular.z;
        if(nav_queue_.size() > 5)                          //若队列满了，就会移除队头元素，然后
                nav_queue_.pop();
        nav_queue_.push(push);
}

/*************************************
* 函数名: 异或校验函数
* 参数:   需要校验的数组 ， 数组长度
* 返回值： 异或校验数值
*************************************/
uint8_t MotorAdaptor::check_data(uint8_t cmd[],int begin,int end)
{
        int i;
        uint8_t check=cmd[begin];

        for(i = begin+1; i<end; i++)
                check^=cmd[i];

        // ROS_INFO("check: 0x%X",check);
        return check;
}

int main(int argc, char *argv[])
{
        ros::init(argc, argv, "motor_adaptor_node");
        MotorAdaptor ma;
        ma.execute();
        return 0;
}
