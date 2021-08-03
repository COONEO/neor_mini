
#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"
 

#define  STATUS_A    0x01     // 中间和左侧 < min_range ,急右转
#define  STATUS_B    0x02     // 中间和右侧 < min_range ,急左转
#define  STATUS_C    0x03     // 两侧都 < min_range ，慢速直行
#define  STATUS_D    0x04     // 仅左侧 <= min_range 右转
#define  STATUS_E    0x05     // 仅右侧<= min_range 左转
#define  STATUS_F    0x06     // 仅中间 < min_range 判断左右间距，然后转弯 ,左侧距离 > 右侧距离 ，左转 
#define  STATUS_G   0x07     // 仅中间 < min_range 判断左右间距，然后转弯 ,左侧距离 < 右侧距离 ，右转
#define  STATUS_H   0x08     // 三个方向有任意一个方向距离 < worn_range
#define  STATUS_I     0x09      // 距离避障距离足够远
#define  STATUS_J    0x00     // 没有接收到超声波距离信息
 
 
//global variable
geometry_msgs::Twist twist_cmd;
ros::Publisher twist_pub;
 
const double warn_range = 0.23;        //前方两侧超声波最小的的停车距离 1.414 * ((车宽/2.0) - 超声波间距) = 0.26017 m( 23cmm - 36.8cm )；但是车身正对墙角时的情况除外。
const double F_L_min_range = 0.7;      //前左侧设定的避障距离 ,此处的设定依据是：车道为1.5m，前面两侧的超声波分别距离正中间的超声波的距离为4.5cm,所以
const double F_C_min_range = 0.5;      //前中侧设定的避障距离 
const double F_R_min_range = 0.7;      //前右侧设定的避障距离 
 
double range_array[3];                 //save three sonar value
uint8_t flag = 0x00;


double default_period_hz = 10;       //  hz
double default_linear_x = 0.2;         // (m/s)
double V_Angle = 0.0;                          // rad/s
double Steer_max_angle = 0.78539;      // 45度 前轮的机械结构决定
double Front_rear_distance = 0.3552;   // 前后轮的轴距：（整车长 - 前后车轮的两个半径）
double Min_turning_radius = 0.3552 ;   // ( Front_rear_distance / tan(Steer_max_angle))

/****************************************************************************************
 * 函数参数：超声波编号 、距离障碍物距离小的一侧的距离值
 * 
 * 函数来源：结合阿克曼转向底盘的运动学，根据避障的效果，需要做的是，距离越小，前轮转向的角度就越大，为了能够
 * 及时的避开障碍物，采用了开口向上的：一元二次函数来计算实时的角速度，解算之后，输出到前轮就是转向角度。
 * 
 * 小车目前是以固定的线速度default_linear_x前进的，不涉及后退功能。且已知：当超声波检测的距离满足以下条件的时候，开始避障。
 
 *  range = min_range 时，开始轻微转向避障，此时：V_Angle 约=0；
 *  range = warn_range 时，此时小车会停障，此时V_Angle 约 = 最大角速度值(default_linear_x / Min_turning_radius )
 * V_Angle = a*range*range + b*range + c
 * 其中：
 *      c = 0;
 *      a = default_linear_x / (warn_range(warn_range - min_range) * Min_turning_radius);
 *      b = -min_range;
 * 结论：
 *     当输入参数是左、右两个超声波采集的距离的时候：
 *         V_Angle = -5.2087*range*range -0.7*range；
 *     当输入参数是前面超声波采集的距离的时候：
 *         V_Angle = -0.9067*range*range -0.5*range；
 * 
 * 函数返回值： V_Angle 
 ***************************************************************************************/

double Calculate_V_angle(char sonar, double obstacles_range)
{
    double V_temp_Angle = 0.0;
    V_temp_Angle = -5.2087 * obstacles_range * obstacles_range -0.7 *obstacles_range;
    if(sonar == 'L')
        return V_temp_Angle;
    else if(sonar == 'R')
        return -V_temp_Angle;
    else
    {
        ROS_WARN("Calculate_V_angle Param input error");
        return 0;
    }
}
 
void Ultra_F_L_callback(const sensor_msgs::Range::ConstPtr& msg)
{
    // ROS_INFO(" Left range:[%f]", msg->range);
    range_array[0] = msg->range;
}
 
void Ultra_F_C_callback(const sensor_msgs::Range::ConstPtr& msg)
{
    // ROS_INFO(" Front range:[%f]", msg->range);
    range_array[1] = msg->range;
}
 
void Ultra_F_R_callback(const sensor_msgs::Range::ConstPtr& msg)
{
    // ROS_INFO(" Right range:[%f]", msg->range);
    range_array[2] = msg->range;
}
 
void publishTwistCmd(double linear_x, double angular_z)
{
    twist_cmd.linear.x = linear_x;
    twist_cmd.linear.y = 0.0;
    twist_cmd.linear.z = 0.0;
 
    twist_cmd.angular.x = 0.0;
    twist_cmd.angular.y = 0.0;
    twist_cmd.angular.z = angular_z;
 
    twist_pub.publish(twist_cmd);
}

void checkSonarRange(double sonar_F_L, double sonar_F_C, double sonar_F_R)
{
   unsigned char flag = 0;
   if( (sonar_F_L <= warn_range) || ( sonar_F_C<= warn_range) || ( sonar_F_R<= warn_range) )  //三个方向的某个方向距离 < worn_range 停车
   {
       flag = 0x08;
       publishTwistCmd(0.0 , 0.0);
   }
   else                                                                                       // 前面三个方向的距离都 > warn_range 继续行驶
   {
        if(((sonar_F_L <= F_L_min_range) && ( sonar_F_C <= F_C_min_range)) || (( sonar_F_C <= F_C_min_range) && ( sonar_F_R <= F_R_min_range)) || ((sonar_F_L <= F_L_min_range) && ( sonar_F_R <= F_R_min_range)) )    // 两侧距离 < min_range               
        {
            if((sonar_F_L <= F_L_min_range) && ( sonar_F_C <= F_C_min_range))                          // 中间和左侧 < min_range ,急右转
            {
                flag = 0x01;
                V_Angle = Calculate_V_angle('L', sonar_F_L);
            }
            else if(( sonar_F_C <= F_C_min_range) && ( sonar_F_R <= F_R_min_range))                    // 中间和右侧 < min_range ,急左转
            {
                flag = 0x02;
                V_Angle = Calculate_V_angle('R', sonar_F_R);
            }
            else                                                                                       // 两侧都 < min_range ，慢速直行
            {
                flag = 0x03;
            }
        }
        else  if((sonar_F_L <= F_L_min_range) || ( sonar_F_R <= F_R_min_range) || ( sonar_F_C <= F_C_min_range))        // 一侧距离 < min_range
        {
            if(sonar_F_L <= F_L_min_range)                                                                              // 仅左侧 <= min_range 缓慢右转
            {
                flag = 0x04;
                V_Angle = Calculate_V_angle('L', sonar_F_L);
            }
            else if( sonar_F_R <= F_R_min_range)                                                                        // 仅右侧<= min_range 缓慢左转                                         
            {
                flag = 0x05;
                V_Angle = Calculate_V_angle('R', sonar_F_R);
            }
            else                                                                                                        // 仅中间 < min_range 判断左右间距，然后转弯
            {
                if(sonar_F_L > sonar_F_R)                                                                               // 左侧距离 > 右侧距离 ，缓慢左转   
                {
                    flag = 0x06;
                    V_Angle = Calculate_V_angle('R', sonar_F_R);
                }
                else if(sonar_F_L < sonar_F_R)                                                                          // 左侧距离 < 右侧距离 ，缓慢右转 
                {
                    flag = 0x07;
                    V_Angle = Calculate_V_angle('L', sonar_F_L);
                } 
                else                                                                                                    // 仅中间 < min_range，且左侧距离 = 右侧距离 ，此时车身应该正对着一面墙 或 正对一个墙角驶过去，唯一能避开的方式是：试探性的向左或向右转弯。，此处特定为缓慢左转
                {
                    flag = 0x06;
                    V_Angle = -0.563 ;                                                                                   // 此时车身应该正对着一面墙 或 正对一个墙角驶过去,试探性地急速左转

                }
            }
        } 
        else              // 距离障碍物足够远
        {
            flag = 0x09;
        } 
    }

   //    ROS_INFO("CheckSonarRange get status:0x%x", flag);
   switch(flag)
   {
    case STATUS_A:                                  // turn right 0x04
        // "中间和左侧 < min_range ,急右转");
        ROS_WARN(" ---->");
        publishTwistCmd(default_linear_x, V_Angle);
        break;
 
    case STATUS_B:                                  
        // "中间和右侧 < min_range ,急左转");
        ROS_WARN(" <---- ");
        publishTwistCmd(default_linear_x, V_Angle);
        
        break;
    case STATUS_C:
        // "两侧都 < min_range ，慢速直行");
            ROS_INFO(" ^ ");
            ROS_INFO(" | ");
            ROS_INFO(" | ");
        publishTwistCmd(default_linear_x/2, 0);
        
        break;
 
    case STATUS_D:
        // "仅左侧 <= min_range 慢右转");
        ROS_WARN(" --->");
        publishTwistCmd(default_linear_x, V_Angle);
        
        break;
 
    case STATUS_E:
        // ("仅右侧<= min_range 慢左转"
        ROS_WARN(" <----");
        publishTwistCmd(default_linear_x, V_Angle);
        
        break;
 
    case STATUS_F:
        // "仅中间 < min_range 判断左右间距，然后转弯 ,左侧距离 > 右侧距离 ，慢左转 "
        ROS_WARN(" <----");
        publishTwistCmd(default_linear_x, V_Angle);
        
        break;
 
    case STATUS_G:
        // 仅中间 < min_range 判断左右间距，然后转弯 ,左侧距离 < 右侧距离 ，慢右转"
        ROS_WARN(" --->");
        publishTwistCmd(default_linear_x, V_Angle);
        
        break;
    
    case STATUS_H:
        // "三个方向有任意一个方向距离 < worn_range"
        ROS_WARN(" stop ");
        publishTwistCmd(0,0);
        
        break;
    
    case STATUS_J:
        ROS_WARN(" Didn't recv ultra range data, stop! ");
        publishTwistCmd(0,0);
        break;

    default:                                  
        //  STATUS_I 距离障碍物足够远
        ROS_INFO("Far from obstacles");
        publishTwistCmd(default_linear_x, 0);
        break;
   }
 
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ultra_avoid_obstacle_node");
    ros::NodeHandle handle;
    ros::Rate loop_rate = default_period_hz;
 
    ros::Subscriber sub_sonar0 = handle.subscribe("//Ultra_F_L", 100, Ultra_F_L_callback);
    ros::Subscriber sub_sonar1 = handle.subscribe("/Ultra_F_C", 100, Ultra_F_C_callback);
    ros::Subscriber sub_sonar2 = handle.subscribe("/Ultra_F_R", 100, Ultra_F_R_callback);
 
    twist_pub = handle.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 10);
 
    while(ros::ok())
    {
       checkSonarRange(range_array[0], range_array[1], range_array[2]);
       ros::spinOnce();
       loop_rate.sleep();
    }
 
    return 0;
}