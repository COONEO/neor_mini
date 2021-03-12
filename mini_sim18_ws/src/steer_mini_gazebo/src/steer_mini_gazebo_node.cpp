    #include<ros/ros.h>
    int main(int argc, char** argv)
    {
        ros::init(argc, argv, "steer_mini_gazebo_node"); 
        ros::NodeHandle nh;
        //....   Node Function lists
        //....
        ros::spin();//  Response queue used to trigger topic and service
        return 0;
    }