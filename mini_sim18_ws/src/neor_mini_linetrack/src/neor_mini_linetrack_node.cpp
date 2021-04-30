    #include<ros/ros.h>
    int main(int argc, char** argv)
    {
        ros::init(argc, argv, "neor_mini_linetrack_node"); 
        ros::NodeHandle nh;
        //....   Node function Lists
        //....
        ros::spin();    // Response queue used to trigger topic and service
        return 0;
    }