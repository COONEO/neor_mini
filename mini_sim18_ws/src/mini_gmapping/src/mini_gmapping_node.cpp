    #include<ros/ros.h>
    int main(int argc, char** argv)
    {
        ros::init(argc, argv, "mini_gmapping_node"); 
        ros::NodeHandle nh;
        //.... 节点功能    Node function
        //....
        ros::spin();//用于触发topic、service的响应队列     Response queue used to trigger topic and service
        return 0;
    }