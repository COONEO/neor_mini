#!/usr/bin/python
# -*- coding: UTF-8 -*-
# 上面两行不可省略，第一行是： 告诉操作系统执行这个脚本的时候，调用 /usr/bin 下的 python 解释器。第二行是：定义编码格式 "UTF-8-" 支持中文

from actionlib.action_client import GoalManager
import rospy 
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def send_goals_python():
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    #定义 5 个发送目标点的对象
    goal0 = MoveBaseGoal()
    goal1 = MoveBaseGoal()
    goal2 = MoveBaseGoal()
    goal3 = MoveBaseGoal()
    goal4 = MoveBaseGoal()
    
    # 初始化 5 个目标点在 map 坐标系下的坐标,数据来源于《采集的目标点.docx》
    goal0.target_pose.pose.position.x = 1.84500110149
    goal0.target_pose.pose.position.y = -0.883078575134
    goal0.target_pose.pose.orientation.z = -0.306595935327
    goal0.target_pose.pose.orientation.w = 0.951839761956
    
    goal1.target_pose.pose.position.x = 3.24358606339
    goal1.target_pose.pose.position.y = 0.977679371834
    goal1.target_pose.pose.orientation.z = 0.647871240469
    goal1.target_pose.pose.orientation.w = 0.761749864308
    
    goal2.target_pose.pose.position.x = 2.41693687439
    goal2.target_pose.pose.position.y = 1.64631867409
    goal2.target_pose.pose.orientation.z = 0.988149484601
    goal2.target_pose.pose.orientation.w = 0.153494612555
    
    goal3.target_pose.pose.position.x = -0.970185279846
    goal3.target_pose.pose.position.y = 0.453477025032
    goal3.target_pose.pose.orientation.z = 0.946238058267
    goal3.target_pose.pose.orientation.w = -0.323471076121

    goal4.target_pose.pose.position.x = 1.84500110149
    goal4.target_pose.pose.position.y = -0.883078575134
    goal4.target_pose.pose.orientation.z = -0.306595935327
    goal4.target_pose.pose.orientation.w = 0.951839761956
    
    goal_lists=[goal0, goal1, goal2, goal3, goal4]       # 采用 python 中的列表方式，替代实现C/C++ 中的数组概念 

    goal_number = 5     # total  goals:  5
    while(goal_number):      
        if(5 - goal_number ==0):
            goal_lists[5-goal_number].target_pose.header.frame_id = "map"
            goal_lists[5-goal_number].target_pose.header.stamp = rospy.Time.now()
            client.send_goal( goal_lists[5-goal_number])
            str_log = "Send NO. %s Goal !!!" %str(5-goal_number)
            rospy.loginfo(str_log)
        elif(5 - goal_number ==1):
            goal_lists[5-goal_number].target_pose.header.frame_id = "map"
            goal_lists[5-goal_number].target_pose.header.stamp = rospy.Time.now()
            client.send_goal( goal_lists[5-goal_number])
            str_log = "Send NO. %s Goal !!!" %str(5-goal_number)
            rospy.loginfo(str_log)
        elif(5 - goal_number ==2):
            goal_lists[5-goal_number].target_pose.header.frame_id = "map"
            goal_lists[5-goal_number].target_pose.header.stamp = rospy.Time.now()
            client.send_goal( goal_lists[5-goal_number])
            str_log = "Send NO. %s Goal !!!" %str(5-goal_number)
            rospy.loginfo(str_log)
        elif(5 - goal_number ==3):
            goal_lists[5-goal_number].target_pose.header.frame_id = "map"
            goal_lists[5-goal_number].target_pose.header.stamp = rospy.Time.now()
            client.send_goal( goal_lists[5-goal_number])
            str_log = "Send NO. %s Goal !!!" %str(5-goal_number)
            rospy.loginfo(str_log)
        elif(5 - goal_number ==4):
            goal_lists[5-goal_number].target_pose.header.frame_id = "map"
            goal_lists[5-goal_number].target_pose.header.stamp = rospy.Time.now()
            client.send_goal( goal_lists[5-goal_number])
            str_log = "Send NO. %s Goal !!!" %str(5-goal_number)
            rospy.loginfo(str_log)

        wait = client.wait_for_result(rospy.Duration.from_sec(30.0))     # 发送完毕目标点之后，根据action 的机制，等待反馈执行的状态，等待时长是：30 s.
        if not wait:
            str_log="The NO. %s Goal Planning Failed for some reasons && try again." %str(5-goal_number)
            rospy.loginfo(str_log)
            continue
        else:
            str_log="The NO. %s Goal achieved success && wait for 5 s !!!" %str(5-goal_number)
            rospy.loginfo(str_log)
            rospy.sleep(5.0)   # wait for 5 second.
            goal_number = goal_number - 1
    return "Mission Finished."

if __name__ == '__main__':
        rospy.init_node('send_goals_python',anonymous=True)    # python 语言方式下的　初始化 ROS 节点，
        result = send_goals_python()
        rospy.loginfo(result)