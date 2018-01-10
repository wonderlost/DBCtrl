/*
该程序发布cmd_vel话题，以控制单兵机器人行走
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>


int main(int argc, char *argv[]) {
    /* code */
    ros::init(argc, argv, "DB_Robot_Ctrl");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::Rate loop(20);
    while (nh.ok())
    {
        geometry_msgs::Twist tw;
        tw.linear.x = 0.8;
        tw.angular.z = 0.2;

        // CarSpeed sp = speedTrans(tw.linear.x, tw.angular.z);
        // ROS_INFO("ls = %02x, rs = %02x", sp.left_speed, sp.right_speed);

        pub.publish(tw);
        loop.sleep();
    }
    return 0;
}
