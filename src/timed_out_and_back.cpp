#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "out_and_back_base_time");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    int rate = 50;
    ros::Rate loop(rate);

    double linear_speed = 0.2, goal_distance = 1.0;
    double linear_duration = goal_distance / linear_speed;
    double angular_speed = 1.0;
    double goal_angle = M_PI;
    double angular_duration = goal_angle / angular_speed;

    for (int i = 0 ; i < 2; i++)        // 一去一回两次
    {
        geometry_msgs::Twist move_cmd;

        move_cmd.linear.x = linear_speed;       // 线速度
        int ticks = int(linear_duration * rate);    // 发送次数

        for (int t = 0; t < ticks; t++)
        {
            pub.publish(move_cmd);  // 发送线速度
            loop.sleep();
        }

        // Stop move before the rotation
        move_cmd.linear.x = 0;
        pub.publish(move_cmd);
        ros::Duration(1.0).sleep();

        move_cmd.angular.z = angular_speed;

        ticks = (int)(goal_angle * rate);
        for (int t = 0; t < ticks; t++)
        {
            pub.publish(move_cmd);
            loop.sleep();
        }

        move_cmd.angular.z = 0;
        pub.publish(move_cmd);
        ros::Duration(1.0).sleep();

        pub.publish(move_cmd);
    }

    return 0;
}
