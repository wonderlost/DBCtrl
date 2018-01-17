/*
    运行在车体端的节点，订阅/cmd_cmd主题，转换为CAN控制信号，以便直接控制车。
*/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include "advcan.h"

typedef struct tagCarSpeed
{
    int left_speed;
    int right_speed;
}CarSpeed;

const double wheel_dis = 0.398;     // 轮距(m)
const double wheel_radius = 0.0958;  // 轮子半径(m)

/*
线速度、角速度===》左轮速度、右轮速度
*/
CarSpeed speedTrans(double linearSpsed, double angularSpeed)
{
    double speedR = (linearSpsed + (wheel_dis * angularSpeed)/2) / wheel_radius;
    double speedL = (linearSpsed - (wheel_dis * angularSpeed)/2) / wheel_radius;

    speedR = speedR > 45 ? 45 : speedR;     // 限速
    speedR = speedR < -45 ? -45 : speedR;

    speedL = speedL > 45 ? 45 : speedL;
    speedL = speedL < -45 ? -45 : speedL;

    CarSpeed cs;
    if (speedR >= 0)
        cs.right_speed = ((int)speedR) | 0x80;
    else
        cs.right_speed = (int)(fabs(speedR));

    if (speedL >= 0)
        cs.left_speed = ((int)speedL) | 0x80;
    else
        cs.left_speed = (int)(fabs(speedL));

    return cs;
}

void callback(const geometry_msgs::Twist &tw)
{
    geometry_msgs::Twist tmp = tw;
    // 速度转换，转换为左右轮速度
    CarSpeed speed = speedTrans(tmp.linear.x, tmp.angular.z);

    // 将转换的左右轮速度通过CAN总线发出去
    CanMsg msg;
    msg.ID = 0x01;      // 自主控制ID
    msg.LEN = 5;        // 发送5个字节的数据（实际只使用2个）
    msg.DATA[0] = speed.left_speed;
    msg.DATA[1] = speed.right_speed;
    int ret = WriteCAN(msg);        // 向CAN上写入数据
}

int main(int argc, char  *argv[]) {
    ros::init(argc, argv, "move_sub");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/cmd_vel", 1, callback);

    ros::spin();


    return 0;
}
