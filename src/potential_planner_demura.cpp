#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <vector>

// RoombaRobotクラス
class RoombaRobot {
public:
    RoombaRobot();
    void main();

private:
    void setVel(double lv, double av);
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    std::vector<double> ranges;
    double angle_min;
    double angle_max;
    double angle_increment;
    double range_min;
    double range_max;

    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    ros::Publisher vel_pub;

    // ターゲット位置
    double target_x = 5.0;
    double target_y = 5.0;

    std::vector<double> attractiveForce(double position_x, double position_y);
    std::vector<double> repulsiveForce(double position_x, double position_y);
};

RoombaRobot::RoombaRobot() {
    odom_sub = nh.subscribe("/create1/rplidar/scan", 10, &RoombaRobot::lidarCallback, this);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/create1/cmd_vel", 10);
    setVel(0, 0);
}

void RoombaRobot::setVel(double lv, double av) {
    geometry_msgs::Twist vel;
    vel.linear.x = lv;  // 並進速度
    vel.angular.z = av; // 角速度
    vel_pub.publish(vel);
    ROS_INFO("Velocity: Linear=%f Angular=%f", vel.linear.x, vel.angular.z);
}

void RoombaRobot::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    //ranges = msg->ranges;
    angle_min = msg->angle_min;
    angle_max = msg->angle_max;
    angle_increment = msg->angle_increment;
    range_min = msg->range_min;
    range_max = msg->range_max;
}

std::vector<double> RoombaRobot::attractiveForce(double position_x, double position_y) {
    double K_att = 1.0;  // アトラクティブポテンシャルのスケーリング係数
    std::vector<double> force(2);
    force[0] = K_att * (target_x - position_x);
    force[1] = K_att * (target_y - position_y);
    return force;
}

std::vector<double> RoombaRobot::repulsiveForce(double position_x, double position_y) {
    double K_rep = 100.0;  // リパルシブポテンシャルのスケーリング係数
    std::vector<double> force(2, 0.0);
    for (size_t i = 0; i < ranges.size(); ++i) {
        double distance = ranges[i];
        if (range_min < distance && distance < range_max) {
            double angle_rad = angle_min + i * angle_increment;
            double obstacle_x = position_x + distance * cos(angle_rad);
            double obstacle_y = position_y + distance * sin(angle_rad);
            force[0] += K_rep * (1.0 / distance - 1.0 / range_max) * (position_x - obstacle_x) / (distance * distance);
            force[1] += K_rep * (1.0 / distance - 1.0 / range_max) * (position_y - obstacle_y) / (distance * distance);
        }
    }
    return force;
}

void RoombaRobot::main() {
    ros::Rate rate(10);
    double position_x = 0.0;
    double position_y = 0.0;

    while (ros::ok()) {
        if (!ranges.empty()) {
            std::vector<double> f_att = attractiveForce(position_x, position_y);
            std::vector<double> f_rep = repulsiveForce(position_x, position_y);
            std::vector<double> f_total(2);
            f_total[0] = f_att[0] + f_rep[0];
            f_total[1] = f_att[1] + f_rep[1];

            // double linear_vel = std::clamp(f_total[0], -0.5, 0.5);
            // double angular_vel = std::clamp(f_total[1], -1.0, 1.0);
            double linear_vel = f_total[0];
            double angular_vel = f_total[1];
            setVel(linear_vel, angular_vel);

            ROS_INFO("Number of Rays=%zu", ranges.size());
            ROS_INFO("Angle [rad] min=%f max=%f", angle_min, angle_max);
            //ROS_INFO("Angle [deg] increment=%.3f", std::degrees(angle_increment));
            ROS_INFO("Range [m] min=%.3f max=%.3f", range_min, range_max);
            ROS_INFO("Position: (%f, %f)", position_x, position_y);
        }

        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "potential_planner");
    RoombaRobot robot;
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        robot.main();
        rate.sleep();
    }
    return 0;
}