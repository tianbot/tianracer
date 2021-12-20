#ifndef __TIANBOARD_H__
#define __TIANBOARD_H__

#include <rclcpp/rclcpp.hpp>
#include "serial.h"
#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/pose2_d.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "boost/bind.hpp"
#include "boost/function.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "sensor_msgs/msg/imu.hpp"
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define DEFAULT_SERIAL_DEVICE "/dev/ttyUSB0"

class Tianboard: public rclcpp::Node {
public:
    Tianboard();
    ~Tianboard(){};
private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr uwb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr heart_timer_;
    rclcpp::TimerBase::SharedPtr communication_timer_;
    rclcpp::Node::SharedPtr nh_;
    Serial serial_;
    void ackermannCallback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg);
    void serialDataProc(uint8_t *data, unsigned int data_len);
    void tianboardDataProc(unsigned char *buf, int len);
    void heartCallback();
    void communicationErrorCallback();
    // new function
    void initSub();
    void initPub();
    void heartBeatTimer(const std::chrono::milliseconds timeout);
    void communicationTimer(const std::chrono::milliseconds timeout);
    void run();
};

#endif
