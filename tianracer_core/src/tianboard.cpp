#include "tianboard.hpp"
#include "protocol.h"
#include <vector>

using namespace std;

void Tianboard::serialDataProc(uint8_t *data, unsigned int data_len)
{
    static uint8_t state = 0;
    uint8_t *p = data;
    static vector<uint8_t> recv_msg;
    static uint32_t len;
    uint32_t j;
    while (data_len != 0)
    {
        switch (state)
        {
        case 0:
            if (*p == (PROTOCOL_HEAD & 0xFF))
            {
                recv_msg.clear();
                recv_msg.push_back(PROTOCOL_HEAD & 0xFF);
                state = 1;
            }
            p++;
            data_len--;
            break;

        case 1:
            if (*p == ((PROTOCOL_HEAD >> 8) & 0xFF))
            {
                recv_msg.push_back(((PROTOCOL_HEAD >> 8) & 0xFF));
                p++;
                data_len--;
                state = 2;
            }
            else
            {
                state = 0;
            }
            break;

        case 2: // len
            recv_msg.push_back(*p);
            len = *p;
            p++;
            data_len--;
            state = 3;
            break;

        case 3: // len
            recv_msg.push_back(*p);
            len += (*p) * 256;
            if (len > 1024 * 10)
            {
                state = 0;
                break;
            }
            p++;
            data_len--;
            state = 4;
            break;

        case 4: // pack_type
            recv_msg.push_back(*p);
            p++;
            data_len--;
            len--;
            state = 5;
            break;

        case 5: // pack_type
            recv_msg.push_back(*p);
            p++;
            data_len--;
            len--;
            state = 6;
            break;

        case 6: //
            if (len--)
            {
                recv_msg.push_back(*p);
                p++;
                data_len--;
            }
            else
            {
                state = 7;
            }
            break;

        case 7:
        {
            int i;
            uint8_t bcc = 0;
            recv_msg.push_back(*p);
            p++;
            data_len--;
            state = 0;

            for (i = 4; i < recv_msg.size(); i++)
            {
                bcc ^= recv_msg[i];
            }

            if (bcc == 0)
            {
                tianboardDataProc(&recv_msg[0], recv_msg.size()); // process recv msg
            }
            else
            {
                RCLCPP_INFO(get_logger(), "BCC error");
            }
            state = 0;
        }
        break;

        default:
            state = 0;
            break;
        }
    }
}

void Tianboard::tianboardDataProc(unsigned char *buf, int len)
{
    struct protocol_pack *p = (struct protocol_pack *)buf;
    switch (p->pack_type)
    {
    case PACK_TYPE_ODOM_RESPONSE:
        if (sizeof(struct odom) == p->len - 2)
        {
            nav_msgs::msg::Odometry odom_msg;
            struct odom *pOdom = (struct odom *)(p->data);
            rclcpp::Time current_time = this->now();
            odom_msg.header.stamp = current_time;
            odom_msg.header.frame_id = "odom";

            odom_msg.pose.pose.position.x = pOdom->pose.point.x;
            odom_msg.pose.pose.position.y = pOdom->pose.point.y;
            odom_msg.pose.pose.position.z = pOdom->pose.point.z;
            tf2::Quaternion q_odom;
            q_odom.setRPY(0.0, 0.0, pOdom->pose.yaw);
            geometry_msgs::msg::Quaternion q = tf2::toMsg(q_odom);
            odom_msg.pose.pose.orientation = q;
            //set the velocity
            odom_msg.child_frame_id = "base_footprint";
            odom_msg.twist.twist.linear.x = pOdom->twist.linear.x;
            odom_msg.twist.twist.linear.y = pOdom->twist.linear.y;
            odom_msg.twist.twist.linear.z = pOdom->twist.linear.z;
            odom_msg.twist.twist.angular.x = pOdom->twist.angular.x;
            odom_msg.twist.twist.angular.y = pOdom->twist.angular.y;
            odom_msg.twist.twist.angular.z = pOdom->twist.angular.z;
            //publish the message
            odom_pub_->publish(odom_msg);
        }
        break;

    case PACK_TYPE_UWB_RESPONSE:
        if (sizeof(struct uwb) == p->len - 2)
        {
            geometry_msgs::msg::Pose2D pose2d_msg;
            struct uwb *pUwb = (struct uwb *)(p->data);
            pose2d_msg.x = pUwb->x_m;
            pose2d_msg.y = pUwb->y_m;
            pose2d_msg.theta = pUwb->yaw;
            uwb_pub_->publish(pose2d_msg);
        }
        break;

    case PACK_TYPE_HEART_BEAT_RESPONSE:
        break;

    case PACK_TYPE_IMU_REPONSE:
        if (sizeof(struct imu_feedback) == p->len - 2)
        {
            sensor_msgs::msg::Imu imu_msg;
            struct imu_feedback *pImu = (struct imu_feedback *)(p->data);

            rclcpp::Time current_time = this->now();
            imu_msg.header.stamp = current_time;
            imu_msg.header.frame_id = "imu_link";
            imu_msg.orientation.x = pImu->quat.x;
            imu_msg.orientation.y = pImu->quat.y;
            imu_msg.orientation.z = pImu->quat.z;
            imu_msg.orientation.w = pImu->quat.w;
            imu_msg.angular_velocity.x = pImu->angular_vel.x;
            imu_msg.angular_velocity.y = pImu->angular_vel.y;
            imu_msg.angular_velocity.z = pImu->angular_vel.z;
            imu_msg.linear_acceleration.x = pImu->linear_acc.x;
            imu_msg.linear_acceleration.y = pImu->linear_acc.y;
            imu_msg.linear_acceleration.z = pImu->linear_acc.z;
            imu_pub_->publish(imu_msg);
        }
        break;

    default:
        break;
    }
    communication_timer_.reset();
}

void Tianboard::ackermannCallback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg)
{
    uint16_t len;
    vector<uint8_t> buf;

    uint8_t bcc = 0;
    struct ackermann_cmd ackermann_cmd;
    int i;
    uint8_t *out = (uint8_t *)&ackermann_cmd;
    ackermann_cmd.steering_angle = msg->steering_angle;
    ackermann_cmd.speed = msg->speed;

    buf.push_back(PROTOCOL_HEAD & 0xFF);
    buf.push_back((PROTOCOL_HEAD >> 8) & 0xFF);

    len = sizeof(struct ackermann_cmd) + 2;

    buf.push_back(len & 0xFF);
    buf.push_back((len >> 8) & 0xFF);

    buf.push_back(PACK_TYPE_ACKERMANN_CMD & 0xFF);
    buf.push_back((PACK_TYPE_ACKERMANN_CMD >> 8) & 0xFF);

    for (i = 0; i < sizeof(struct ackermann_cmd); i++)
    {
        buf.push_back(out[i]);
    }

    for (i = 4; i < buf.size(); i++)
    {
        bcc ^= buf[i];
    }

    buf.push_back(bcc);

    serial_.send(&buf[0], buf.size());

    heart_timer_.reset();

}

void Tianboard::initPub()
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("tianracer/odom", qos);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("tianracer/imu", qos);
    uwb_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("tianracer/uwb", qos);
}

void Tianboard::initSub()
{
    ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
        "tianracer/ackermann_cmd", \
         rclcpp::SensorDataQoS(), \
         std::bind( \
             &Tianboard::ackermannCallback, \
             this, \
             std::placeholders::_1));
}

void Tianboard::heartBeatTimer(const std::chrono::milliseconds timeout) 
{
    heart_timer_ = this->create_wall_timer(timeout,
        [this]() -> void
        {
            uint16_t len;
            vector<uint8_t> buf;
            uint16_t dummy = 0;
            uint8_t bcc = 0;
            uint8_t *out = (uint8_t *)&dummy;
            buf.push_back(PROTOCOL_HEAD & 0xFF);
            buf.push_back((PROTOCOL_HEAD >> 8) & 0xFF);

            len = 2 + sizeof(dummy);

            buf.push_back(len & 0xFF);
            buf.push_back((len >> 8) & 0xFF);

            buf.push_back(PACK_TYPE_HEART_BEAT & 0xFF);
            buf.push_back((PACK_TYPE_HEART_BEAT >> 8) & 0xFF);

            for (uint16_t i = 0; i < sizeof(dummy); i++)
            {
                buf.push_back(out[i]);
            }

            for (uint16_t i = 4; i < buf.size(); i++)
            {
                bcc ^= buf[i];
            }

            buf.push_back(bcc);

            serial_.send(&buf[0], buf.size());
        }
    );
}

void Tianboard::communicationTimer(const std::chrono::milliseconds timeout)
{
    communication_timer_ = this->create_wall_timer(timeout,
        [this]() -> void {
            RCLCPP_ERROR(this->get_logger(), "Communication with base error");
        }
    );
}

void Tianboard::run()
{
    RCLCPP_INFO(this->get_logger(), "Run in Tianboard");

    heartBeatTimer(std::chrono::milliseconds(200));

    communicationTimer(std::chrono::milliseconds(200));
}

Tianboard::Tianboard(): Node("tianracer")
{
    this->declare_parameter<std::string>("serial_port", DEFAULT_SERIAL_DEVICE);

    initSub();
    
    initPub();

    run();

    std::string param_serial_port;
    this->get_parameter("serial_port", param_serial_port);
    if (serial_.open(param_serial_port.c_str(), 115200, 0, 8, 1, 'N',
                     boost::bind(&Tianboard::serialDataProc, this, _1, _2)) != true)
    {
        exit(-1);
    }
}
