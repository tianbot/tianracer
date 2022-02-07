/*
Copyright (c) 2017, Tianbot,
latest editor: Bo,Tian
All rights reserved. (Tianbot ROS Workshop)

This file is part of tianbot_racecar package.

tianbot_racecar is free software: you can redistribute it and/or modify
it under the terms of the GNU LESSER GENERAL PUBLIC LICENSE as published
by the Free Software Foundation, either version 3 of the License, or
any later version.

tianbot_racecar is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU LESSER GENERAL PUBLIC LICENSE for more details.

You should have received a copy of the GNU LESSER GENERAL PUBLIC LICENSE
along with tianbot_racecar.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
// #include <geometry_msgs/Twist.h>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
// #include <tf/transform_datatypes.h>
#include <tf2/transform_datatypes.h>
// #include <nav_msgs/Path.h>
#include <nav_msgs/msg/path.hpp>
// #include <nav_msgs/Odometry.h>
#include <nav_msgs/msg/odometry.hpp>
// #include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
// #include <visualization_msgs/Marker.h>
#include <visualization_msgs/msg/marker.hpp>
// add new hpp
#include <tf2_ros/buffer.h>
#include <memory>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define PI 3.14159265358979

/********************/
/* CLASS DEFINITION */
/********************/
class L1Controller
{
    public:
        explicit L1Controller(std::shared_ptr<rclcpp::Node> & nh);
        void initMarker();
        bool isForwardWayPt(const geometry_msgs::msg::Point& wayPt, const geometry_msgs::msg::Pose& carPose);
        bool isWayPtAwayFromLfwDist(const geometry_msgs::msg::Point& wayPt, const geometry_msgs::msg::Point& car_pos);
        double getYawFromPose(const geometry_msgs::msg::Pose& carPose);        
        double getEta(const geometry_msgs::msg::Pose& carPose);
        double getCar2GoalDist();
        double getL1Distance(const double& _Vcmd);
        double getSteeringAngle(double eta);
        double getGasInput(const float& current_v);
        geometry_msgs::msg::Point get_odom_car2WayPtVec(const geometry_msgs::msg::Pose& carPose);

    private:
        // ros::NodeHandle n_;
        std::shared_ptr<rclcpp::Node> nh_;
        // ros::Subscriber odom_sub, path_sub, goal_sub;
        // ros::Publisher pub_, marker_pub;
        // ros::Timer timer1, timer2;
        // tf::TransformListener tf_listener;

        // ros2 sub
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

        // ros2 pub
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

        // ros2 timer
        rclcpp::TimerBase::SharedPtr timer1_;
        rclcpp::TimerBase::SharedPtr timer2_;

        // ros2 tf2
        std::shared_ptr<tf2_ros::TransformListener>  tf_listener_;
        std::shared_ptr<tf2_ros::Buffer> buffer_; 


        visualization_msgs::msg::Marker points, line_strip, goal_circle;
        ackermann_msgs::msg::AckermannDrive ackermann_cmd;
        geometry_msgs::msg::Point odom_goal_pos;
        nav_msgs::msg::Odometry odom;
        nav_msgs::msg::Path map_path, odom_path;

        double L, Lfw, Lrv, Vcmd, lfw, lrv, steering, u, v;
        double gas_gain, base_angle, base_speed, angle_gain, goal_radius;
        int controller_freq;
        bool foundForwardPt, goal_received, goal_reached;

        void odomCB(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
        void pathCB(const nav_msgs::msg::Path::SharedPtr pathMsg);
        void goalCB(const geometry_msgs::msg::PoseStamped::SharedPtr goalMsg);
        // void goalReachingCB(const ros::TimerEvent&);
        // void controlLoopCB(const ros::TimerEvent&);

        // ros2 function
        void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr goalMsg);
        void goal_reaching_callback();
        void control_loop_callback();

}; // end of class


L1Controller::L1Controller(std::shared_ptr<rclcpp::Node> & nh) : nh_(nh)
{
    // //Private parameters handler
    // ros::NodeHandle pn("~");

    // //Car parameter
    // pn.param("L", L, 0.26);
    // pn.param("Lrv", Lrv, 10.0);
    // pn.param("Vcmd", Vcmd, 1.0);
    // pn.param("lfw", lfw, 0.13);
    // pn.param("lrv", lrv, 10.0);

    // //Controller parameter
    // pn.param("controller_freq", controller_freq, 20);
    // pn.param("angle_gain", angle_gain, -1.0);
    // pn.param("gas_gain", gas_gain, 1.0);
    // pn.param("base_speed", base_speed, 0.0);
    // pn.param("base_angle", base_angle, 0.0);

    // ros2 parameters with node;
    std::shared_ptr<rclcpp::Node> private_node = rclcpp::Node::make_shared("", "~");
    // declare car parameter
    private_node->declare_parameter("L", 0.26);
    private_node->declare_parameter("Lrv", 10.0);
    private_node->declare_parameter("Vcmd", 1.0);
    private_node->declare_parameter("lfw", 0.13);
    private_node->declare_parameter("lrv", 10.0);
    // get car parameter
    L = private_node->get_parameter("L").as_double();
    Lrv = private_node->get_parameter("Lrv").as_double();
    Vcmd = private_node->get_parameter("Vcmd").as_double();
    lfw = private_node->get_parameter("lfw").as_double();
    lrv = private_node->get_parameter("lrv").as_double();

    // declare controller parameter
    private_node->declare_parameter("controller_freq", 20);
    private_node->declare_parameter("angle_gain", -1.0);
    private_node->declare_parameter("gas_gain", 1.0);
    private_node->declare_parameter("base_speed", 0.0);
    private_node->declare_parameter("base_angle", 0.0);
    // get controller parameter
    controller_freq = private_node->get_parameter("controller_freq").as_int();
    angle_gain = private_node->get_parameter("angle_gain").as_double();
    gas_gain = private_node->get_parameter("gas_gain").as_double();
    base_speed = private_node->get_parameter("base_speed").as_double();
    base_angle = private_node->get_parameter("base_angle").as_double();


    // tf listener
    buffer_ = std::make_shared<tf2_ros::Buffer>(nh_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);  

    // //Publishers and Subscribers
    // odom_sub = n_.subscribe("/odometry/filtered", 1, &L1Controller::odomCB, this);
    // path_sub = n_.subscribe("/move_base_node/NavfnROS/plan", 1, &L1Controller::pathCB, this);
    // goal_sub = n_.subscribe("/move_base_simple/goal", 1, &L1Controller::goalCB, this);
    // marker_pub = n_.advertise<visualization_msgs::msg::Marker>("car_path", 10);
    // pub_ = n_.advertise<ackermann_msgs::msg::AckermannDrive>("tianracer/ackermann_cmd", 1);

    // ros2 pub and sub
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    odom_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", qos, std::bind(&L1Controller::odomCB, this, std::placeholders::_1));
    path_sub_ = nh_->create_subscription<nav_msgs::msg::Path>("/local_plan", qos, std::bind(&L1Controller::pathCB, this, std::placeholders::_1));
    goal_sub_ = nh_->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", qos, std::bind(&L1Controller::goal_callback, this, std::placeholders::_1));
    marker_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("car_path", 10);
    pub_ = nh_->create_publisher<ackermann_msgs::msg::AckermannDrive>("tianracer/ackermann_cmd", 1);

    // //Timer
    // timer1 = n_.createTimer(ros::Duration((1.0)/controller_freq), &L1Controller::controlLoopCB, this); // Duration(0.05) -> 20Hz
    // timer2 = n_.createTimer(ros::Duration((0.5)/controller_freq), &L1Controller::goalReachingCB, this); // Duration(0.05) -> 20Hz

    // ros2 timer
    timer1_ = nh_->create_wall_timer(std::chrono::duration<double>((1.0)/controller_freq), std::bind(&L1Controller::goal_reaching_callback, this));
    timer2_ = nh->create_wall_timer(std::chrono::duration<double>((0.5)/controller_freq), std::bind(&L1Controller::control_loop_callback, this));

    //Init variables
    Lfw = goal_radius = getL1Distance(Vcmd);
    foundForwardPt = false;
    goal_received = false;
    goal_reached = false;
    ackermann_cmd.speed = 0.0;
    ackermann_cmd.steering_angle = 0.0;

    // //Show info
    // ROS_INFO("[param] base_speed: %f", base_speed);
    // ROS_INFO("[param] base_angle: %f", base_angle);
    // ROS_INFO("[param] angle_gain: %f", angle_gain);
    // ROS_INFO("[param] Vcmd: %f", Vcmd);
    // ROS_INFO("[param] Lfw: %f", Lfw);

    // ros2 show info 
    RCLCPP_INFO(nh_->get_logger(), "[param] base_speed: %f", base_speed);
    RCLCPP_INFO(nh_->get_logger(), "[param] base_angle: %f", base_angle);
    RCLCPP_INFO(nh_->get_logger(), "[param] angle_gain: %f", angle_gain);
    RCLCPP_INFO(nh_->get_logger(), "[param] Vcmd: %f", Vcmd);
    RCLCPP_INFO(nh_->get_logger(), "[param] Lfw: %f", Lfw);

    //Visualization Marker Settings
    initMarker();
}



void L1Controller::initMarker()
{
    points.header.frame_id = line_strip.header.frame_id = goal_circle.header.frame_id = "odom";
    points.ns = line_strip.ns = goal_circle.ns = "Markers";
    points.action = line_strip.action = goal_circle.action = visualization_msgs::msg::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;
    points.id = 0;
    line_strip.id = 1;
    goal_circle.id = 2;

    points.type = visualization_msgs::msg::Marker::POINTS;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    goal_circle.type = visualization_msgs::msg::Marker::CYLINDER;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    //LINE_STRIP markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    goal_circle.scale.x = goal_radius;
    goal_circle.scale.y = goal_radius;
    goal_circle.scale.z = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    //goal_circle is yellow
    goal_circle.color.r = 1.0;
    goal_circle.color.g = 1.0;
    goal_circle.color.b = 0.0;
    goal_circle.color.a = 0.5;
}


void L1Controller::odomCB(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
{
    odom = *odomMsg;
}


void L1Controller::pathCB(const nav_msgs::msg::Path::SharedPtr pathMsg)
{
    map_path = *pathMsg;
}


// void L1Controller::goalCB(const geometry_msgs::msg::PoseStamped::SharedPtr goalMsg)
// {
//     try
//     {
//         geometry_msgs::msg::PoseStamped odom_goal;
//         tf_listener.transformPose("odom", ros::Time(0) , *goalMsg, "map" ,odom_goal);
//         odom_goal_pos = odom_goal.pose.position;
//         goal_received = true;
//         goal_reached = false;

//         /*Draw Goal on RVIZ*/
//         goal_circle.pose = odom_goal.pose;
//         marker_pub.publish(goal_circle);
//     }
//     catch(tf::TransformException &ex)
//     {
//         ROS_ERROR("%s",ex.what());
//         ros::Duration(1.0).sleep();
//     }
// }

void L1Controller::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr goalMsg) {
    try {
        geometry_msgs::msg::TransformStamped transStamped = buffer_->lookupTransform("odom", "map", tf2::TimePointZero);
        RCLCPP_INFO(nh_->get_logger(), "son1 related to son2: father %s, child %s offset(%.2f, %.2f, %.2f)",
            transStamped.header.frame_id.c_str(),
            transStamped.child_frame_id.c_str(),
            transStamped.transform.translation.x,
            transStamped.transform.translation.y,
            transStamped.transform.translation.z    
        );
        // convert pose to target frame
        geometry_msgs::msg::PoseStamped odom_goal;
        tf2::doTransform(*goalMsg, odom_goal, transStamped);

        odom_goal_pos = odom_goal.pose.position;
        goal_received = true;
        goal_reached = false;

        /*Draw Goal on RVIZ*/
        goal_circle.pose = odom_goal.pose;
        marker_pub_->publish(goal_circle);        
    } catch (tf2::TransformException &e) {
        RCLCPP_WARN(nh_->get_logger(), "warning %s", e.what());
    }    
}

double L1Controller::getYawFromPose(const geometry_msgs::msg::Pose& carPose)
{
    float x = carPose.orientation.x;
    float y = carPose.orientation.y;
    float z = carPose.orientation.z;
    float w = carPose.orientation.w;

    double tmp,yaw;
    tf2::Quaternion q(x,y,z,w);
    tf2::Matrix3x3 quaternion(q);
    quaternion.getRPY(tmp,tmp, yaw);

    return yaw;
}

bool L1Controller::isForwardWayPt(const geometry_msgs::msg::Point& wayPt, const geometry_msgs::msg::Pose& carPose)
{
    float car2wayPt_x = wayPt.x - carPose.position.x;
    float car2wayPt_y = wayPt.y - carPose.position.y;
    double car_theta = getYawFromPose(carPose);

    float car_car2wayPt_x = cos(car_theta)*car2wayPt_x + sin(car_theta)*car2wayPt_y;
    // float car_car2wayPt_y = -sin(car_theta)*car2wayPt_x + cos(car_theta)*car2wayPt_y;

    if(car_car2wayPt_x >0) /*is Forward WayPt*/
        return true;
    else
        return false;
}


bool L1Controller::isWayPtAwayFromLfwDist(const geometry_msgs::msg::Point& wayPt, const geometry_msgs::msg::Point& car_pos)
{
    double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double dist = sqrt(dx*dx + dy*dy);

    // if(dist < Lfw)
    //     return false;
    // else if(dist >= Lfw)
    //     return true;

    return dist < Lfw ? false : true;
}

geometry_msgs::msg::Point L1Controller::get_odom_car2WayPtVec(const geometry_msgs::msg::Pose& carPose)
{
    geometry_msgs::msg::Point carPose_pos = carPose.position;
    double carPose_yaw = getYawFromPose(carPose);
    geometry_msgs::msg::Point forwardPt;
    geometry_msgs::msg::Point odom_car2WayPtVec;
    foundForwardPt = false;

    if(!goal_reached){
        for(unsigned int i =0; i< map_path.poses.size(); i++)
        {
            geometry_msgs::msg::PoseStamped map_path_pose = map_path.poses[i];
            geometry_msgs::msg::PoseStamped odom_path_pose;

            try
            {
                // tf_listener.transformPose("odom", ros::Time(0) , map_path_pose, "map" ,odom_path_pose);
                geometry_msgs::msg::TransformStamped transStamped = buffer_->lookupTransform("odom", "map", tf2::TimePointZero);
                tf2::doTransform(map_path_pose, odom_path_pose, transStamped);

                geometry_msgs::msg::Point odom_path_wayPt = odom_path_pose.pose.position;
                bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt,carPose);

                if(_isForwardWayPt)
                {
                    bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt,carPose_pos);
                    if(_isWayPtAwayFromLfwDist)
                    {
                        forwardPt = odom_path_wayPt;
                        foundForwardPt = true;
                        break;
                    }
                }
            } catch (tf2::TransformException &e) {
                RCLCPP_ERROR(nh_->get_logger(), "warning %s", e.what());
            } 
            // catch(tf::TransformException &ex)
            // {
            //     ROS_ERROR("%s",ex.what());
            //     ros::Duration(1.0).sleep();
            // }
        }
        
    }
    else if(goal_reached)
    {
        forwardPt = odom_goal_pos;
        foundForwardPt = false;
    }

    /*Visualized Target Point on RVIZ*/
    /*Clear former target point Marker*/
    points.points.clear();
    line_strip.points.clear();
    
    if(foundForwardPt && !goal_reached)
    {
        points.points.push_back(carPose_pos);
        points.points.push_back(forwardPt);
        line_strip.points.push_back(carPose_pos);
        line_strip.points.push_back(forwardPt);
    }

    marker_pub_->publish(points);
    marker_pub_->publish(line_strip);
    
    odom_car2WayPtVec.x = cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + sin(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    odom_car2WayPtVec.y = -sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + cos(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    return odom_car2WayPtVec;
}


double L1Controller::getEta(const geometry_msgs::msg::Pose& carPose)
{
    geometry_msgs::msg::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);

    double eta = atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x);
    return eta;
}


double L1Controller::getCar2GoalDist()
{
    geometry_msgs::msg::Point car_pose = odom.pose.pose.position;
    double car2goal_x = odom_goal_pos.x - car_pose.x;
    double car2goal_y = odom_goal_pos.y - car_pose.y;

    double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);

    return dist2goal;
}

double L1Controller::getL1Distance(const double& _Vcmd)
{
    double L1 = 0;
    if(_Vcmd < 1.34)
        L1 = 3 / 3.0;
    else if(_Vcmd > 1.34 && _Vcmd < 5.36)
        L1 = _Vcmd*2.24 / 3.0;
    else
        L1 = 12 / 3.0;
    return L1;
}

double L1Controller::getSteeringAngle(double eta)
{
    double steering_angle = -atan2((L*sin(eta)),(Lfw/2+lfw*cos(eta)))*(180.0/PI);
    //ROS_INFO("Steering Angle = %.2f", steering_angle);
    return steering_angle;
}

double L1Controller::getGasInput(const float& current_v)
{
    double u = (Vcmd - current_v)*gas_gain;
    //ROS_INFO("velocity = %.2f\tu = %.2f",current_v, u);
    return u;
}


// void L1Controller::goalReachingCB(const ros::TimerEvent&)
// {

//     if(goal_received)
//     {
//         double car2goal_dist = getCar2GoalDist();
//         if(car2goal_dist < goal_radius)
//         {
//             goal_reached = true;
//             goal_received = false;
//             ROS_INFO("Goal Reached !");
//         }
//     }
// }

void L1Controller::goal_reaching_callback() {
    if(goal_received)
    {
        double car2goal_dist = getCar2GoalDist();
        if(car2goal_dist < goal_radius)
        {
            goal_reached = true;
            goal_received = false;
            RCLCPP_INFO(nh_->get_logger(),"Goal Reached !");
        }
    }    
}

// void L1Controller::controlLoopCB(const ros::TimerEvent&)
// {

//     geometry_msgs::msg::Pose carPose = odom.pose.pose;
//     geometry_msgs::msg::Twist carVel = odom.twist.twist;
//     ackermann_cmd.speed = 0;
//     ackermann_cmd.steering_angle = 0;


//     if(goal_received)
//     {
//         /*Estimate Steering Angle*/
//         double eta = getEta(carPose);  
//         if(foundForwardPt)
//         {
//             ackermann_cmd.steering_angle = getSteeringAngle(eta)*angle_gain;
//             /*Estimate Gas Input*/
//             if(!goal_reached)
//             {
//                 //double u = getGasInput(carVel.linear.x);
//                 //cmd_vel.linear.x = baseSpeed - u;
//                 ackermann_cmd.speed = base_speed;
//                 ROS_DEBUG("\nGas = %.2f\nSteering angle = %.2f",ackermann_cmd.speed,ackermann_cmd.steering_angle);
//             }
//         }
//     }
//     pub_.publish(ackermann_cmd);
// }

void L1Controller::control_loop_callback() {
    geometry_msgs::msg::Pose carPose = odom.pose.pose;
    // geometry_msgs::msg::Twist carVel = odom.twist.twist;
    ackermann_cmd.speed = 0;
    ackermann_cmd.steering_angle = 0;


    if(goal_received)
    {
        /*Estimate Steering Angle*/
        double eta = getEta(carPose);  
        if(foundForwardPt)
        {
            ackermann_cmd.steering_angle = getSteeringAngle(eta)*angle_gain;
            /*Estimate Gas Input*/
            if(!goal_reached)
            {
                //double u = getGasInput(carVel.linear.x);
                //cmd_vel.linear.x = baseSpeed - u;
                ackermann_cmd.speed = base_speed;
                RCLCPP_DEBUG(nh_->get_logger(), "\nGas = %.2f\nSteering angle = %.2f",ackermann_cmd.speed,ackermann_cmd.steering_angle);
            }
        }
    }
    pub_->publish(ackermann_cmd);
}

/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    // ros::init(argc, argv, "L1Controller_v2");
    // ros2 init
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("L1Controller_v2");
    L1Controller controller(node);
    rclcpp::spin(node);
    rclcpp::shutdown();

    // L1Controller controller;
    // ros::spin();
    return 0;
}
