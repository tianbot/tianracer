#include <rclcpp/rclcpp.hpp>
#include "tianboard.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Tianboard>();

    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    return 0;
}
