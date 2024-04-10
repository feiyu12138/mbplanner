#ifndef MBSERVICE_HPP
#define MBSERVICE_HPP
#include <rclcpp/rclcpp.hpp>
#include <mbplanner/MBPlanner.hpp>
#include <mb_msgs/srv/control_grip.hpp>

namespace MBP{
class MBService{
public:
    MBService(const rclcpp::Node::SharedPtr& node);
    ~MBService();
    void initialize();
private:
    const rclcpp::Node::SharedPtr& node_;
    std::shared_ptr<MBPlanner> planner_;
    rclcpp::Service<mb_msgs::srv::ControlGrip>::SharedPtr control_grip_service_;
    void ControlGrip(const std::shared_ptr<mb_msgs::srv::ControlGrip::Request> request,
                     std::shared_ptr<mb_msgs::srv::ControlGrip::Response> response);
};
};

#endif