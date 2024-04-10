#ifndef MBREMAPPERR_HPP
#define MBREMAPPERR_HPP
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
namespace MBP{
class MBRemapper : public rclcpp::Node {
public:
    MBRemapper();
    ~MBRemapper();
private:
    std::string remapper_ip_ = "/test_topic";
    std::string remapper_ur_op_ = "/ur_test";
    std::string remapper_tool_op_ = "/tool_test";
    std::string prefix_ur_ = ""; 
    std::string prefix_tool_ = "tool_"; 
    std::string prefix_ur_out_= "ur_"; 
    std::string prefix_tool_out_ = "a_tool_";
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr tool_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ur_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ip_sub_;

    std::pair<sensor_msgs::msg::JointState,sensor_msgs::msg::JointState> 
    remapping(const sensor_msgs::msg::JointState::SharedPtr msg);

    void ip_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    


};


}

#endif