#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mbplanner/MBRemapper.hpp>

namespace MBP{
    MBRemapper::MBRemapper():Node("remapper"){
        
        this->declare_parameter("remapper_ip", "/test_topic");
        this->declare_parameter("remapper_ur_op", "/ur_test");
        this->declare_parameter("remapper_tool_op", "/tool_test");
        this->declare_parameter("prefix_ur", "");
        this->declare_parameter("prefix_tool", "tool_");
        this->declare_parameter("prefix_ur_out", "a_ur_");
        this->declare_parameter("prefix_tool_out", "a_tool_");

        this->get_parameter("remapper_ip", remapper_ip_);
        this->get_parameter("remapper_ur_op", remapper_ur_op_);
        this->get_parameter("remapper_tool_op", remapper_tool_op_);
        this->get_parameter("prefix_ur", prefix_ur_);
        this->get_parameter("prefix_tool", prefix_tool_);
        this->get_parameter("prefix_ur_out", prefix_ur_out_);
        this->get_parameter("prefix_tool_out", prefix_tool_out_);
        
        tool_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(remapper_tool_op_, 10);
        ur_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(remapper_ur_op_, 10);
        ip_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            remapper_ip_,10,std::bind(&MBRemapper::ip_callback,this,std::placeholders::_1));
    }
    MBRemapper::~MBRemapper(){}

    std::pair<sensor_msgs::msg::JointState,sensor_msgs::msg::JointState>
    MBRemapper::remapping(const sensor_msgs::msg::JointState::SharedPtr msg){
        double yaw1, yaw2, yaw0, open1;
        sensor_msgs::msg::JointState ur_msg, tool_msg;
        std::vector<std::string> tool_set = {prefix_tool_+"roll",prefix_tool_+"yaw0",prefix_tool_+"open1",prefix_tool_+"pitch"};
        for (size_t i = 0; i < msg->name.size(); i++){
            bool isToolJoint = std::find(tool_set.begin(), tool_set.end(), msg->name[i]) != tool_set.end();
            if(isToolJoint){
                if (msg->name[i] == prefix_tool_+"yaw0"){
                    yaw0 = msg->position[i];
                    continue;
                }
                else if (msg->name[i] == prefix_tool_+"open1"){
                    open1 = msg->position[i];
                    continue;
                }
                std::string name = msg->name[i];
                // replace prefix
                if(name.find(prefix_tool_) != std::string::npos){
                    name.replace(0,prefix_tool_.size(),prefix_tool_out_);
                }
                tool_msg.name.push_back(name);
                tool_msg.position.push_back(msg->position[i]);
            }
            else{
                std::string name = msg->name[i];
                if(name.find(prefix_ur_) != std::string::npos){
                    name.replace(0,prefix_ur_.size(),prefix_ur_out_);
                }
                ur_msg.name.push_back(name);
                ur_msg.position.push_back(msg->position[i]);
            }
        }
        yaw1 = yaw0 + open1;
        yaw2 = open1 - yaw0;
        tool_msg.name.push_back(prefix_tool_out_ + "yaw1");
        tool_msg.position.push_back(yaw1);
        tool_msg.name.push_back(prefix_tool_out_ + "yaw2");
        tool_msg.position.push_back(yaw2);
        ur_msg.header = msg->header;
        tool_msg.header = msg->header;
        return std::make_pair(ur_msg,tool_msg);
    }

    void MBRemapper::ip_callback(const sensor_msgs::msg::JointState::SharedPtr msg){
        sensor_msgs::msg::JointState ur_msg, tool_msg;
        std::pair<sensor_msgs::msg::JointState,sensor_msgs::msg::JointState> remapped = remapping(msg);
        ur_msg = remapped.first;
        tool_msg = remapped.second;
        ur_pub_->publish(ur_msg);
        tool_pub_->publish(tool_msg);
    }
}
