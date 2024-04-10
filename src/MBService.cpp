#include <rclcpp/rclcpp.hpp>
#include <mbplanner/MBPlanner.hpp>
#include <mb_msgs/srv/control_grip.hpp>
#include <mbplanner/MBService.hpp>

namespace MBP{
    
MBService::MBService(const rclcpp::Node::SharedPtr& node):node_(node){
    planner_ = std::make_shared<MBPlanner>(node_);
    control_grip_service_ = node_->create_service<mb_msgs::srv::ControlGrip>("control_grip",
        std::bind(&MBService::ControlGrip,this,std::placeholders::_1,std::placeholders::_2));
}
MBService::~MBService(){}

void MBService::initialize(){
    planner_->initialize();
}

void MBService::ControlGrip(const std::shared_ptr<mb_msgs::srv::ControlGrip::Request> request,
                            std::shared_ptr<mb_msgs::srv::ControlGrip::Response> response){
    if(request->open == true){
        response->success = planner_->openGrip();
    }
    else if(request->open == false){
        response->success = planner_->closeGrip();
    }
    else{
        response->success = false;
    }
    response->message = "Grip control success";
}

}