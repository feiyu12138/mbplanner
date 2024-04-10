#ifndef MBPPLANNER_HPP
#define MBPPLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <Eigen/Dense>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/planning_scene.hpp>

namespace MBP{
class MBPlanner
{
public:
    MBPlanner(const rclcpp::Node::SharedPtr& node);
    ~MBPlanner();
    void initialize();
    bool approachObject(const std::string& object_id);
    bool pickObject(const std::string& object_id);
    bool placeObject(const std::string& object_id, const std::string& target_id);
    bool openGrip();
    bool closeGrip(const std::string& object_id);
    bool liftObject(const std::string& object_id);
    bool moveHome();
    void setHome();
    // bool movePose(const geometry_msgs::msg::Pose& target_pose);
    void drawTitle(const std::string& title);
    void prompt(const std::string& message);
    void addObject(const std::string& object_id, const geometry_msgs::msg::Pose& pose, std::string shape = "box");
    

private:
    const rclcpp::Node::SharedPtr& node_;
    std::string target_topic_ = "/test_topic";
    std::string execute_ = "both";
    std::vector<double> home_;
    bool home_set_ = false;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Logger logger_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
    geometry_msgs::msg::Pose objPose_;
    Eigen::Vector3d approach_offset_ = Eigen::Vector3d(0.0,0.0,-0.01);
    Eigen::Vector3d pick_offset_ = Eigen::Vector3d(0.0,0.0,0.0);
    Eigen::Vector3d lift_offset_ = Eigen::Vector3d(0.0,0.0,-0.02);
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> grip_group_interface_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::unique_ptr<planning_scene_monitor::LockedPlanningSceneRW> scene_;
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> moveit_visual_tools_;
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;
    std::pair<bool,moveit::planning_interface::MoveGroupInterface::Plan> result_;
    moveit_msgs::msg::PlanningScene ps_;
    
    geometry_msgs::msg::Pose calculateApproachPose(geometry_msgs::msg::Pose Pose_t, Eigen::Vector3d offset);
    
    moveit_msgs::msg::CollisionObject createObject(geometry_msgs::msg::Pose pose, const std::string& object_id,std::string shape = "box");
    
    moveit_msgs::msg::AttachedCollisionObject createAttachedObject(moveit_msgs::msg::CollisionObject object);

    void attachObject(const std::string& object_id);
    
    void detachObject(const std::string& object_id);

    std::pair<bool,moveit::planning_interface::MoveGroupInterface::Plan> 
    plan(std::string group_name);
    
    moveit_msgs::msg::PlanningScene
    get_grasp_acm(bool collision);

    void updateACM(bool collision);

    std::vector<sensor_msgs::msg::JointState> planToJntStates(moveit::planning_interface::MoveGroupInterface::Plan plan,std::string group_name);

    void executePlan(moveit::planning_interface::MoveGroupInterface::Plan plan, std::string group_name);


};
}
#endif