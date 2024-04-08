#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <Eigen/Dense>
#include <cmath>
#include <geometric_shapes/shape_operations.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <mbplanner/MBPlanner.hpp>

namespace MBP{

MBPlanner::MBPlanner(const rclcpp::Node::SharedPtr& node):node_(node),logger_(node_->get_logger())
{
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_,"arm_dvrk");
    grip_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_,"grip");
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
}
    
void MBPlanner::initialize(){
    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_,"robot_description");
    psm_->startSceneMonitor("/monitored_planning_scene");
    scene_ = std::make_unique<planning_scene_monitor::LockedPlanningSceneRW>(psm_);
    move_group_interface_->allowReplanning(true);
    move_group_interface_->setMaxVelocityScalingFactor(0.1);
    move_group_interface_->setMaxAccelerationScalingFactor(0.1);
    move_group_interface_->setPlanningPipelineId("ompl");
    move_group_interface_->setPlanningTime(15.0);
    grip_group_interface_->allowReplanning(true);
    grip_group_interface_->setMaxVelocityScalingFactor(0.1);
    grip_group_interface_->setMaxAccelerationScalingFactor(0.1);
    grip_group_interface_->setPlanningPipelineId("ompl");
    grip_group_interface_->setPlanningTime(15.0);
    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
        node_,"base_link",rviz_visual_tools::RVIZ_MARKER_TOPIC,
        move_group_interface_->getRobotModel()};
    moveit_visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(moveit_visual_tools);
    moveit_visual_tools_->deleteAllMarkers();
    moveit_visual_tools_->loadRemoteControl();
    
    
}
void MBPlanner::drawTitle(const std::string& title){
    auto msg = Eigen::Isometry3d::Identity();
    msg.translation().z() = 1.0; 
    moveit_visual_tools_->publishText(msg,
    title,
    rviz_visual_tools::WHITE,
    rviz_visual_tools::XLARGE);
}
void MBPlanner::prompt(const std::string& text){
    moveit_visual_tools_->prompt(text);
}
moveit_msgs::msg::CollisionObject
MBPlanner::createObject(
    geometry_msgs::msg::Pose pose, 
    const std::string& object_id,
    std::string shape
    )
    {
    moveit_msgs::msg::CollisionObject object;
    shape_msgs::msg::SolidPrimitive primitive;
    double xy;
    if (shape == "box"){
        xy = 0.005;
    }
    else{
        xy = 0.05;
    }
    object.id = object_id;
    object.header.frame_id = move_group_interface_->getPlanningFrame();
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = xy;
    primitive.dimensions[primitive.BOX_Y] = xy;
    primitive.dimensions[primitive.BOX_Z] = 0.005;
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose);
    object.operation = object.ADD;
    return object;
    }

void MBPlanner::addObject(
    const std::string& object_id, 
    const geometry_msgs::msg::Pose& pose,
    std::string shape)
    {
    moveit_msgs::msg::CollisionObject object;
    object = createObject(pose,object_id,shape);
    planning_scene_interface_->applyCollisionObject(object);
    }

moveit_msgs::msg::AttachedCollisionObject
MBPlanner::createAttachedObject(moveit_msgs::msg::CollisionObject object){
    moveit_msgs::msg::AttachedCollisionObject AttachedObject;
    AttachedObject.link_name = "tool_grip_link";
    AttachedObject.object = object;
    AttachedObject.touch_links = 
    {
        "tool_grip_link",
        "tool_wrist",
        "tool_jaw1",
        "tool_jaw2"
    };
    AttachedObject.object.operation = AttachedObject.object.ADD;
    return AttachedObject;
}

std::pair<bool,moveit::planning_interface::MoveGroupInterface::Plan> 
MBPlanner::plan(std::string group_name){
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (group_name == "arm_dvrk"){
        auto const ok = static_cast<bool>(move_group_interface_->plan(plan));
        return std::make_pair(ok,plan);
    }
    else{
        auto const ok = static_cast<bool>(grip_group_interface_->plan(plan));
        return std::make_pair(ok,plan);
    }
}

bool MBPlanner::openGrip(){
    drawTitle("Open Grip");
    moveit_visual_tools_->trigger();
    grip_group_interface_->setNamedTarget("open");
    result_ = plan("grip");
    if (result_.first){
        prompt("Press 'Next' to open gripper");
        drawTitle("Open Grip Execution");
        moveit_visual_tools_->trigger();
        grip_group_interface_->execute(result_.second);
        return true;
    }
    else{
        drawTitle("Open Grip Failed");
        moveit_visual_tools_->trigger();
        moveit_visual_tools_->deleteAllMarkers();
        RCLCPP_ERROR(logger_,"Failed to plan open grip");
        return false;
    }
}

moveit_msgs::msg::PlanningScene
MBPlanner::get_grasp_acm(bool collision)
{
    auto& planning_scene = *(scene_->operator->());
    auto& acm = planning_scene.getAllowedCollisionMatrixNonConst();
    
    moveit_msgs::msg::PlanningScene ps;
    moveit_msgs::msg::AllowedCollisionMatrix updated_acm;
    ps.robot_state.is_diff = true;
    ps.is_diff = true;
    acm.setEntry("goal_box","tool_grip_link", collision);
    acm.setEntry("goal_box","tool_wrist", collision);
    acm.setEntry("goal_box","tool_jaw1", collision);
    acm.setEntry("goal_box","tool_jaw2", collision);
    acm.getMessage(updated_acm);
    ps.allowed_collision_matrix = updated_acm;
    return ps;
}

void MBPlanner::updateACM(bool collision){
    ps_ = get_grasp_acm(collision);
    planning_scene_interface_->applyPlanningScene(ps_);
}

geometry_msgs::msg::Pose
MBPlanner::calculateApproachPose(
    geometry_msgs::msg::Pose Pose_t, 
    Eigen::Vector3d offset)
{
    Eigen::Matrix3d R_t = Eigen::Quaterniond(
        Pose_t.orientation.w,
        Pose_t.orientation.x,
        Pose_t.orientation.y,
        Pose_t.orientation.z
    ).toRotationMatrix();

    Eigen::Vector3d P_t(
        Pose_t.position.x, 
        Pose_t.position.y, 
        Pose_t.position.z);

    Eigen::Matrix4d F_t;
    F_t.setIdentity();
    F_t.block<3,3>(0,0) = R_t;
    F_t.block<3,1>(0,3) = P_t;

    Eigen::Matrix4d F_te;
    F_te.setIdentity(); 
    F_te.block<3,1>(0,3) = offset;

    Eigen::Matrix4d F_e = F_t * F_te;

    geometry_msgs::msg::Pose Pose_e;
    Eigen::Quaterniond q(Eigen::Matrix3d(F_e.block<3,3>(0,0)));
    Pose_e.orientation.w = q.w();
    Pose_e.orientation.x = q.x();
    Pose_e.orientation.y = q.y();
    Pose_e.orientation.z = q.z();
    Pose_e.position.x = F_e(0,3);
    Pose_e.position.y = F_e(1,3);
    Pose_e.position.z = F_e(2,3);
    return Pose_e;
  }

bool MBPlanner::approachObject(const std::string& object_id){
    geometry_msgs::msg::Pose objectPose;
    geometry_msgs::msg::Pose approachPose;
    prompt("Press 'Next' to plan to approach object");
    updateACM(false);
    drawTitle("Approach Object");
    moveit_visual_tools_->trigger();
    objectPose = planning_scene_interface_
        ->getObjectPoses({object_id})[object_id];
    approachPose = calculateApproachPose(objectPose,approach_offset_);
    move_group_interface_->setJointValueTarget(approachPose);
    result_ = plan("arm_dvrk");
    if (result_.first){
        prompt("Press 'Next' to approach object");
        drawTitle("Approach Object Execution");
        moveit_visual_tools_->trigger();
        move_group_interface_->execute(result_.second);
        return true;
    }
    else{
        drawTitle("Approach Object Failed");
        moveit_visual_tools_->trigger();
        moveit_visual_tools_->deleteAllMarkers();
        RCLCPP_ERROR(logger_,"Failed to plan approach object");
        return false;
    }
}

bool MBPlanner::pickObject(const std::string& object_id){
    geometry_msgs::msg::Pose objectPose;
    geometry_msgs::msg::Pose pickPose;
    updateACM(true);
    drawTitle("Pick Object");
    moveit_visual_tools_->trigger();
    objectPose = planning_scene_interface_
        ->getObjectPoses({object_id})[object_id];
    objPose_ = objectPose;
    pickPose = calculateApproachPose(objectPose,pick_offset_);
    move_group_interface_->setJointValueTarget(pickPose);
    result_ = plan("arm_dvrk");
    if (result_.first){
        prompt("Press 'Next' to pick object");
        drawTitle("Pick Object Execution");
        moveit_visual_tools_->trigger();
        move_group_interface_->execute(result_.second);
        attachObject(object_id);
        return true;
    }
    else{
        drawTitle("Pick Object Failed");
        moveit_visual_tools_->trigger();
        moveit_visual_tools_->deleteAllMarkers();
        RCLCPP_ERROR(logger_,"Failed to plan pick object");
        return false;
    }
}

void MBPlanner::attachObject(const std::string& object_id){
    moveit_msgs::msg::CollisionObject obj;
    moveit_msgs::msg::AttachedCollisionObject attachedObj;
    obj = planning_scene_interface_
        ->getObjects({object_id}).at(object_id);
    
    attachedObj = createAttachedObject(obj);
    planning_scene_interface_
        ->applyAttachedCollisionObject(attachedObj);
    obj.operation = obj.REMOVE;
    planning_scene_interface_
        ->applyCollisionObject(obj);
    drawTitle("Attach Goal Box");
    moveit_visual_tools_->trigger();
}

void MBPlanner::detachObject(const std::string& object_id){
    moveit_msgs::msg::CollisionObject obj;
    moveit_msgs::msg::AttachedCollisionObject attachedObj;
    drawTitle("Detach Goal Box");
    moveit_visual_tools_->trigger();
    attachedObj = planning_scene_interface_
        ->getAttachedObjects().at(object_id);
    planning_scene_interface_
        ->applyCollisionObject(attachedObj.object);
    attachedObj.object.operation = attachedObj.object.REMOVE;
    planning_scene_interface_
        ->applyAttachedCollisionObject(attachedObj);
}

bool MBPlanner::closeGrip(const std::string& object_id){
    drawTitle("Close Grip");
    moveit_visual_tools_->trigger();
    grip_group_interface_->setNamedTarget("close");
    result_ = plan("grip");
    if (result_.first){
        prompt("Press 'Next' to close gripper");
        drawTitle("Close Grip Execution");
        moveit_visual_tools_->trigger();
        grip_group_interface_->execute(result_.second);
        return true;
    }
    else{
        drawTitle("Close Grip Failed");
        moveit_visual_tools_->trigger();
        moveit_visual_tools_->deleteAllMarkers();
        RCLCPP_ERROR(logger_,"Failed to plan close grip");
        return false;
    }
}

bool MBPlanner::liftObject(const std::string& object_id){
    geometry_msgs::msg::Pose objectPose;
    geometry_msgs::msg::Pose liftPose;
    updateACM(false);
    drawTitle("Lift Object");
    moveit_visual_tools_->trigger();
    liftPose = calculateApproachPose(objPose_,lift_offset_);
    move_group_interface_->setJointValueTarget(liftPose);
    result_ = plan("arm_dvrk");
    if (result_.first){
        prompt("Press 'Next' to lift object");
        drawTitle("Lift Object Execution");
        moveit_visual_tools_->trigger();
        move_group_interface_->execute(result_.second);
        return true;
    }
    else{
        drawTitle("Lift Object Failed");
        moveit_visual_tools_->trigger();
        moveit_visual_tools_->deleteAllMarkers();
        RCLCPP_ERROR(logger_,"Failed to plan lift object");
        return false;
    }
}

bool MBPlanner::placeObject(const std::string& object_id, const std::string& target_id){
    geometry_msgs::msg::Pose targetPose;
    geometry_msgs::msg::Pose placePose;
    prompt("Press 'Next' to plan to place object");
    updateACM(true);
    drawTitle("Place Object");
    moveit_visual_tools_->trigger();
    targetPose = planning_scene_interface_
        ->getObjectPoses({target_id})[target_id];
    placePose = calculateApproachPose(targetPose,approach_offset_);
    move_group_interface_->setJointValueTarget(placePose);
    result_ = plan("arm_dvrk");
    if (result_.first){
        prompt("Press 'Next' to place object");
        drawTitle("Place Object Execution");
        moveit_visual_tools_->trigger();
        move_group_interface_->execute(result_.second);
        detachObject(object_id);
        return true;
    }
    else{
        drawTitle("Place Object Failed");
        moveit_visual_tools_->trigger();
        moveit_visual_tools_->deleteAllMarkers();
        RCLCPP_ERROR(logger_,"Failed to plan place object");
        detachObject(object_id);
        return false;
    }
}

void MBPlanner::setHome(){
    drawTitle("Set Home");
    moveit_visual_tools_->trigger();
    prompt("Press 'Next' to set home");
    home_ = move_group_interface_->getCurrentJointValues();
    home_set_ = true;
}

bool MBPlanner::moveHome(){
    if (!home_set_){
        RCLCPP_ERROR(logger_,"Home not set");
        return false;
    }
    drawTitle("Move Home");
    moveit_visual_tools_->trigger();
    move_group_interface_->setJointValueTarget(home_);
    result_ = plan("arm_dvrk");
    if (result_.first){
        prompt("Press 'Next' to move home");
        drawTitle("Move Home Execution");
        moveit_visual_tools_->trigger();
        move_group_interface_->execute(result_.second);
        return true;
    }
    else{
        drawTitle("Move Home Failed");
        moveit_visual_tools_->trigger();
        moveit_visual_tools_->deleteAllMarkers();
        RCLCPP_ERROR(logger_,"Failed to plan move home");
        return false;
    }
}

}