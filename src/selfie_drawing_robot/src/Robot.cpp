#include "Robot.h"

Robot::Robot(): PLANNING_GROUP("manipulator"), move_group_interface(PLANNING_GROUP),
joint_model_group(move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP)),
visual_tools("base_link")
{
    visual_tools.deleteAllMarkers();
    // remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visual_tools.loadRemoteControl();
    // we can print the name of the reference frame for this robot
    ROS_INFO("Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    // we can also print the name of the end-effector link for this group
    ROS_INFO("End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
    // we can get a list of all the groups in the robot
    ROS_INFO("Available Planning Groups:");
    std::copy(move_group_interface.getJointModelGroupNames().begin(),
        move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
}

bool Robot::planPoseTarget(geometry_msgs::Pose target_pose)
{
    move_group_interface.setPoseTarget(target_pose);
    bool success = (move_group_interface.plan(current_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO("Attempting to plan pose target... %s", success ? "" : "FAILED");
    return success;
}

void Robot::visualisePlan(geometry_msgs::Pose target_pose)
{
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(target_pose, "target_pose");
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(current_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
}

void Robot::awaitUserPrompt()
{
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to proceed...");
}
