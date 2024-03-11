#ifndef ROBOT_H
#define ROBOT_H

// required moveit packages
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;

class Robot{
    
    public:
        Robot();
        bool planPoseTarget(geometry_msgs::Pose target);
        void visualisePlan(geometry_msgs::Pose target_pose);
        void awaitUserPrompt();
        ~Robot(){}
        
    private:
        const std::string PLANNING_GROUP;
        moveit::planning_interface::MoveGroupInterface move_group_interface;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface; // class to add and remove collision objects in our "virtual world" scene
        const moveit::core::JointModelGroup* joint_model_group;
        moveit_visual_tools::MoveItVisualTools visual_tools;
        moveit::planning_interface::MoveGroupInterface::Plan current_plan;
};

#endif // ROBOT_H