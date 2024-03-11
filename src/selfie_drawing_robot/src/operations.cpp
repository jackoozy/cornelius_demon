#include "Robot.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "operations");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    Robot robot;

    robot.awaitUserPrompt();

    geometry_msgs::Pose pose;
    pose.orientation.w = 1;
    pose.position.x = 0.4;
    pose.position.y = 0.2;
    pose.position.z = 0.5;

    if(robot.planPoseTarget(pose)) robot.visualisePlan(pose);

    robot.awaitUserPrompt();

    return 1;
}