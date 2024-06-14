#include "operations.h"
#include <pluginlib/class_loader.hpp>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/scoped_ptr.hpp>
#include <iostream>
#include <thread>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <signal.h>
#include <atomic>



// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


const double tau = 2 * M_PI;
int numMarkerCallBacks = 0;
Operations ops;
std::atomic<bool> g_stopThreads(false);
Eigen::Vector3d currentPose;

void signalHandler(int signum) {
    if (signum == SIGINT) {
        g_stopThreads.store(true);
    }
}

// Interface with UI
void uiCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Received message: %s", msg->data.c_str());

    // Assuming ops is an instance of your Operations class
    if (msg->data == "calibration 0") {
        ops.topLeftCorner = currentPose;
    } else if (msg->data == "calibration 1") {
        ops.topRightCorner = currentPose;
    } else if (msg->data == "calibration 2") {
        ops.bottomLeftCorner = currentPose;
    } else if (msg->data == "calibration 3") {
        ops.bottomRightCorner = currentPose;
    } else {
        ROS_WARN("Invalid calibration message: %s", msg->data.c_str());
    }
}

void publishArr(ros::Publisher* pub, visualization_msgs::MarkerArray* arr){
  ros::Rate rate(1); // Adjust publishing rate as needed
  while (ros::ok() && !g_stopThreads.load()) {
      pub->publish(*arr);
      // ros::spinOnce();
      rate.sleep();
  }
}

void publishMrk(ros::Publisher* pub, std::vector<visualization_msgs::Marker>* sphere){
  ros::Rate rate(1); // Adjust publishing rate as needed
  while (numMarkerCallBacks<4 && !g_stopThreads.load()) {
    for (auto marker : *sphere)
      pub->publish(marker);
    rate.sleep();
  }

}

void calibratorSimCallBack(const geometry_msgs::PointStamped::ConstPtr& msg) {
  ROS_INFO("\n Received point: x=%f, y=%f, z=%f", msg->point.x, msg->point.y, msg->point.z);

  switch (numMarkerCallBacks) {
      case 0:
          ops.bottomLeftCorner.x() = msg->point.x;
          ops.bottomLeftCorner.y() = msg->point.y;
          ops.bottomLeftCorner.z() = msg->point.z;
          break;
      case 1:
          ops.topLeftCorner.x() = msg->point.x;
          ops.topLeftCorner.y() = msg->point.y;
          ops.topLeftCorner.z() = msg->point.z;
          break;
      case 2:
          ops.topRightCorner.x() = msg->point.x;
          ops.topRightCorner.y() = msg->point.y;
          ops.topRightCorner.z() = msg->point.z;
          break;
      case 3:
          ops.bottomRightCorner.x() = msg->point.x;
          ops.bottomRightCorner.y() = msg->point.y;
          ops.bottomRightCorner.z() = msg->point.z;
          break;
      default:
          ROS_ERROR("Invalid number of callbacks");
          break;
  }

  ++numMarkerCallBacks;
}

int main(int argc, char** argv)
{
  // Allow time for rviz launch
  std::this_thread::sleep_for(std::chrono::seconds(5));


  // NODE SETUP
  const std::string node_name = "current_program";
  ros::init(argc, argv, node_name);
  ros::NodeHandle node_handle("~");
  // Create a publisher to publish messages of type std_msgs::String on the "gui_com" topic
  ros::Publisher pub = node_handle.advertise<std_msgs::String>("gui_com", 10);
  // Create a subscriber to subscribe to messages of type std_msgs::String on the "gui_com" topic
  ros::Subscriber sub_ui = node_handle.subscribe("gui_com", 10, uiCallback);
  // SIMULATION SPECIFIC
  // Publisher for marker array drawing
  ros::Publisher marker_arr_pub = node_handle.advertise<visualization_msgs::MarkerArray>("markers", 10);
  // Publisher for marker array corners
  ros::Publisher marker_corners_pub = node_handle.advertise<visualization_msgs::MarkerArray>("corners", 10);
  // Publisher for markers
  ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::Marker>("sphere_markers", 100);
  // Subscriber to clicked points
  ros::Subscriber sub_calibration_sim = node_handle.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, calibratorSimCallBack);
  ros::AsyncSpinner spinner(1);
  spinner.start();


  // SIMULATION
  // DRAWSPACE SELECTION
  geometry_msgs::Point ctr;
  ctr.x = 0.0;
  ctr.y = 0.0;
  ctr.z = .25;
  std::vector<visualization_msgs::Marker> sphere = ops.createSphereMarkers(ctr, 0.4, 40);
  std::thread marker_thread(publishMrk, &marker_pub, &sphere);
  ROS_INFO("\n Calibrate by clicking corners from bottom left in clockwise fashion.");
  ros::Rate rate(2);
  marker_thread.join();
  ROS_INFO("\n Calibration completed.");
  // VISUALISE SPATIAL TRAJECTORY & RESULTANT DRAWSPACE CORNERS
  // Convert SVG and render spatial draw-space coordinates of all strokes
  ops.calculateDrawSpaceTransformation();
  ops.svg_to_contours("sample_5");
  ops.renderSpatialData();
  // Create a marker array message
  visualization_msgs::MarkerArray marker_array;
  // Populate marker array with spheres using Operations class methods
  marker_array = ops.generateMarkerArray();
  // Publish marker array 
  std::thread trajThread(publishArr, &marker_arr_pub, &marker_array);
  ROS_INFO("\n Publishing SVG data to transformed spatial trajectory.");
  // Publish corners of drawspace
  auto corners = ops.getCornersDrawSpace();

  std::thread cornersThread(publishArr, &marker_corners_pub, &corners);
  // CLEANUP
  // Wait for Ctrl+C to stop threads
  while (!g_stopThreads.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  trajThread.join();
  cornersThread.join();
  return 1;



  // while(1)
  //   std::this_thread::sleep_for(std::chrono::seconds(5));

  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState(2);



  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");



  std::vector<double> joint_group_positions_arm;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_arm);
  move_group_interface.setStartStateToCurrentState();
  // joint_group_positions_arm[0] = 0.00 //Shoulder Pan
  joint_group_positions_arm[1] = 2.50;    //Shoulder lift
  joint_group_positions_arm[2] = 1.50;    //Elbow
  joint_group_positions_arm[3] = -1.50;   //Wrist 1
  joint_group_positions_arm[4] = -1.55;   //Wrist 2
  joint_group_positions_arm[5] = 0.00;    //Wrist 3
  move_group_interface.setJointValueTarget(joint_group_positions_arm);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success_arm = (move_group_interface.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
  move_group_interface.execute(my_plan_arm);



  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


  
  joint_group_positions_arm = {1.0, 1.0, 1.0, 0.0, 0.0, 0.0};
  move_group_interface.setJointValueTarget(joint_group_positions_arm);

  // Plan the trajectory
  success_arm = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success_arm) move_group_interface.execute(my_plan_arm);
  else ROS_WARN("Failed to plan a motion to the joint space goal.");

  // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_arm);
  // move_group_interface.setStartStateToCurrentState();
  // // joint_group_positions_arm[0] = 0.00 //Shoulder Pan
  // joint_group_positions_arm[1] = 1.50;    //Shoulder lift
  // joint_group_positions_arm[2] = 2.50;    //Elbow
  // joint_group_positions_arm[3] = 1.50;   //Wrist 1
  // joint_group_positions_arm[4] = 1.55;   //Wrist 2
  // joint_group_positions_arm[5] = 0.00;    //Wrist 3
  // move_group_interface.setJointValueTarget(joint_group_positions_arm);
  // success_arm = (move_group_interface.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
  // move_group_interface.execute(my_plan_arm);




  // joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  // current_state = move_group_interface.getCurrentState(10);
  // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_arm);

  // geometry_msgs::Pose target_pose;
  // target_pose.orientation.x = -1.0;
  // target_pose.orientation.y = 0.0;
  // target_pose.orientation.z = 0.0;
  // target_pose.orientation.w = 0.0;
  // target_pose.position.x = 0.343;
  // target_pose.position.y = 0.132;
  // target_pose.position.z = 0.264;
  // move_group_interface.setPoseTarget(target_pose);

  // success_arm = (move_group_interface.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
  // move_group_interface.execute(my_plan_arm);


  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


  // .. _move_group_interface-planning-to-pose-goal:
  //
  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.5;
  move_group_interface.setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group_interface
  // to actually move the robot.

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Finally, to execute the trajectory stored in my_plan, you could use the following method call:
  // Note that this can lead to problems if the robot moved in the meanwhile.
  // move_group_interface.execute(my_plan);

  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // If you do not want to inspect the planned trajectory,
  // the following is a more robust combination of the two-step plan+execute pattern shown above
  // and should be preferred. Note that the pose goal we had set earlier is still active,
  // so the robot will try to move to that goal.

  // move_group_interface.move();

  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  //
  // To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  current_state = move_group_interface.getCurrentState();
  //

  std::vector<double> joint_group_positions;
  // Next get the current set of joint values for the group.
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = -tau / 6;  // -1/6 turn in radians
  move_group_interface.setJointValueTarget(joint_group_positions);

  // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
  // The default values are 10% (0.1).
  // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
  // or set explicit factors in your code if you need your robot to move faster.
  move_group_interface.setMaxVelocityScalingFactor(0.05);
  move_group_interface.setMaxAccelerationScalingFactor(0.05);

  success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Path constraints can easily be specified for a link on the robot.
  // Let's specify a path constraint and a pose goal for our group.
  // First define the path constraint.
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "panda_link7";
  ocm.header.frame_id = "panda_link0";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group_interface.setPathConstraints(test_constraints);

  // Enforce Planning in Joint Space
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Depending on the planning problem MoveIt chooses between
  // ``joint space`` and ``cartesian space`` for problem representation.
  // Setting the group parameter ``enforce_joint_model_state_space:true`` in
  // the ompl_planning.yaml file enforces the use of ``joint space`` for all plans.
  //
  // By default planning requests with orientation path constraints
  // are sampled in ``cartesian space`` so that invoking IK serves as a
  // generative sampler.
  //
  // By enforcing ``joint space`` the planning process will use rejection
  // sampling to find valid requests. Please note that this might
  // increase planning time considerably.
  //
  // We will reuse the old goal that we had and plan to it.
  // Note that this will only work if the current state already
  // satisfies the path constraints. So we need to set the start
  // state to a new pose.
  moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  start_state.setFromIK(joint_model_group, start_pose2);
  move_group_interface.setStartState(start_state);

  // Now we will plan to the earlier pose target from the new
  // start state that we have just created.
  move_group_interface.setPoseTarget(target_pose1);

  // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  move_group_interface.setPlanningTime(10.0);

  success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose2, "start");
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // When done with the path constraint be sure to clear it.
  move_group_interface.clearPathConstraints();

  // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  // You can plan a Cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through. Note that we are starting
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list but adding it can help with visualizations
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose2);

  geometry_msgs::Pose target_pose3 = start_pose2;

  target_pose3.position.z -= 0.2;
  waypoints.push_back(target_pose3);  // down

  target_pose3.position.y -= 0.2;
  waypoints.push_back(target_pose3);  // right

  target_pose3.position.z += 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // up and left

  // We want the Cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in Cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishTrajectoryLine(trajectory, joint_model_group);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Cartesian motions should often be slow, e.g. when approaching objects. The speed of cartesian
  // plans cannot currently be set through the maxVelocityScalingFactor, but requires you to time
  // the trajectory manually, as described `here <https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4>`_.
  // Pull requests are welcome.
  //
  // You can execute a trajectory like this:
  // move_group_interface.execute(trajectory);

  // Adding objects to the environment
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // First let's plan to another simple goal with no objects in the way.
  move_group_interface.setStartState(*move_group_interface.getCurrentState());
  geometry_msgs::Pose another_pose;
  another_pose.orientation.x = 1.0;
  another_pose.position.x = 0.7;
  another_pose.position.y = 0.0;
  another_pose.position.z = 0.59;
  move_group_interface.setPoseTarget(another_pose);

  success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (with no obstacles) %s", success ? "" : "FAILED");

  visual_tools.deleteAllMarkers();
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // The result may look like this:
  //
  // .. image:: ./move_group_interface_tutorial_clear_path.gif
  //    :alt: animation showing the arm moving relatively straight toward the goal
  //
  // Now let's define a collision object ROS message for the robot to avoid.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.1;
  primitive.dimensions[primitive.BOX_Y] = 1.5;
  primitive.dimensions[primitive.BOX_Z] = 0.5;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.5;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.25;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  // (using a vector that could contain additional objects)
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

  // Now when we plan a trajectory it will avoid the obstacle
  success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 6 (pose goal move around cuboid) %s", success ? "" : "FAILED");
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

  // The result may look like this:
  //
  // .. image:: ./move_group_interface_tutorial_avoid_path.gif
  //    :alt: animation showing the arm moving avoiding the new obstacle
  //
  // Attaching objects to the robot
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // You can attach objects to the robot, so that it moves with the robot geometry.
  // This simulates picking up the object for the purpose of manipulating it.
  // The motion planning should avoid collisions between the two objects as well.
  moveit_msgs::CollisionObject object_to_attach;
  object_to_attach.id = "cylinder1";

  shape_msgs::SolidPrimitive cylinder_primitive;
  cylinder_primitive.type = primitive.CYLINDER;
  cylinder_primitive.dimensions.resize(2);
  cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.20;
  cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

  // We define the frame/pose for this cylinder so that it appears in the gripper
  object_to_attach.header.frame_id = move_group_interface.getEndEffectorLink();
  geometry_msgs::Pose grab_pose;
  grab_pose.orientation.w = 1.0;
  grab_pose.position.z = 0.2;

  // First, we add the object to the world (without using a vector)
  object_to_attach.primitives.push_back(cylinder_primitive);
  object_to_attach.primitive_poses.push_back(grab_pose);
  object_to_attach.operation = object_to_attach.ADD;
  planning_scene_interface.applyCollisionObject(object_to_attach);

  // Then, we "attach" the object to the robot at the given link and allow collisions between the object and the listed
  // links. You could also use applyAttachedCollisionObject to attach an object to the robot directly.
  ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
  move_group_interface.attachObject(object_to_attach.id, "panda_hand", { "panda_leftfinger", "panda_rightfinger" });

  visual_tools.trigger();

  /* Wait for MoveGroup to receive and process the attached collision object message */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");

  // Replan, but now with the object in hand.
  move_group_interface.setStartStateToCurrentState();
  success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 7 (move around cuboid with cylinder) %s", success ? "" : "FAILED");
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

  // The result may look something like this:
  //
  // .. image:: ./move_group_interface_tutorial_attached_object.gif
  //    :alt: animation showing the arm moving differently once the object is attached
  //
  // Detaching and Removing Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Now, let's detach the cylinder from the robot's gripper.
  ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
  move_group_interface.detachObject(object_to_attach.id);

  // Show text in RViz of status
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  /* Wait for MoveGroup to receive and process the attached collision object message */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");

  // Now, let's remove the objects from the world.
  ROS_INFO_NAMED("tutorial", "Remove the objects from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  object_ids.push_back(object_to_attach.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // Show text in RViz of status
  visual_tools.trigger();

  /* Wait for MoveGroup to receive and process the attached collision object message */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disappears");

  // END_TUTORIAL

  ros::shutdown();
  return 0;
}