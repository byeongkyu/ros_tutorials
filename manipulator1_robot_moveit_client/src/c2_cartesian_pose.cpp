#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveit_client_c1_target_pose");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose target_pose1;
    // target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.5;
    target_pose1.position.y = 0.1;
    target_pose1.position.z = 0.5;

    tf2::Quaternion q_target;
    q_target.setRotation(tf2::Vector3(0, 1, 0), M_PI/2.0);

    target_pose1.orientation.x = q_target.x();
    target_pose1.orientation.y = q_target.y();
    target_pose1.orientation.z = q_target.z();
    target_pose1.orientation.w = q_target.w();


    move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group.move();



    waypoints.push_back(target_pose1);

    target_pose1.position.y -= 0.2;
    waypoints.push_back(target_pose1);  // right

    target_pose1.position.z += 0.2;
    waypoints.push_back(target_pose1);  // up

    target_pose1.position.y += 0.2;
    waypoints.push_back(target_pose1);  // left

    move_group.setMaxVelocityScalingFactor(0.1);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.move();

    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // if(success) {
    //     ROS_INFO("Success!");
    //     // move_group.move();
    // }
    // else {
    //     ROS_ERROR("Error!");
    // }

    return 0;
}