#include <ros/ros.h>
#include <tf/tf.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <shape_msgs/SolidPrimitive.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/kinematic_constraints/utils.h>

#include <actionlib/client/simple_action_client.h>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <tmc_control_msgs/GripperApplyEffortAction.h>
#include <tmc_control_msgs/GripperApplyEffortGoal.h>

typedef actionlib::SimpleActionClient<tmc_control_msgs::GripperApplyEffortAction> Client;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveit_grasp");

    ros::NodeHandle node;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "whole_body";
    std::vector<std::string> handLinkNames;

    robot_model_loader::RobotModelLoader robotModelLoader("robot_description");
    robot_model::RobotModelPtr kinematicsModel = robotModelLoader.getModel();
    robot_state::JointModelGroup* joint_model_group = kinematicsModel->getJointModelGroup("gripper");
    handLinkNames = joint_model_group->getLinkModelNames();

    planning_scene::PlanningScene scene(kinematicsModel);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_monitor::PlanningSceneMonitor sceneMonitor("robot_description");

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setPlannerId("RRTstarkConfigDefault");
    move_group.setPlanningTime(30.0);

    ROS_INFO_NAMED("moveit_grasp", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("moveit_grasp", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    sceneMonitor.startSceneMonitor();

    ROS_INFO("Opening Gripper");
    Client cli("/hsrb/gripper_controller/grasp", true);
    cli.waitForServer();
    tmc_control_msgs::GripperApplyEffortGoal applyEffortGoal;
    applyEffortGoal.effort = 0.3;
    cli.sendGoal(applyEffortGoal);
    cli.waitForResult(ros::Duration(5.0));

    tf::TransformListener listener;
    tf::StampedTransform tf_odom_object,tf_odom_pregrasp,tf_odom_grasp;

    listener.waitForTransform("/odom", "/object",ros::Time(0),ros::Duration(20.0));
    listener.lookupTransform("/odom","/object",ros::Time(0),tf_odom_object);
    listener.lookupTransform("/odom","/pregrasp",ros::Time(0),tf_odom_pregrasp);
    listener.lookupTransform("/odom","/grasp",ros::Time(0),tf_odom_grasp);
    ROS_INFO("Recieved transforms for object");

    ROS_INFO("Adding collision to object");
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "box1";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.1;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.1;
    geometry_msgs::Pose boxPose;
    tf::poseTFToMsg(tf_odom_object,boxPose);

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(boxPose);
    collision_object.operation = collision_object.ADD;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    ROS_INFO_NAMED("grasp", "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);

    geometry_msgs::Pose target_pose1, target_pose2;

    tf::poseTFToMsg(tf_odom_pregrasp,target_pose1);
    tf::poseTFToMsg(tf_odom_grasp,target_pose2);

    std::vector<std::string> names;

    ros::ServiceClient client = node.serviceClient<moveit_msgs::ApplyPlanningScene>("/apply_planning_scene");

    moveit_msgs::PlanningScene sceneMsg;
    collision_detection::AllowedCollisionMatrix& acm = sceneMonitor.getPlanningScene()->getAllowedCollisionMatrixNonConst();
    acm.getAllEntryNames(names);
    for(int i = 0; i < names.size(); i++)
    {
        if(names[i].c_str()[0] == 'b')
        {
            acm.setEntry(names[i],"<octomap>",true);
        }
    }

    acm.getMessage(sceneMsg.allowed_collision_matrix);
    sceneMsg.is_diff = true;
    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = sceneMsg;
    client.call(srv);
    ROS_INFO("ACM Sent");

    ROS_INFO("Set pose target");
    move_group.setPoseTarget(target_pose1);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    ROS_INFO("Planned Pregrasp state");
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    int maxTries = 10;
    while(success == false && maxTries >= 0)
    {
        ROS_WARN("Retrying");
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        maxTries --;
    }
    ROS_INFO("Move to Pregrasp state");
    move_group.move();

    geometry_msgs::Pose currentPose = move_group.getCurrentPose().pose;
    ROS_INFO("Current pose: %f %f %f",currentPose.position.x,currentPose.position.y,currentPose.position.z);

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose t1,t2;
    tf::Transform tf_pregrasp_change2(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.1));
    tf::Transform tf_currentPose;
    tf::poseMsgToTF(currentPose,tf_currentPose);
    tf::Transform tf_pregrasp2 = tf_currentPose * tf_pregrasp_change2;
    tf::poseTFToMsg(tf_pregrasp2,t1);

    waypoints.push_back(currentPose);
    waypoints.push_back(target_pose2);
    
    move_group.setMaxVelocityScalingFactor(0.1);

    ROS_INFO("Computing Cartesian Path to Grasp state");
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("grasp", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    my_plan.trajectory_ = trajectory;
    ROS_INFO("Executing plan to Grasp State");
    move_group.execute(my_plan);

    move_group.attachObject(collision_object.id,move_group.getEndEffectorLink(),handLinkNames);
    ros::Duration(5.0).sleep();
    ROS_INFO("Trying to grasp");
    applyEffortGoal.effort = -0.5;
    cli.sendGoal(applyEffortGoal);
    cli.waitForResult(ros::Duration(5.0));
    ROS_INFO("Grasping executed");

    move_group.setStartStateToCurrentState();
    move_group.setSupportSurfaceName("<octomap>");
    currentPose = move_group.getCurrentPose().pose;
    ROS_INFO("Current pose: %f %f %f",currentPose.position.x,currentPose.position.y,currentPose.position.z);

    waypoints.clear();
    tf::Transform tf_post_grasp_change(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, -0.1));
    tf::poseMsgToTF(currentPose,tf_currentPose);
    tf::Transform tf_post_grasp = tf_currentPose * tf_post_grasp_change;
    tf::poseTFToMsg(tf_post_grasp,t2);
    waypoints.push_back(t2);

    ROS_INFO("Computing Cartesian Path to Post Grasp state");
    fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("grasp", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    if(fraction <= 0.5)
    {
        ROS_INFO("Failed to plan Post Grasp");
        return 0;
    }
    my_plan.trajectory_ = trajectory;
    ROS_INFO("Executing plan to Post Grasp State");
    move_group.execute(my_plan);

    ROS_INFO("Moving to final state");
    geometry_msgs::Pose finalPose;
    finalPose.position.x = 0.566;
    finalPose.position.y = -0.23;
    finalPose.position.z = 0.5;
    finalPose.orientation.x = finalPose.orientation.y = 0.0;
    finalPose.orientation.z = 0.76;
    finalPose.orientation.w = 0.644;

    move_group.setPoseTarget(finalPose);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Script complete");

    return 0;
}