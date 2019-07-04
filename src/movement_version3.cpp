#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <math.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <invite_utils/cartesian_task_planner.h>

moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "first_movement2");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base_link"));
    visual_tools_->deleteAllMarkers();

    static const std::string PLANNING_GROUP = "manipulator";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    move_group.setMaxAccelerationScalingFactor(0.01);
    move_group.setMaxVelocityScalingFactor(0.01);

    geometry_msgs::Pose p1, temp_p;
    tf2::Quaternion orientation_p1;
    orientation_p1.setRPY(0, 0, 0);
    p1.position.x = 0.3;
    p1.position.y = 0.77;
    p1.position.z = 0.45;
    p1.orientation = tf2::toMsg(orientation_p1);

    Eigen::Affine3d temp, rot1, rot2;
    //tf2::fromMsg(p1, temp);
    temp.linear().setIdentity();
    temp.translation() = Eigen::Vector3d(0.5, 0.5, 0.5);
    temp.rotate(Eigen::AngleAxis<double>(270 * M_PI / 180, Eigen::Vector3d(1, 0, 0)));
    visual_tools_->publishAxisLabeled(temp, "temp_inital");
    visual_tools_->trigger();

    rot1.linear().setIdentity();
    rot1.rotate(Eigen::AngleAxis<double>(20 * M_PI / 180, Eigen::Vector3d(0, 1, 0)));
    rot2.linear().setIdentity();
    rot2.rotate(Eigen::AngleAxis<double>(-20 * M_PI / 180, Eigen::Vector3d(0, 1, 0)));

    std::vector<geometry_msgs::Pose> waypoints;

    float r = 0.2;
    ROS_INFO("Error has not happen yet: 1");
    for (float alpha = -180; alpha < 180; alpha += 20)
    {
        temp.prerotate(rot1.linear());
        temp.translation() = Eigen::Vector3d(p1.position.x - r * (180 - abs(alpha)) / 180, p1.position.y, p1.position.z);
        temp_p = tf2::toMsg(temp);
        waypoints.push_back(temp_p);
        //ros::Duration(5.0).sleep();
        visual_tools_->publishAxis(temp);
        visual_tools_->trigger();

        if (alpha == 160)
        {
            for (float beta = -180; beta < 180; beta += 20)
            {
                temp.prerotate(rot2.linear());
                temp.translation() = Eigen::Vector3d(p1.position.x + r * (180 - abs(beta)) / 180, p1.position.y, p1.position.z);

                temp_p = tf2::toMsg(temp);
                waypoints.push_back(temp_p);
                visual_tools_->publishAxis(temp);
                visual_tools_->trigger();
                //temp.rotate(Eigen::AngleAxis<double>((alpha + beta) * M_PI / 180, Eigen::Vector3d(0, 0, 1)));
            }
        }
    }
    ROS_INFO("Error has not happen yet: 2");

    invite_utils::CartesianTaskPlanner task_planner;

    float motion_achievable;
    int max_ik = 15;
    std::vector<std::string> allowed_touch_objects;
    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> motion_plans;
    motion_achievable = task_planner.planCartesianMotionTask(&move_group,
                                                             waypoints,
                                                             motion_plans,
                                                             allowed_touch_objects,
                                                             max_ik);

    ROS_WARN("Trajectory achievable: %.2f" , motion_achievable*100);

    // Perform approach and cartesian path motion.
    if (motion_achievable > 0.1)
    {   
        //move_group.execute(motion_plans[0]); // Approach motion.
        //move_group.execute(motion_plans[1]); // Cartesian motion.
        while (ros::ok() && motion_achievable >= 1)
        {
            ros::Duration(0.3).sleep();
            //move_group.execute(motion_plans[1]); // Cartesian motion.
        }
        if (task_planner.getDiscartedRobotConfigurations().size() > 0)
        {
            ROS_WARN("NPOOOOOOOWWWW .           .................. \n \n \n");
            //visual_tools_->publishIKSolutions(task_planner.getDiscartedRobotConfigurations(), joint_model_group, 1.5);
            ros::Duration(5).sleep();
            ROS_WARN("My error is not Daniels code2222222");
            //visual_tools_->publishTrajectoryPath(motion_plans[0].trajectory_, motion_plans[0].start_state_, false);
            ROS_WARN("My error is not Daniels code2222222");
            visual_tools_->publishTrajectoryPath(motion_plans[1].trajectory_, motion_plans[1].start_state_, false);
            ROS_WARN("My error is not Daniels code2222222");
        }
        visual_tools_->trigger();
    }
    

    ros::waitForShutdown();
}