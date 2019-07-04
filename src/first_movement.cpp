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
    ros::init(argc, argv, "first_movement");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base_link"));
    visual_tools_->deleteAllMarkers();


    static const std::string PLANNING_GROUP = "manipulator";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


    geometry_msgs::Pose p1, temp_p;
    tf2::Quaternion orientation_p1;
    orientation_p1.setRPY(0,0,0);
    p1.position.x = 0.8;
    p1.position.y = 0.1;
    p1.position.z = 0.8;
    p1.orientation = tf2::toMsg(orientation_p1);
    
    Eigen::Affine3d temp,rot1,rot2;
    //tf2::fromMsg(p1, temp);
    temp.linear().setIdentity();
    temp.translation() = Eigen::Vector3d(0.5, 0.5, 0.5);
    temp.rotate(Eigen::AngleAxis<double>(135 * M_PI / 180, Eigen::Vector3d(0, 1, 0)));
    visual_tools_->publishAxisLabeled(temp, "temp_inital");
    visual_tools_->trigger();

    rot1.linear().setIdentity();
    rot1.rotate(Eigen::AngleAxis<double>(20* M_PI / 180, Eigen::Vector3d(1, 0, 0)));
    rot2.linear().setIdentity();
    rot2.rotate(Eigen::AngleAxis<double>(-20* M_PI / 180, Eigen::Vector3d(1, 0, 0)));

    std::vector<geometry_msgs::Pose> waypoints;

    float r = 0;

    for (int alpha = 0; alpha <360; alpha += 20)
    {
        //temp.rotate(Eigen::AngleAxis<double>(alpha * M_PI / 180, Eigen::Vector3d(0, 0, 1)));
        temp.prerotate(rot1.linear());
        //temp.rotate(rot.linear());
        temp.translation() = Eigen::Vector3d(p1.position.x - r - r * cos((alpha - 180) * M_PI / 180), p1.position.y + r * sin((alpha - 180) * M_PI / 180), p1.position.z + r * sin((alpha - 180) * M_PI / 180));
        //temp.translation() = Eigen::Vector3d(p1.position.x-r - r * cos((alpha - 180) * M_PI / 180), p1.position.y , p1.position.z + r * sin((alpha - 180) * M_PI / 180));
        temp_p = tf2::toMsg(temp);
        waypoints.push_back(temp_p);
        //ros::Duration(5.0).sleep();
        visual_tools_->publishAxis(temp, rviz_visual_tools::scales::XSMALL);
        visual_tools_->trigger();
        // point_pub.publish(temp_p);
        
       

        if (alpha == 360)
        {
            for (int beta = 0; beta < 360; beta += 20)
            {
                //temp.rotate(Eigen::AngleAxis<double>(-(alpha + beta) * M_PI / 180, Eigen::Vector3d(0, 0, 1)));
                temp.prerotate(rot2.linear());
                temp.translation() = Eigen::Vector3d(p1.position.x + r - r * cos((360 - alpha - beta) * M_PI / 180), p1.position.y - r * sin((360 - alpha - beta) * M_PI / 180), p1.position.z + r * sin((360 - alpha - beta) * M_PI / 180));
                //temp.translation() = Eigen::Vector3d(p1.position.x + r - r * cos((360 - alpha - beta) * M_PI / 180),  p1.position.y , p1.position.z + r * sin((360 - alpha - beta) * M_PI / 180));
                temp_p = tf2::toMsg(temp);
                waypoints.push_back(temp_p);
                visual_tools_->publishAxis(temp, rviz_visual_tools::scales::XSMALL);
                visual_tools_->trigger();
                
            }
        }
    }



    invite_utils::CartesianTaskPlanner task_planner;

    float motion_achievable;
    int max_ik = 20;
    std::vector<std::string> allowed_touch_objects;
    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> motion_plans;
    motion_achievable = task_planner.planCartesianMotionTask(&move_group,
                                                             waypoints,
                                                             motion_plans,
                                                             allowed_touch_objects,
                                                             max_ik);

    // Perform approach and cartesian path motion.
    if (motion_achievable > 0.7)
    {
        move_group.execute(motion_plans[0]); // Approach motion.

        ros::Duration(0.1).sleep();

        move_group.execute(motion_plans[1]); // Cartesian motion.
    }

    // Visualize the plan in RViz
    visual_tools_->publishIKSolutions(task_planner.getDiscartedRobotConfigurations(), joint_model_group, 0);
    visual_tools_->publishTrajectoryPath(motion_plans[0].trajectory_,  motion_plans[0].start_state_, false);        
    visual_tools_->publishTrajectoryPath(motion_plans[1].trajectory_, motion_plans[1].start_state_, false);        
    visual_tools_->trigger();

    ros::waitForShutdown();
}