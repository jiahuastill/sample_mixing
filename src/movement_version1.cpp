#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


#include <moveit_visual_tools/moveit_visual_tools.h>
#include <invite_utils/cartesian_task_planner.h>

moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

std::vector<geometry_msgs::Pose> waypoints;

typedef moveit::planning_interface::MoveGroupInterface MoveGroupinterface;

std::vector<geometry_msgs::Pose> ComputeWaypoints(float, geometry_msgs::Pose&);//this function computes the trajectory, given a desired radius and a starting position

void MoveRobot(std::vector<geometry_msgs::Pose>&);//this function will move the robot along the computed trajectory if motion is 100% achievable.

int main(int argc, char **argv)
{
    ros::init(argc, argv, "first_movement4");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();
  
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base_link"));
    visual_tools_->deleteAllMarkers();

    
    geometry_msgs::Pose p1, temp_p;
    tf2::Quaternion orientation_p1;
    orientation_p1.setRPY(270 * M_PI / 180, M_PI, 0);
    p1.position.x = 0.3;
    p1.position.y = 0.65;
    p1.position.z = 0.4;
    p1.orientation = tf2::toMsg(orientation_p1);

    std::vector<geometry_msgs::Pose> waypoints = ComputeWaypoints(0.12,p1);
    MoveRobot(waypoints);


    ros::waitForShutdown();
}

std::vector<geometry_msgs::Pose> ComputeWaypoints(float r, geometry_msgs::Pose& p1)

{
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(p1);
    geometry_msgs::Pose temp_p; 

    Eigen::Affine3d initial, temp, rot1, rot2;
    tf2::fromMsg(p1, temp);
    tf2::fromMsg(p1, initial);
   
    rot1.linear().setIdentity();
    rot1.rotate(Eigen::AngleAxis<double>(20 * M_PI / 180, Eigen::Vector3d(0, 1, 0)));
    rot2.linear().setIdentity();
    rot2.rotate(Eigen::AngleAxis<double>(-20 * M_PI / 180, Eigen::Vector3d(0, 1, 0)));
   

    for (float alpha = 20; alpha < 380; alpha += 20)
    
    {

        temp.prerotate(rot1.linear());
        temp.translation() = initial.linear().inverse() * Eigen::Vector3d(r * cos((180 + alpha) * M_PI / 180) + r, 0.5 * r * sin((180 + alpha) * M_PI / 180), 0) + initial.translation();
    
        temp_p = tf2::toMsg(temp);

        waypoints.push_back(temp_p);

        visual_tools_->publishAxis(temp);
        visual_tools_->trigger();
        //ros::Duration(1.0).sleep();
       
        if (alpha == 360)
        {
           
            for (float beta = 20; beta < 380; beta += 20)
            {
                temp.prerotate(rot2.linear());
                temp.translation() = initial.linear().inverse() * Eigen::Vector3d(r * cos((360 - beta) * M_PI / 180) - r, 0.5 * r * sin((360 - beta) * M_PI / 180), 0) + initial.translation();

                temp_p = tf2::toMsg(temp);
                waypoints.push_back(temp_p);

                visual_tools_->publishAxis(temp);
                visual_tools_->trigger();
                //ros::Duration(1.0).sleep();
                //ROS_WARN_STREAM(__LINE__);
            }
        }
    }  
    
    ROS_INFO("waypoints have %d points",waypoints.size());
    return waypoints;

}


void MoveRobot(std::vector<geometry_msgs::Pose>& points)

{
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    
    move_group.setMaxAccelerationScalingFactor(1.0);
    move_group.setMaxVelocityScalingFactor(0.7);
    //move_group.setGoalJointTolerance(0.00001);    // HERE !!
    //move_group.setGoalTolerance(0.0001);
    invite_utils::CartesianTaskPlanner task_planner;

    float motion_achievable;
    int max_ik = 20;
    std::vector<std::string> allowed_touch_objects;
    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> motion_plans;

    motion_achievable = task_planner.planCartesianMotionTask(&move_group,
                                                             points,
                                                             motion_plans,
                                                             allowed_touch_objects,
                                                             max_ik);

    ROS_ERROR("----------------------------------------------"); 
    ROS_INFO("%.3f %% of motion is possible ... %d IK configurations considered", motion_achievable, (int) task_planner.getDiscartedRobotConfigurations().size() ); 

    MoveGroupinterface::Plan approach_plan = motion_plans[0];
    MoveGroupinterface::Plan cartesian_plan = motion_plans[1];


    
    std::vector<trajectory_msgs::JointTrajectoryPoint> final_trajectory = cartesian_plan.trajectory_.joint_trajectory.points;

    /*int num_point=final_trajectory.size();
    int num_joint=final_trajectory.at(0).velocities.size();
    for (int it_i=0; it_i<num_point; it_i++)
    {
        for(int it_j=0; it_j<num_joint; it_j++)
        {
            ROS_INFO("current point is %d, current joint is %d, current velocity is %f", it_i,it_j,final_trajectory.at(it_i).velocities.at(it_j));
            //ROS_INFO("current point is %d, current joint is %d, current acceleration is %f", it_i,it_j,final_trajectory.at(it_i).accelerations.at(it_j));
        }
    }*/


    float original_size = final_trajectory.size(); 
    for (int i = 0; i < 4; i++) {
        float last_time_stamp = final_trajectory.back().time_from_start.toSec(); 
        for (int point_idx = 0; point_idx < original_size; point_idx++) {
            float delta_time = point_idx == 0 ? 0.001 : 0;
            final_trajectory.push_back(final_trajectory[point_idx]);
            final_trajectory.back().time_from_start = ros::Duration(final_trajectory[point_idx].time_from_start.toSec() + last_time_stamp + delta_time);
        }
    }

    if (motion_achievable >= 1)
    {
        // Perform approach and cartesian path motion.
        move_group.execute(approach_plan);

        if (task_planner.getDiscartedRobotConfigurations().size() > 0)
        {
           
            visual_tools_->publishIKSolutions(task_planner.getDiscartedRobotConfigurations(), joint_model_group, 1.5);

            
           // visual_tools_->publishTrajectoryPath(motion_plans[0].trajectory_, motion_plans[0].start_state_, false);
            
            //visual_tools_->publishTrajectoryPath(motion_plans[1].trajectory_, motion_plans[1].start_state_, false);
            
        }
        visual_tools_->trigger();
  
        cartesian_plan.trajectory_.joint_trajectory.points=final_trajectory;

        move_group.execute(cartesian_plan);
        
    }

}