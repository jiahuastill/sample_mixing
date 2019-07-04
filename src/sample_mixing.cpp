#include "ros/ros.h"
//#include <rviz_visual_tools/rviz_visual_tools.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <tf2_eigen/tf2_eigen.h>

#include <pluginlib/class_loader.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <boost/scoped_ptr.hpp>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sample_mixing");
    ros::NodeHandle n;

    /*Eigen::Vector3d point0(0,0,0);
    Eigen::Vector3d point1(0,-1,1.1547);
    Eigen::Vector3d point2(1.1547,-1,0);
    Eigen::Vector3d point3(0,-1,-1.1547);
    Eigen::Vector3d point4(-1.1547,-1,0);*/

    
    //rviz_visual_tools::RvizVisualTools vt;

    const std::string PLANNING_GROUP = "panda_arm";
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
    const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

    geometry_msgs::Pose p;
    Eigen::Affine3d temp, rot;
    temp.translation() = Eigen::Vector3d(1,1,1);
    temp.linear().setIdentity();
    rot.linear().setIdentity();
    rot.rotate( Eigen::AngleAxis<double>(45, Eigen::Vector3d(1,0,0)));
  
    for (float alpha = 10; alpha < 360; alpha+=10)
    {
        temp.rotate( Eigen::AngleAxis<double>(alpha * M_PI/180, Eigen::Vector3d(0,1,0)));
  
        temp.rotate( rot.linear() );

        temp.linear().setIdentity();
        
        p= tf2::toMsg (temp);

            















                     
    }
