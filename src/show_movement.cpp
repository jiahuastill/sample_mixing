#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <math.h>

rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "show_movement");
    ros::NodeHandle n;
    ros::Publisher point_pub = n.advertise<geometry_msgs::Pose>("points", 1000);

    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("/base_link", "/rviz_visual_markers"));

    /*geometry_msgs::Pose pose1;

    tf2::Quaternion orientation1;
    orientation1.setRPY(-135* M_PI / 180, 0, -90* M_PI / 180);

    pose1.position.x = 0.35;
    pose1.position.y =0.17999;
    pose1.position.z =0.62253 ;
    pose1.orientation = tf2::toMsg(orientation1);
    visual_tools_->publishAxis(pose1);
    visual_tools_->trigger();
    

    geometry_msgs::Pose pose2;

    tf2::Quaternion orientation2;
    orientation2.setRPY(-90* M_PI / 180, 0, -135* M_PI / 180);

    pose2.position.x = 0.35;
    pose2.position.y =0.22242;
    pose2.position.z =0.58005 ;
    pose2.orientation = tf2::toMsg(orientation2);
    visual_tools_->publishAxis(pose2);
    visual_tools_->trigger();

    geometry_msgs::Pose pose3;

    tf2::Quaternion orientation3;
    orientation3.setRPY(-45* M_PI / 180, 0, -90* M_PI / 180);

    pose3.position.x = 0.35;
    pose3.position.y =0.18;
    pose3.position.z =0.53747 ;
    pose3.orientation = tf2::toMsg(orientation3);
    visual_tools_->publishAxis(pose3);
    visual_tools_->trigger();

    geometry_msgs::Pose pose4;

    tf2::Quaternion orientation4;
    orientation4.setRPY(-90* M_PI / 180, 0, -45* M_PI / 180);

    pose4.position.x = 0.35;
    pose4.position.y =0.13762;
    pose4.position.z =0.58004 ;
    pose4.orientation = tf2::toMsg(orientation4);
    visual_tools_->publishAxis(pose4);
    visual_tools_->trigger();*/

    /*Eigen::Affine3d temp, rot;
    temp.translation() = Eigen::Vector3d(1, 1, 1);
    temp.linear().setIdentity();
    rot.linear().setIdentity();

    rot.rotate(Eigen::AngleAxis<double>(90, Eigen::Vector3d(0, 1, 0)));
    for (float alpha = 10; alpha < 360; alpha += 10)
    {
        temp.rotate(Eigen::AngleAxis<double>(alpha * M_PI / 180, Eigen::Vector3d(0, 1, 0)));
        
        temp.rotate(rot.linear());

        temp.linear().setIdentity();
    }*/

    /* Eigen::Affine3d temp;
    temp.translation() = Eigen::Vector3d(0.3, 0.3, 0.3);
    temp.linear().setIdentity();
    for (float alpha = 90; alpha <= 360; alpha += 90)
    {
        temp.rotate(Eigen::AngleAxis<double>(alpha * M_PI / 180, Eigen::Vector3d(1, 0, 0)));

        temp.linear().setIdentity();*/

    // Delete all markers
    
    visual_tools_->deleteAllMarkers();

    geometry_msgs::Pose p1, temp_p;
    tf2::Quaternion orientation_p1;
    orientation_p1.setRPY(-90 * M_PI / 180, 90 * M_PI / 180, 0);
    p1.position.x = 0.3;
    p1.position.y = 0.3;
    p1.position.z = 0.3;
    p1.orientation = tf2::toMsg(orientation_p1);
    



    Eigen::Affine3d temp;
    
    tf2::fromMsg(p1, temp);
    visual_tools_->publishAxisLabeled(temp, "Temp original");
    visual_tools_->trigger();
    // Ger rotation matrix representing rotation in Y axis.
   // rot.linear().setIdentity();
   // rot.rotate(Eigen::AngleAxis<double>(90* M_PI / 180, Eigen::Vector3d(0, 0, 1)));

    // Initialize Temp
    //temp.translation() = Eigen::Vector3d(0, 0.1, 0);
    //temp.linear().setIdentity();
    //temp.rotate(Eigen::AngleAxis<double>(45* M_PI / 180, Eigen::Vector3d(0, 0, 1)));
    //visual_tools_->publishAxisLabeled(temp, "Temp original");
    //visual_tools_->trigger();
    
    
    float r = 0.4;
    
   for (float alpha = 10; alpha <360; alpha += 10)
    {
        temp.rotate(Eigen::AngleAxis<double>(alpha * M_PI / 180, Eigen::Vector3d(0, 0, 1)));
        //temp.rotate(rot.linear());
        temp.translation() = Eigen::Vector3d(0.3-r - r * cos((alpha-180) * M_PI / 180), 0.3 + r * sin((alpha-180) * M_PI / 180) , 0.3 + r * sin((alpha-180) * M_PI / 180));
       
        visual_tools_->publishAxisLabeled(temp, std::to_string(alpha));
        visual_tools_->trigger();
        temp_p = tf2::toMsg(temp);
        point_pub.publish(temp_p);
        temp.rotate(Eigen::AngleAxis<double>(-alpha * M_PI / 180, Eigen::Vector3d(0, 0, 1)));
       
        //temp.linear().setIdentity();
        //visual_tools_->publishAxis(temp);
        //visual_tools_->trigger();
        ros::Duration(0.5).sleep();
        if (alpha==350)
        {
            for(float beta=10; beta<=360; beta+=10 )
            {
                temp.rotate(Eigen::AngleAxis<double>(-(alpha+beta) * M_PI / 180, Eigen::Vector3d(0, 0, 1)));
                temp.translation() = Eigen::Vector3d(0.3+r - r * cos((360-alpha-beta) * M_PI / 180), 0.3  - r * sin((360-alpha-beta) * M_PI / 180), 0.3  + r * sin((360-alpha-beta) * M_PI / 180));
                visual_tools_->publishAxisLabeled(temp, std::to_string(alpha));
                visual_tools_->trigger();
                temp.rotate(Eigen::AngleAxis<double>((alpha+beta) * M_PI / 180, Eigen::Vector3d(0, 0, 1)));
                ros::Duration(0.5).sleep();
            }
         
        }
    }

    ros::Rate loop_rate(10);
    ros::spinOnce();

    loop_rate.sleep();
    
}
