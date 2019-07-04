#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <math.h>

rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yuanzhui");
    ros::NodeHandle n;
    //ros::Publisher point_pub = n.advertise<geometry_msgs::Pose>("points", 1000);

    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("/base_link", "/rviz_visual_markers"));   
    visual_tools_->deleteAllMarkers();


    Eigen::Affine3d temp,rot,rot1,rot2;
    
    temp.linear().setIdentity();
   

    temp.translation() = Eigen::Vector3d(0.5, 0.5, 0.5);
    temp.rotate(Eigen::AngleAxis<double>(45 * M_PI / 180, Eigen::Vector3d(1, 0, 0)));
    
    temp.translation() = Eigen::Vector3d(0.3, 0.3, 0.3);
    // visual_tools_->trigger();

    // rot1.linear().setIdentity();
    // rot1.rotate(Eigen::AngleAxis<double>(45 * M_PI / 180, Eigen::Vector3d(0, 0, 1)));
    // rot2.linear().setIdentity();
    // rot2.rotate(Eigen::AngleAxis<double>(45 * M_PI / 180, Eigen::Vector3d(0, 1, 0)));
  


    // temp.linear().setIdentity();
    // temp.translation() = Eigen::Vector3d(0.5, 0.5, 0.5);
    // temp.prerotate(rot1.linear());
    // temp.prerotate(rot2.linear());
    
    // visual_tools_->publishAxisLabeled(temp, "temp_third");
    // visual_tools_->trigger();
    
    rot.linear().setIdentity();
    rot.rotate(Eigen::AngleAxis<double>(10* M_PI / 180, Eigen::Vector3d(0, 1, 0)));
  
    float r = 0;
    for (float alpha = 0; alpha <360; alpha += 10)
    {
        temp.translation() = Eigen::Vector3d(0.3, 0.3+r*cos(alpha * M_PI/180), 0.3+r*sin(alpha * M_PI/180));
        temp.prerotate(rot.linear());
        visual_tools_->publishAxis(temp);
        visual_tools_->trigger();
       
        //temp.linear().setIdentity();
        //visual_tools_->publishAxis(temp);
        //visual_tools_->trigger();
        ros::Duration(0.5).sleep();
      
        /*if (alpha==350)
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
         
        }*/
    }
    ros::spin();

    
}
