#include "EKFSLAM.hpp"

int main(int argc, char** argv)
{
    /*ros initialization*/
    ros::init(argc,argv,"EkfSlamNode");
    ros::NodeHandle n;
    ROS_INFO("start EKF SlAM Node.....");
    
    EKFSLAM ekfSlam;

    ros::spin();
    
    return 0;
}