#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"

nav_msgs::Path odomPath;
nav_msgs::Path slamPath;
nav_msgs::Path refPath;

void odomCb(const nav_msgs::OdometryConstPtr& odomPose)
{
    geometry_msgs::PoseStamped p;
    p.header.frame_id = odomPose->header.frame_id;
    p.header.stamp = odomPose->header.stamp;
    p.pose = odomPose->pose.pose;
    odomPath.poses.push_back(p);
}
void slamCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& slamPose)
{
    geometry_msgs::PoseStamped p;
    p.header.frame_id = slamPose->header.frame_id;
    p.header.stamp = slamPose->header.stamp;
    p.pose = slamPose->pose.pose;
    slamPath.poses.push_back(p);
}
void refCb(const nav_msgs::OdometryConstPtr& refPose)
{
    geometry_msgs::PoseStamped p;
    p.header.frame_id = refPose->header.frame_id;
    p.header.stamp = refPose->header.stamp;
    p.pose = refPose->pose.pose;
    refPath.poses.push_back(p);

}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"visualize_traj_node");
    ros::NodeHandle nh;

    ROS_INFO("Start visualize_traj_node ....");

    ros::Subscriber odomPoseSub = nh.subscribe("odom",1000,odomCb);
    ros::Subscriber slamPoseSub = nh.subscribe("ekf_slam_pose",1000,slamCb);
    ros::Subscriber refPoseSub = nh.subscribe("ground_truth/state",1000,refCb);

    ros::Publisher odomTrajPub = nh.advertise<nav_msgs::Path>("odom_trajectory",1000);
    ros::Publisher slamTrajPub = nh.advertise<nav_msgs::Path>("slam_trajectory",1000);
    ros::Publisher refTrajPub= nh.advertise<nav_msgs::Path>("ref_trajectory",1000);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        odomPath.header.frame_id = "odom";
        slamPath.header.frame_id = "world";
        refPath.header.frame_id = "world";
        odomPath.header.stamp = ros::Time::now();
        slamPath.header.stamp = ros::Time::now();
        refPath.header.stamp = ros::Time::now();
        odomTrajPub.publish(odomPath);
        slamTrajPub.publish(slamPath);
        refTrajPub.publish(refPath);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}