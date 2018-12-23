#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <boost/bind.hpp>

nav_msgs::Path odomPath;
nav_msgs::Path slamPath;
nav_msgs::Path refPath;

void callback(const nav_msgs::OdometryConstPtr& odomPose,const nav_msgs::OdometryConstPtr& refPose,const geometry_msgs::PoseWithCovarianceStampedConstPtr& slamPose)
{
    geometry_msgs::PoseStamped p1;
    p1.header.frame_id = odomPose->header.frame_id;
    p1.header.stamp = odomPose->header.stamp;
    p1.pose = odomPose->pose.pose;
    odomPath.poses.push_back(p1);

    geometry_msgs::PoseStamped p2;
    p2.header.frame_id = slamPose->header.frame_id;
    p2.header.stamp = slamPose->header.stamp;
    p2.pose = slamPose->pose.pose;
    slamPath.poses.push_back(p2);

    geometry_msgs::PoseStamped p3;
    p3.header.frame_id = refPose->header.frame_id;
    p3.header.stamp = refPose->header.stamp;
    p3.pose = refPose->pose.pose;
    refPath.poses.push_back(p3);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"Visualize_Traj_Node_2");
    ros::NodeHandle n;

    message_filters::Subscriber<nav_msgs::Odometry> odom_pose_sub(n,"odom",1);
    message_filters::Subscriber<nav_msgs::Odometry> ref_pose_sub(n,"ground_truth/state",1);
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> slam_pose_sub(n,"ekf_slam_pose",1);

    message_filters::TimeSynchronizer<nav_msgs::Odometry,nav_msgs::Odometry,geometry_msgs::PoseWithCovarianceStamped> sync(odom_pose_sub,ref_pose_sub,slam_pose_sub,10);
    sync.registerCallback(boost::bind(&callback,_1,_2,_3));

    ros::Publisher odomTrajPub = n.advertise<nav_msgs::Path>("odom_trajectory",1000);
    ros::Publisher slamTrajPub = n.advertise<nav_msgs::Path>("slam_trajectory",1000);
    ros::Publisher refTrajPub= n.advertise<nav_msgs::Path>("ref_trajectory",1000);

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