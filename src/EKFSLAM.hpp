#include "ros/ros.h"

//for image transport 
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"

//ros message types i need
#include "sensor_msgs/image_encodings.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"
#include "nav_msgs/Path.h"

//for rviz marker visualization support
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

// For transform support
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"
#include "tf/tf.h"

#include <boost/bind.hpp>
#include <map>
#include <Eigen/Dense>

#include <opencv2/core.hpp>
#include <aruco/aruco.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "utilities.hpp"

//one camera measurement
class CamMeasure{     
public:
    int id;
    float range;
    float theta;
};

//one odometry measurement
class OdomMeasure{     
public:
    Eigen::Vector3d delta;      //the difference between current time and last time         
    Eigen::Vector3d pose;       // odom pose of current time
};

class EKFSLAM{
public:
    EKFSLAM();
    ~EKFSLAM();

private:
    //the callback function for "image" topic
    void  imageCb(const sensor_msgs::ImageConstPtr& image);
    
    //apply EKF odometry update
    void updateAction(const OdomMeasure& measure);             
    //apply EKF measurement update
    void updateSensor(const CamMeasure& measure);               
    //initialize ekf slam
    void init_ekf_slam();       
    //get odometric pose
    bool getOdomPose(geometry_msgs::PoseStamped& odom_pose,double& x,double& y,double& yaw,const ros::Time& t);
    //get range-bearing mearsurement from Rvec and Tvec obtained from aruco detecting
    void getObservation(const aruco::Marker& m,CamMeasure& obs);

    void publishRosEllipseMarkers(double scale);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher aruco_marker_pub_;
    ros::Publisher pose_pub_;
    ros::Subscriber ground_truth_sub_;

    cv::Matx33d cameraMatrix_;
    cv::Matx<double,1,5> distorsionCoeff_;
    aruco::CameraParameters camera_;
    aruco::MarkerDetector detector_;

    //EKF state vector
    Eigen::VectorXd mu_;       
    //EKF covariance matrix         
    Eigen::MatrixXd sigma_;             

    //motion noise matrix
    Eigen::Matrix3d R;     
    //observation noise matrix             
    Eigen::Matrix2d Q;                 
    //the minimum value of travel distance and angle to update ekf
    double d_thresh,a_thresh;
    //flag to indicate whether ekf is initialized
    bool ekf_init;
    //flag to indicate if it is necessary to update ekf
    bool ekf_update;
    //the odom pose of last ekf update step since ekf initialization
    Eigen::Vector3d ekf_odom_pose;

    geometry_msgs::PoseStamped latest_odom_pose;

    //tf related
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfb;
    std::shared_ptr<tf2_ros::TransformListener> tfl;
    std::shared_ptr<tf2_ros::Buffer> tf;

    //frame id
    std::string base_frame_id;      //default: "base_link"
    std::string odom_frame_id;      //default: "odom"
    std::string global_frame_id;     //default: "world"

    //the correspondences variable of mearsurment landmark
    //aruco id --> ekf landmark id
    std::map<int,int> correspondences;  

    //Transformation matrix from camera to robot base_link
    Eigen::Matrix4d T_r_c_;

};

//class contruction
EKFSLAM::EKFSLAM()
    :private_nh_("~"),it_(nh_),base_frame_id("base_link"),odom_frame_id("odom"),global_frame_id("world")
{
    image_sub_ = it_.subscribe("/camera/rgb/image_raw",1,&EKFSLAM::imageCb,this);
    aruco_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("aruco_detected",2,true);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("ekf_slam_pose",2,true);

    //cameramatrix for gazebo turtlebot3 pi camera
    cameraMatrix_ = cv::Matx33d(530.4669406576809, 0.000000, 320.5, 
                        0.000000, 530.4669406576809, 240.5, 
                        0.000000, 0.000000, 1.000000);
    distorsionCoeff_ = cv::Matx<double,1,5>(0.0, 0.0, 0.0, 0.0, 0.0);
    camera_.setParams(cv::Mat(cameraMatrix_),cv::Mat(distorsionCoeff_),cv::Size(640,480));
    detector_.setDictionary("ARUCO_MIP_36h12");

    //initialize ekf slam 
    init_ekf_slam();

    d_thresh = 0.05; a_thresh = M_PI/36.0;
    ekf_update = false;

    //default R and Q
    R << 0.01,0,0,0,0.01,0,0,0,0.01;
    Q << 0.01,0,0,0.01;

    tfb.reset(new tf2_ros::TransformBroadcaster());
    tf.reset(new tf2_ros::Buffer());
    tfl.reset(new tf2_ros::TransformListener(*tf));

    //initialize tranfrom matrix from camera_link to base_link
    this->T_r_c_ << 0.0,0.0,1.0,0.076,
                    -1.0,0.0,0.0,0.0,
                    0.0,-1.0,0.0,0.093,
                    0.0,0.0,0.0,1.0;

    //for debug, visualize the detected aruco marker and plot axis or cubes over it
    cv::namedWindow("Image Window");
}

//initialize ekf slam
void EKFSLAM::init_ekf_slam()
{
    //the initial pose of robot is at (0,0,0) without covariance,perfectly exact;
    this->mu_.resize(3);
    this->mu_.setZero();
    this->sigma_.resize(3,3);
    this->sigma_.setZero();

    ekf_init = false;
}


bool EKFSLAM::getOdomPose(geometry_msgs::PoseStamped& odom_pose,double& x,double& y,double& yaw,const ros::Time& t)
{
    //Get the robot's pose
    geometry_msgs::PoseStamped ident;
    ident.header.frame_id=base_frame_id;
    ident.header.stamp = t;
    tf2::toMsg(tf2::Transform::getIdentity(),ident.pose);
    try{
        if(tf->canTransform(odom_frame_id,base_frame_id,t,ros::Duration(0.5)))
            this->tf->transform(ident,odom_pose,odom_frame_id);
    }
    catch(tf2::TransformException e){
        ROS_WARN("Failed to compute odom pose(%s)",e.what());
        return false;
    }
    x = odom_pose.pose.position.x;
    y = odom_pose.pose.position.y;
    yaw = tf2::getYaw(odom_pose.pose.orientation);
    
    return true;
}

void EKFSLAM::getObservation(const aruco::Marker& m,CamMeasure& obs)
{
    /***TODO:This method always failed, the range and theta are incorrect.**/
    /*NOW: use tf transformation to obtain result**/
    Eigen::Vector4d markerposeincam;
    markerposeincam <<m.Tvec.at<float>(0,0),m.Tvec.at<float>(1,0),m.Tvec.at<float>(2,0) , 1 ;
    Eigen::Vector4d markerposeinrobot = this->T_r_c_ * markerposeincam;
    // std::cout << "markerposeincam :" << std::endl << markerposeincam << std::endl;
    // std::cout << "T_r_c_ =  " << std::endl << this->T_r_c_ << std::endl;
    // std::cout << "markerposeinrobot: " << std::endl << markerposeinrobot << std::endl;
    double x = markerposeinrobot(0);
    double y = markerposeinrobot(1);
    obs.range = sqrt(x*x + y*y);
    obs.theta = normalize2(atan2(y,x));
    obs.id = m.id;
}

//apply EKF odometry update, pick from AMCL package,amcl_odom.cpp L167
void EKFSLAM::updateAction(const OdomMeasure& measure)
{
    //the odom pose of last time 
    Eigen::Vector3d old_pose = vector_sub(measure.pose,measure.delta);
    //Implement sample_motion_odometry(Prob Rob p136)
    double delta_rot1,delta_trans,delta_rot2;


    if(sqrt(measure.delta(1)*measure.delta(1)+measure.delta(0)*measure.delta(0)) < 0.01)
        delta_rot1 = 0.0;
    else
        delta_rot1 = angle_diff(atan2(measure.delta(1),measure.delta(0)),old_pose(2));
    
    delta_trans = sqrt(measure.delta(0)*measure.delta(0)+measure.delta(1)*measure.delta(1));
    delta_rot2 = angle_diff(measure.delta(2),delta_rot1);

    //apply odometry update to ekf pose
    this->mu_(0) += delta_trans*cos(this->mu_(2)+delta_rot1);
    this->mu_(1) += delta_trans*sin(this->mu_(2)+delta_rot1);
    this->mu_(2) += normalize2(delta_rot1 + delta_rot2);
    this->mu_(2) = normalize2(this->mu_(2));
    
    //compute the low-dimensional jacobian gt 
    Eigen::Matrix3d gt;
    gt(0,0) = 1;
    gt(0,1) = 0;
    gt(0,2) = -1*delta_trans*sin(this->mu_(2)+delta_rot1);
    gt(1,0) = 0;
    gt(1,1) = 1;
    gt(1,2) = delta_trans*cos(this->mu_(2)+delta_rot1);
    gt(2,0) = 0;
    gt(2,1) = 0;
    gt(2,2) = 1;

    //compute the high-dimensional jacobian Gt
    int N = this->mu_.rows();
    Eigen::MatrixXd Gt(N,N);
    Gt.setZero();
    Gt.block(0,0,3,3) = gt;
    //TODO: need to understand 
    Gt.bottomRightCorner(N-3,N-3).setIdentity();

    //update covariance matrix
    Eigen::MatrixXd Rt(N,N);
    Rt.setZero();
    Rt.block(0,0,3,3) = R;
    sigma_ = Gt*(sigma_)*Gt.transpose() + Rt;

}

//apply EKF SLAM measurement updation
//reference: Prob Rob P314
void EKFSLAM::updateSensor(const CamMeasure& measure)
{
    ROS_INFO("Update Sensor..");
    ROS_INFO("The Marker ID: %d",measure.id);
    std::cout << "The maker range = " << measure.range << ", theta = " << measure.theta << std::endl;
    //find the corresponding id of this measurement
    auto l = this->correspondences.find(measure.id);
    //the landmark have never seen before
    if(l == this->correspondences.end())
    {
        ROS_INFO("the landmark have never seen before.");
        // the corresponding id of landmark 
        int j = this->correspondences.size()+1;
        double mu_j_x = this->mu_(0) + measure.range*cos(normalize2(measure.theta+this->mu_(2)));
        double mu_j_y = this->mu_(1) + measure.range*sin(normalize2(measure.theta+this->mu_(2)));

        // std::cout << "mu_j_x = " << mu_j_x << std::endl;
        // std::cout << "mu_j_y = " << mu_j_y << std::endl;

        //extend state space
        this->mu_.conservativeResize(3+2*j);
        this->mu_(2*j+1) = mu_j_x;
        this->mu_(2*j+2) = mu_j_y;
        this->sigma_.conservativeResize(3+2*j,3+2*j);
        this->sigma_.rightCols(2).setZero();
        this->sigma_.bottomRows(2).setZero();
        this->sigma_.bottomRightCorner(2,2) = Eigen::Matrix2d::Identity()*1000;
        //add landmark to correspondence variables
        this->correspondences.insert(std::make_pair(measure.id,j));

        // std::cout << "this->mu_ = " << this->mu_.transpose() << std::endl;
        // std::cout << "this->sigma_ = " << std::endl << this->sigma_ << std::endl;
        std::cout << "Done" << std::endl;
    }
    //the landmark have already been seen 
    else
    {
        ROS_INFO("the landmark have seen before.");
        int j = l->second;
        double mu_j_x = this->mu_(2*j+1);
        double mu_j_y = this->mu_(2*j+2);
        // double delta_x = this->mu_(0) - mu_j_x;
        // double delta_y = this->mu_(1) - mu_j_y;
        double delta_x = mu_j_x - this->mu_(0);
        double delta_y = mu_j_y - this->mu_(1);
        Eigen::Vector2d delta;
        delta << delta_x,delta_y;
        double q = delta.transpose()*delta;
        Eigen::Vector2d z_hat;
        double angle = atan2(delta_y,delta_x)-this->mu_(2);
        double robottheta = mu_(2);
        z_hat << sqrt(q),normalize2(atan2(delta_y,delta_x)-this->mu_(2));
        std::size_t N = this->mu_.rows();
        Eigen::MatrixXd Ht(2,N);
        Ht.setZero();
        // Eigen::MatrixXd hx(2,3);
        // hx << -1/sqrt(q)*delta_x,-1/sqrt(q)*delta_y,0,
        //         1/q*delta_y, -1/q*delta_x,-1;
        // Eigen::MatrixXd hj(2,2);
        // hj << 1/sqrt(q)*delta_x,1/sqrt(q)*delta_y,
        //         -1/q*delta_y, 1/q*delta_x;
        // Ht.block(0,0,2,3) = hx;
        // Ht.block(0,2*j+1,2,2) = hj;
        Eigen::MatrixXd lowHt(2,5);
        lowHt << -sqrt(q)*delta_x,-sqrt(q)*delta_y,0,sqrt(q)*delta_x,sqrt(q)*delta_y,
                    delta_y,    -delta_x,      -q,      -delta_y,       delta_x;
        lowHt = lowHt / q;
        Ht.block(0,0,2,3) = lowHt.block(0,0,2,3);
        Ht.block(0,2*j+1,2,2) = lowHt.block(0,3,2,2);

        Eigen::MatrixXd St = Ht*sigma_*Ht.transpose()+Q;
        Eigen::MatrixXd Kt = sigma_*Ht.transpose()*St.inverse();
        Eigen::Vector2d z;
        z << measure.range,measure.theta;
        Eigen::Vector2d delta_z = z - z_hat;
        delta_z(1) = normalize2(delta_z(1));
        mu_ += Kt * delta_z;
        sigma_ = (Eigen::MatrixXd::Identity(N,N)-Kt*Ht)*sigma_;
        mu_(2) = normalize2(mu_(2));

        // std::cout << "this->mu_ = " << this->mu_.transpose() << std::endl;
        // std::cout << "this->sigma_ = " << std::endl << this->sigma_ << std::endl;
        std::cout << "Done" << std::endl;
    }
}


void EKFSLAM::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return;
    }

    auto markers = detector_.detect(cv_ptr->image,camera_,0.4); //0.0986
    for(auto m:markers){
        aruco::CvDrawingUtils::draw3dAxis(cv_ptr->image,m,camera_);
        aruco::CvDrawingUtils::draw3dCube(cv_ptr->image,m,camera_);
    }
    cv::imshow("Image Window",cv_ptr->image);
    cv::waitKey(3);

    //where was the robot now
    Eigen::Vector3d pose;
    if(!getOdomPose(latest_odom_pose,pose(0),pose(1),pose(2),msg->header.stamp)){
        ROS_ERROR("Couldn't determine robot's pose");
        return;
    }

    Eigen::Vector3d delta;
    delta.setZero();

    if(ekf_init)
    {
        //compute change in pose
        delta(0) = pose(0) - ekf_odom_pose(0);
        delta(1) = pose(1) - ekf_odom_pose(1);
        delta(2) = angle_diff(pose(2),ekf_odom_pose(2));

        //see if we should update the filter
        ekf_update = fabs(delta(0))>d_thresh || fabs(delta(1))>d_thresh ||
                        fabs(delta(2))>a_thresh;  
    }
    if(!ekf_init)
    {
        ekf_odom_pose = pose;
        ekf_init = true;
        ekf_update = true;
    }
    //If the robot has moved ,update the filter
    else if(ekf_init && ekf_update)
    {
        ROS_DEBUG("process update action");
        std::cout << "process update" << std::endl;
        OdomMeasure odata;
        odata.pose = pose;
        odata.delta = delta;
        updateAction(odata);

        for(auto m : markers)
        {
            CamMeasure cdata;
            getObservation(m,cdata);
            if(cdata.range < 3)     //apply EKF measurement update when range < 3
                updateSensor(cdata);
        }

        ekf_update = false;
        ekf_odom_pose = pose;


        //Publish ekf slam pose estimation
        geometry_msgs::PoseWithCovarianceStamped p;
        //Fill in the header
        p.header.frame_id=global_frame_id;
        p.header.stamp = msg->header.stamp;
        //Copy in the pose
        p.pose.pose.position.x = mu_(0);
        p.pose.pose.position.y = mu_(1);

        tf2::Quaternion q;
        q.setRPY(0,0,this->mu_(2));
        tf2::convert(q,p.pose.pose.orientation);
        //Copy in the covariance, converting from 3-D to 6-D
        for(int i=0;i<2;i++)
        {
            for(int j=0;j<2;j++)
            {
                p.pose.covariance[6*i+j] = this->sigma_(i,j);
            }
        }
        p.pose.covariance[6*5+5] = this->sigma_(2,2);
        
        pose_pub_.publish(p);
        publishRosEllipseMarkers(0.05);

    }

}


void EKFSLAM::publishRosEllipseMarkers(double scale)
{
    visualization_msgs::MarkerArray markers;
    for(int i = 4; i < mu_.rows(); i += 2)
    {
        double mx = mu_(i-1);
        double my = mu_(i);

        Eigen::Matrix2d sigma_m = sigma_.block(i-1,i-1,2,2);
        cv::Mat cvsigma_m = (cv::Mat_<double>(2,2) << sigma_m(0,0),sigma_m(0,1),sigma_m(1,0),sigma_m(1,1));
        cv::Mat eigen_value,eigen_vector;
        cv::eigen(cvsigma_m,eigen_value,eigen_vector);
        double angle = atan2(eigen_vector.at<double>(0,1),eigen_vector.at<double>(0,0));
        double x_len = 2 * sqrt(eigen_value.at<double>(0,0)*5.991);
        double y_len = 2 * sqrt(eigen_value.at<double>(1,0)*5.991);

        //DEBUG  TODO: delete after debug
        std::cout << "marker covariance: " << i << std::endl << sigma_m << std::endl; 

        //construct marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = global_frame_id;
        marker.header.stamp = ros::Time();
        marker.ns = "ekf_slam";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = mx;
        marker.pose.position.y = my;
        marker.pose.position.z = 0;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
        marker.scale.x = scale * x_len;
        marker.scale.y = scale * y_len;
        marker.scale.z = 0.1 * scale * (x_len + y_len);
        marker.color.a = 0.8;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        markers.markers.push_back(marker);
    } 
    aruco_marker_pub_.publish(markers);
}

EKFSLAM::~EKFSLAM()
{

}


