#include "ros/ros.h"
#include <stdio.h>
#include "std_msgs/String.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/transform_broadcaster.h>
#include <opencv2/calib3d.hpp>

using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;
using namespace cv_bridge;

void callback(const ImageConstPtr& img, const CameraInfoConstPtr& cam_info, const nav_msgs::OdometryConstPtr& odom)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat image;
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f> > corners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    image = cv_bridge::toCvShare(img, "bgr8")->image;
    cv::aruco::detectMarkers(image, dictionary, corners, markerIds, parameters, rejectedCandidates);
    cv::aruco::drawDetectedMarkers(image, corners, markerIds);
    
    // // // pose estimation
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::Mat_<double> cameraMatrix(3,3); // rows, cols
    cv::Mat_<double> distCoeffs(1,5);
    cameraMatrix << cam_info->K[0], cam_info->K[1], cam_info->K[2],
                    cam_info->K[3], cam_info->K[4], cam_info->K[5],
                    cam_info->K[6], cam_info->K[7], cam_info->K[8];
    distCoeffs << cam_info->D[0], cam_info->D[1], cam_info->D[2], cam_info->D[3], cam_info->D[4];
    cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
    // for(int i=0; i<markerIds.size(); i++)
    //         cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
    // cv::imshow("view", image);
    // cv::waitKey(3);
    cv::Mat R1;
    cv::Rodrigues(rvecs[0], R1);
    std::cout<<R1<<"R1";
    // // // tf publishing
    // std::cout<<"\n"<<rvecs[0]<<"\n";
    // tf::Matrix3x3 quat_matrix()
    // static tf::TransformBroadcaster br;
    // tf::Transform transform;
    // transform.setOrigin( tf::Vector3(tvecs[0][0],tvecs[0][1],tvecs[0][2]) );
    // transform.setRotation( tf::Quaternion(msg->theta, 0, 0) );
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"marker_frame","base_footprint"));
    }

int main(int argc, char** argv )
{
    ros::init(argc, argv, "detect");
    ros::NodeHandle nh;
    message_filters::Subscriber<Image> camera_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<CameraInfo> info_sub(nh, "/camera/rgb/camera_info", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/odom", 1);
    typedef sync_policies::ApproximateTime<Image, CameraInfo, nav_msgs::Odometry> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(200), camera_sub, info_sub, odom_sub);

    sync.registerCallback(boost::bind(&callback,_1, _2,_3));
    ros::spin();
    return 0;
}