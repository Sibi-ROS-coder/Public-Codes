#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include <sstream>
#include "cv_bridge/cv_bridge.h"
#include "nav_msgs/Odometry.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>

#include <image_transport/image_transport.h>
#include "tf/transform_datatypes.h"
#include <math.h>
#include <cmath>
#include <ros/console.h>

double x=0.0,y=0.0,theta=0.0,init_x=0.0,init_y=0.0,init_theta=0.0,X_Rob=0.0,Y_Rob=0.0,angle=0.0;
int init=1,first_run=0,finish=0;
tf::Transform TR_init;
void newOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
// Declaration  
// Saving initial pose as reference frame
  if (init==1)
  {
    TR_init.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    TR_init.setRotation(tf::Quaternion( msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
    init=0;
    //std::cout << "Roll: " << (TR_init.getOrigin()).getX();
  }

// Creating relative pose WRT ref frame
  if (init==0)
  {
    tf::Transform TR;
    tf::Transform TR_new;
    tf::Quaternion R_new;
    TR.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    TR.setRotation(tf::Quaternion( msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
    TR_new=TR_init.inverseTimes(TR);
    x=(TR_new.getOrigin()).getX();
    y=(TR_new.getOrigin()).getY();
    R_new=TR_new.getRotation();

    tf::Matrix3x3 m(R_new);
    tf::Pose init_pose;
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    theta=yaw*(180/3.1415926535);
    // ROS_INFO("Position-> x: [%f], y: [%f], Theta: [%f]", x,y,theta);

  }
}

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
// Declaration  
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat image;
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f> > markerCorners, rejectedCandidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  int centre_h=522, centre_w=948;

// Finding aruco marker orientation thereby finding robot pose WRT marker
  image=cv_bridge::toCvShare(msg, "bgr8")->image;
  cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
  if (markerCorners.size()>0)
  {
    std::vector<cv::Point2f> corners=markerCorners[0];
    cv::Point2f topLeft,topRight, bottomRight, bottomLeft,aruco_center;
    topLeft=corners[0];topRight=corners[1]; bottomRight=corners[2];bottomLeft=corners[3];
    int centre_h=522, centre_w=948;

    aruco_center.x = int((topLeft.x + bottomRight.x) / 2.0);
    aruco_center.y = int((topLeft.y + bottomRight.y) / 2.0);
    cv::circle( image,aruco_center,4,cv::Scalar( 0, 0, 255 ),cv::FILLED,cv::LINE_8 );
    cv::line(image,topLeft,topRight,cv::Scalar( 0, 255, 0),2,cv::LINE_8);

    Y_Rob=(centre_w-aruco_center.x)*0.004065;
    X_Rob=-(centre_h-aruco_center.y)*0.004065;
    int direction_x=(topRight.x-bottomRight.x),direction_y=-(topRight.y-bottomRight.y);
    angle=(atan2(direction_x,direction_y))*(180/3.1415926535);
    cv::imshow("view", image);
    cv::waitKey(3);
    finish=1;
  }
  else if (finish==0)
  {
    Y_Rob=0;X_Rob=0;angle=0;
    finish=2;
  }

// Image view
  cv::imshow("view", image);
  cv::waitKey(3);
}

int main(int argc, char **argv)
{
// Declarations
  ros::init(argc, argv, "detect_aruco_move");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  geometry_msgs::Twist speed;
  double goal_x, goal_y, goal_angle,angle_to_goal,distance;
  float max_lin_vel,max_ang_vel,inc_x,inc_y;
  int direction;
  

// Publisher and subscriber
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  image_transport::Subscriber image_sub = it.subscribe("/camera/rgb/image_raw", 1, image_callback);
  ros::Subscriber sub = n.subscribe("/odom", 1000, newOdom);
  ROS_INFO("Using Cpp");

  ros::Rate loop_rate(4);

  ros::Time start_time ;
  ros::Duration stop_time;
  while (ros::ok())
  {
// Checking marker detection success status and initial robot pose saved status
    if (init==0 and finish==1)
    {
      if (first_run==0)
      {
        // One time value assignment
        start_time=ros::Time::now();
        goal_x=X_Rob;
        goal_y=Y_Rob;
        goal_angle=angle;
        max_lin_vel=0.2;
        max_ang_vel=0.2;
        first_run=1;
        ROS_INFO("Robot Pose To marker [%f],[%f],[%f]",X_Rob,Y_Rob,angle);
      }
      inc_x = goal_x -x;
      inc_y = goal_y -y;  
      angle_to_goal=(atan2(inc_y,inc_x))*(180/3.1415926535);
      distance=pow((pow(inc_x,2)+pow(inc_y,2)),0.5);
      
      if (std::abs(angle_to_goal - theta) > 5 and (distance)>0.012)
      { //ROS_INFO("heading angle %f,%f,%f",theta,distance,(angle_to_goal - theta)/std::abs(angle_to_goal - theta));
        direction=(distance<0.06)?1:((angle_to_goal - theta)/std::abs(angle_to_goal - theta)) ;
        speed.linear.x = 0.0;
        speed.angular.z =((((3.1415926535/180)*(std::abs(angle_to_goal - theta))*0.7)>max_ang_vel) ?max_ang_vel: ((3.1415926535/180)*(std::abs(angle_to_goal - theta))*0.7))*direction;}
      else if (distance>0.01)
        {//ROS_INFO("heading straight %f,%f",theta,distance);
        speed.linear.x = ((0.5*distance)>max_lin_vel)?max_lin_vel:(0.5*distance);
        speed.angular.z = 0.0;}
      else if (distance<0.01 and std::abs(goal_angle-theta) > 1)
        {//ROS_INFO("correcting angle %f,%f",theta,distance);
        direction=(goal_angle - theta==0)?1:((goal_angle - theta)/std::abs(goal_angle - theta));
        speed.linear.x = 0.0;
        speed.angular.z =((((3.1415926535/180)*(std::abs(goal_angle - theta))*0.7)>max_ang_vel) ?max_ang_vel: ((3.1415926535/180)*(std::abs(goal_angle - theta))*0.7));}//*direction
      else if (distance<0.01 and std::abs(goal_angle-theta) < 1)
        {ROS_INFO("Robot Error %f,%f,%f,%f",std::abs(goal_angle-theta),std::abs(goal_x-x),std::abs(goal_y-y),distance);
        speed.linear.x = 0.0;
        speed.angular.z = 0.0;
        pub.publish(speed);
        stop_time = ros::Time::now() - start_time;
        ROS_INFO("Robot Running time: %f",(stop_time).toSec());
        ROS_INFO("Goal Reached, Shutting down");
        break;}      
    }
	  else if (finish==2)
    {
      ROS_ERROR("Unable to detect marker");
      ROS_INFO("Shutting down");
      speed.linear.x = 0.0;
      speed.angular.z = 0.0;
      pub.publish(speed);
      break;
    }
	  else
    {
      speed.linear.x = 0.0;
		  speed.angular.z = 0.0;
    }
		// ROS_INFO("velocities are %f,%f",speed.linear.x,speed.angular.z);
    pub.publish(speed);

    ros::spinOnce();

    loop_rate.sleep();
   
  }

  return 0;
}