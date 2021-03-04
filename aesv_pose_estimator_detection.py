#!/usr/bin/env python
import rospy, numpy as np,math,matplotlib.pyplot as plt
from ParametersInitialization import *
from EKF_estimation_algorithm import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped,AckermannDrive
from tf.transformations import quaternion_from_euler, euler_from_quaternion

#### Declaring

class ekf_estimation():
    #### initialization
    def __init__(self):
        self.VEHICLE = 3 ## 1: Pioneer; 2: ACRONIS; 3: AESV
        self.pub = rospy.Publisher('ekf_estimated_Pose',Odometry)
        if self.VEHICLE == 3:
            rospy.Subscriber('cmd_vel',AckermannDriveStamped, self.aesv_cmd_vel_callback)
            rospy.Subscriber('global_odom',Odometry,self.odometry_callback)
        elif self.VEHICLE == 2:
            rospy.Subscriber('twist_cmd',TwistStamped, self.acronis_cmd_vel_callback)
            rospy.Subscriber('ndt_pose',PoseStamped,self.acronis_odometry_callback)
        elif self.VEHICLE == 1:
            rospy.Subscriber('RosAria/cmd_vel',Twist, self.pioneer_cmd_vel_callback)
            rospy.Subscriber('robot_to_map',Odometry,self.odometry_callback)
        else:
            print("Select Vehicle Correctly")
            
        self.init = 0
        self.detection_init = 0
        self.counter = 0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.position_x = list()
        self.position_y = list()
        self.estimated_x = list()
        self.estimated_y = list()
        self.mp = np.array([[1,0,0],[0,1,0],[0,0,0.3]])
        self.omega_tun_param = 1 * np.ones([3,3])
        self.G_tk = list()
        self.CUMSUM_S_tk = np.zeros([3,3])
        self.Threshold_param = 10.556303128934996e+23 
        self.time = 0
        self.seconds_time = list()
        self.residual1 = list()
        self.residual2 = list()
        self.residual3 = list()
        self.covMatrix = np.zeros([3,3])
        self.stack_residue = list()
        self.cumsum = list()
        self.vLidar = np.zeros(3)
        self.ekf = Odometry()
    
    #### velocity call back 
    def aesv_cmd_vel_callback(self,msg):
        self.linear_velocity = msg.drive.speed
        self.angular_velocity = msg.drive.steering_angle_velocity
    
    def acronis_cmd_vel_callback(self,msg):
        self.linear_velocity = msg.twist.linear.x
        self.angular_velocity = msg.twist.angular.z
    
    def pioneer_cmd_vel_callback(self,msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
    def acronis_odometry_callback(self,msg):
        [roll,pitch,yaw_z] = euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
        self.vLidar = np.array([[msg.pose.position.x,msg.pose.position.y,yaw_z]])
        
        if self.init==0:
            fig,self.axs = plt.subplots(2, 3)
            self.time = rospy.get_time()
            self.fnewxhat = self.vLidar[0][0]
            self.fnewyhat = self.vLidar[0][1]
            self.fnewthetahat = self.vLidar[0][2]
        self.init = 1
        [self.fnewxhat,self.fnewyhat,self.fnewthetahat,mnewP,fresidual] = EKF_Lidar(sParameter,self.fnewxhat,self.fnewyhat,self.fnewthetahat,self.linear_velocity,self.angular_velocity,np.transpose(self.vLidar),self.mp,1/30)
        self.sensor_detection(fresidual)
        if np.max(fresidual)==0:
            print("residual_zero",rospy.get_time() - self.time)
        else:
            self.store_values(msg.pose.position.x,msg.pose.position.y,fresidual[0][0],fresidual[1][0],fresidual[2][0])
        self.publish_ekf_estimator(msg.header.frame_id,msg.child_frame_id,msg.header.stamp,self.fnewxhat,self.fnewyhat,self.fnewthetahat)
        
    #### odometry call back
    def odometry_callback(self,msg):
        [roll,pitch,yaw_z] = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        self.vLidar = np.array([[msg.pose.pose.position.x,msg.pose.pose.position.y,yaw_z]])
        
        if self.init==0:
            fig,self.axs = plt.subplots(2, 3)
            self.time = rospy.get_time()
            self.fnewxhat = self.vLidar[0][0]
            self.fnewyhat = self.vLidar[0][1]
            self.fnewthetahat = self.vLidar[0][2]
        self.init = 1
        [self.fnewxhat,self.fnewyhat,self.fnewthetahat,mnewP,fresidual] = EKF_Lidar(sParameter,self.fnewxhat,self.fnewyhat,self.fnewthetahat,self.linear_velocity,self.angular_velocity,np.transpose(self.vLidar),self.mp,1/30)
        self.sensor_detection(fresidual)
        if np.max(fresidual)==0:
            print("residual_zero",rospy.get_time() - self.time)
        else:
            self.store_values(msg.pose.pose.position.x,msg.pose.pose.position.y,fresidual[0][0],fresidual[1][0],fresidual[2][0])
        self.publish_ekf_estimator(msg.header.frame_id,msg.child_frame_id,msg.header.stamp,self.fnewxhat,self.fnewyhat,self.fnewthetahat)
    
    #### Publishing Estimated values
    def publish_ekf_estimator(self,header,frame_id,stamp,xhat,yhat,thetahat):
        self.ekf.header.frame_id = header
        self.ekf.child_frame_id = frame_id
        self.ekf.header.stamp = stamp
        self.ekf.pose.pose.position.x = xhat;
        self.ekf.pose.pose.position.y = yhat;
        self.ekf.pose.pose.position.z = 0.0;
        ekf_quaternion = quaternion_from_euler(0, 0,thetahat)
        self.ekf.pose.pose.orientation.x = ekf_quaternion[0]
        self.ekf.pose.pose.orientation.y = ekf_quaternion[1]
        self.ekf.pose.pose.orientation.z = ekf_quaternion[2]
        self.ekf.pose.pose.orientation.w = ekf_quaternion[3]
        self.pub.publish(self.ekf)
       
    #### storing and plotting vaalues
    def store_values(self,x,y,fresidual00,fresidual10,fresidual20):
        self.position_x.append(x)
        self.position_y.append(y)
        self.estimated_x.append(self.fnewxhat)
        self.estimated_y.append(self.fnewyhat)
        self.seconds_time.append(rospy.get_time() - self.time)
        self.residual1.append(fresidual00)
        self.residual2.append(fresidual10)
        self.residual3.append(fresidual20)
        self.plot_x(self.residual1,self.residual2,self.residual3,self.seconds_time,self.position_x,self.position_y,self.estimated_x,self.estimated_y)
        plt.ion()
    
    #### plotting
    def plot_x(self,x1,x2,x3,t,SLAM_X,SLAM_Y,est_x,est_y):
        if self.counter % 10 == 0:
            self.axs[0,0].plot(t,x1,'tab:blue')
            self.axs[0,0].set_title('residuals blue1')
            
            self.axs[0,1].set_title('residuals red2')
            self.axs[0,1].plot(t,x2,'tab:red')
            
            self.axs[0,2].set_title('residuals green3')
            self.axs[0,2].plot(t,x3,'tab:green')
            
            self.axs[1,0].plot(SLAM_X,SLAM_Y,'tab:blue')
            self.axs[1,0].set_title('SLAM pose ')
            
            self.axs[1,1].plot(est_x,est_y,'tab:red')
            self.axs[1,1].set_title('Estimated pose ')
            
            self.axs[1,2].scatter(SLAM_X[-1]- est_x[-1],SLAM_Y[-1]-est_y[-1],marker='o')
            self.axs[1,2].set_title('error slam pose - estimated pose ')
            plt.draw()
            plt.pause(0.000000001)
        self.counter += 1
######detection part
    def sensor_detection(self,residual):
        if self.detection_init==0:
            stack = [residual[0][0],residual[1][0],residual[2][0]]
            self.stack_residue.insert(0,stack)
            self.detection_init = 1
            print("stack",self.stack_residue)
        elif self.detection_init == 1:
            stack = [residual[0][0],residual[1][0],residual[2][0]]
            self.stack_residue.insert(1,stack)
            self.detection_init = 2
            print("stack",self.stack_residue)
        elif self.detection_init == 2:
            stack = [residual[0][0],residual[1][0],residual[2][0]]
            self.stack_residue.insert(2,stack)
            self.detection_init = 3
            print("stack",self.stack_residue)
        elif self.detection_init == 3:
            self.detection_init = 3
            stack = [residual[0][0],residual[1][0],residual[2][0]]
            self.stack_residue.insert(3,stack)
            self.stack_residue.pop(0)
            print("stack",self.stack_residue)
            #### have 3 residue values to create a 3 X 3 covariance matrix
            self.covMatrix = np.cov(self.stack_residue,rowvar=False,bias=True)
            ##### trying inverse if not trying psuedo inverse
            try:
                inverse_covariance = np.linalg.inv(self.covMatrix)
            except:
                inverse_covariance = np.linalg.pinv(self.covMatrix)
            #### CREATING g(t k )
            first_mul=np.dot(np.transpose(residual),inverse_covariance)
            G_tk_white_residual = np.dot(first_mul,residual)
            self.G_tk.append(G_tk_white_residual[0][0])
            ###print("Gtk",G_tk_white_residual)
            self.CUMSUM_S_tk = np.ndarray.max((self.CUMSUM_S_tk + G_tk_white_residual)-abs(((sum(self.G_tk)/len(self.G_tk)))))
            # print("cumsum max value",self.CUMSUM_S_tk+ G_tk_white_residual)
            # print("Average",(sum(self.G_tk)/len(self.G_tk)))
            print("cu sum",self.CUMSUM_S_tk)
            self.cumsum.append(self.CUMSUM_S_tk)
            print("max value",max(self.cumsum))
            print("\n")
            #### alarm detection
            if self.CUMSUM_S_tk >= self.Threshold_param:
                print("Alarm")
                
            else:
                print("no Alarm")
                print("\n")
                print("\n")


if __name__ == "__main__":
    rospy.init_node('self.pub_estimation')
    try:
        ekf_estimation()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()