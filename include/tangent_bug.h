/* Authors: Jingru Feng, South China University of Technology */
/* Contact: feng_jingru@foxmail.com*/

#ifndef TURTLEBOT3_DRIVE_H_
#define TURTLEBOT3_DRIVE_H_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2

#define LINEAR_VELOCITY  0.3
#define ANGULAR_VELOCITY 1.5

//#define GET_TB3_DIRECTION 0
//#define TB3_DRIVE_FORWARD 1
//#define TB3_RIGHT_TURN    2
//#define TB3_LEFT_TURN     3
/*******************************************************************************
* Position
******************************************************************************/
class pose_xy{
 public:
  double x;
  double y;
  double heading;


	static double getAngularDistance(pose_xy& p1, pose_xy& p2) 
	{ 
		return atan2(p2.y - p1.y, p2.x - p1.x) - p1.heading; 
	} 

	static double getLinearDistance(pose_xy& p1, pose_xy& p2)  
	{ 
	        return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2)); 
	}

	pose_xy(double xp = 0, double yp = 0, double hp = 0)
	{
		x = xp;
		y = yp;
		heading = hp;
	}

};
class Turtlebot3Drive
{
 public:
  Turtlebot3Drive();
  ~Turtlebot3Drive();
  bool init();
  int controlLoop();
  void move(double xvel,double yvel);
  bool obstacle_in_path(pose_xy target, pose_xy robot);
  pose_xy laserIndexToPose(int i, double dist, double heading, pose_xy robott);
  pose_xy findClosestTangentPoint(pose_xy goal, pose_xy robot_pose);
  pose_xy getRobotGoalVector(pose_xy p1, pose_xy p2);

  pose_xy robot;
  pose_xy target;
  pose_xy Oi;//the subgoal
  pose_xy goalRobotV;
  double dist;//queristic dist
  double last_dist = HUGE_VAL;
  double dReached, dFollow;
  pose_xy dFollowp;

  sensor_msgs::LaserScan laser;
  float laserdata[360];

  float laser_angle_increment;
  float laser_angle_min;
  float laser_angle_max;

  float laser_range_min;
  float laser_range_max;
 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Parameters

  // ROS Time

  // ROS Topic Publishers
  ros::Publisher cmd_vel_pub_;

  // ROS Topic Subscribers
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber odom_sub_;
  //ros::Subscriber pose_sub_;

  // Variables
  double escape_range_;
  double check_forward_dist_;
  double check_side_dist_;

  //double scan_data_[3] = {0.0, 0.0, 0.0};

  double tb3_pose_;
  double prev_tb3_pose_;

  // Function prototypes
  void updatecommandVelocity(double linear, double angular);
  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
};
#endif // TURTLEBOT3_DRIVE_H_
