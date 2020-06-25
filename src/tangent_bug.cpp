/* Authors: Jingru Feng, South China University of Technology */
/* Contact: feng_jingru@foxmail.com*/
/* This program implemented the Tangent Bug algorithm*/

#include "turtlebot3_gazebo/tangent_bug.h"

Turtlebot3Drive::Turtlebot3Drive()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("TurtleBot3 Simulation Node Init");
  ROS_ASSERT(init());
}

Turtlebot3Drive::~Turtlebot3Drive()
{
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool Turtlebot3Drive::init()
{
  // initialize ROS parameter
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");

  // initialize variables
  escape_range_       = 30.0 * DEG2RAD;
  check_forward_dist_ = 0.7;
  check_side_dist_    = 0.6;

  tb3_pose_ = 0.0;
  prev_tb3_pose_ = 0.0;

  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("scan", 10, &Turtlebot3Drive::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &Turtlebot3Drive::odomMsgCallBack, this);

  return true;
}

void Turtlebot3Drive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
	robot.x = msg->pose.pose.position.x; 
        robot.y = msg->pose.pose.position.y;
	double roll, pitch;
        std::cout<<"ready?";
	//printf("Getting position!!!! %f %f\n", robot.x, robot.y);
tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, 
                                          msg->pose.pose.orientation.y, 
                                          msg->pose.pose.orientation.z, 
                                          msg->pose.pose.orientation.w); 
        tf::Matrix3x3(q).getRPY(roll, pitch, robot.heading);
	std::cout<<"Getting heading!!!!  "<<robot.heading<<std::endl;
}
void Turtlebot3Drive::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  laser = *msg;
std::cout<<"laser updated"<<std::endl;
//std::cout<<"range_min"<<laser.range_min<<std::endl;//0.12
//std::cout<<"range_max"<<laser.range_max<<std::endl;//3.5
//std::cout<<"ranges"<<sizeof(laser.ranges)<<std::endl;
  for(int i = 0; i< 360; i++)
  {
	laserdata[i] = laser.ranges[i];
	//std::cout<<laserdata[i];
  }
  laser_angle_increment = laser.angle_increment;
  laser_angle_min = laser.angle_min;
  laser_angle_max = laser.angle_max;

  laser_range_min = laser.range_min;
  laser_range_max = laser.range_max;
}
void Turtlebot3Drive::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}

void Turtlebot3Drive::move(double xvel, double yvel)
{
  geometry_msgs::Twist msg;
  double v, w;
std::cout<<"To ("<<xvel<<", "<<yvel<<") "<<std::endl;
	//if(sqrt(pow(robot.x - xvel, 2) + pow(robot.y - yvel, 2)) <0.1)//error 0.1
	//{
		double angVelz;// = atan2(yvel-robot.y, xvel-robot.x) - robot.heading; 
	
		double distance = sqrt(xvel*xvel + yvel*yvel); 
	
		angVelz = atan2(yvel-robot.y, xvel-robot.x) - robot.heading;
		//std::cout<<"atan2: "<<atan2(yvel-robot.y, xvel-robot.x)<<std::endl;
		//std::cout<<"angdiff: "<<angVelz<<std::endl;
		std::cout<<"robot.heading: "<<robot.heading<<std::endl;
		if (std::abs(angVelz)> 0.1){ 
		    v = 0;
		    w = 0.6*angVelz; 
std::cout<<"turning"<<std::endl;
		} 
		else { 
		    v = 0.3;
		    w = 0; 
std::cout<<"walking"<<std::endl;
		}
	//}
	//else 
	//{
		//updatecommandVelocity(0, 0);
	//}	

        //printf("\ndx %f dy %f --- v %f w %f\n", xvel, yvel, v, w);

        msg.linear.x = v; 
        msg.angular.z = w; 
	cmd_vel_pub_.publish(msg);
	
}
bool Turtlebot3Drive::obstacle_in_path(pose_xy targett, pose_xy robott)
{
  bool right = false;
  float phi = atan2(targett.y - robott.y, targett.x - robott.x);
  //std::cout<<"y: "<<targett.y - robott.y<<" x: "<<targett.x - robott.x<<std::endl;
  if (robot.heading > 0)
  {
	if(phi < robot.heading && phi < robot.heading - 3.1415926)
	{	right = true;
		std::cout<<"Right!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
	}
  }
  else
  {
	if((phi < 0 && phi > robot.heading) || (phi>0 && phi<3.1415926 - std::abs(robot.heading))){}//Do nothing
	else
	{	right = true;
		std::cout<<"Right!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
	}
  }		
  float rho = robott.heading - phi;
  //float ang = -laser_angle_min - rho;
  int index = std::abs(rho) / laser_angle_increment;//appear negative nm, so I add an abs

  if(right && index < 180)
	{index = index + 180;}
//std::cout<<"index: "<<index<<std::endl;}
  else if(!right && index>180)
	{index = index - 180;}
//std::cout<<"index: "<<index<<std::endl;}
//std::cout<<"laser_angle_min: "<<laser_angle_min<<std::endl;
//std::cout<<"phi: "<<phi<<std::endl;
//std::cout<<"rho: "<<rho<<std::endl;
//std::cout<<"ang: "<<ang<<std::endl;
std::cout<<"index: "<<index<<"  "<<laserdata[index]<<std::endl;

  float dist = sqrt(pow(targett.x - robott.x, 2) + pow(targett.y - robott.y, 2));
  if(laserdata[index] > dist)
  {
  	return false;
  }
  else
  {
  	return true;
  }
}
pose_xy Turtlebot3Drive::laserIndexToPose(int i, double dist, double heading, pose_xy robott)
{
  double ang_point = heading + laser_angle_increment * i;
  //dist = dist - 0.3;
  pose_xy point;
  point.x = robott.x + dist*cos(ang_point);
  point.y = robott.y + dist*sin(ang_point);
  point.heading = ang_point;
  //double ang_point = heading - (laser_angle_increment * i + laser_angle_min);

  return point;
}
/*
*******
Find Oi, bug highly happened//closest-> min
*******/
pose_xy Turtlebot3Drive::findClosestTangentPoint(pose_xy targett, pose_xy robott)
{
  float hDist;
  float last_hDist;
  double ang_point, dist = HUGE_VAL;
  double minDist = HUGE_VAL;
  pose_xy minPoint, point;
  pose_xy p1,p2;
  bool onObstacle = false;
  double d1,d2;
  /*for(int i = 0;i<360;i++)
  {
	point = laserIndexToPose(i, laserdata[i], robott.heading, robott);
	dist = pose_xy::getLinearDistance(robot, point);
	std::cout<<i<<": "<<std::endl<<"distance to robot: "<<dist<<std::endl;
  	std::cout<<"x: "<<point.x<<" y: "<<point.y<<std::endl;
  }*/

  if(std::abs(laserdata[359]-laserdata[1])>0.7)//select one from 359 and 1 degree
  {
	p1 = laserIndexToPose(0, laserdata[0], robott.heading, robott);
//std::cout<<"p1  x: "<<p1.x<<" y: "<<p1.y<<std::endl;
	d1 = pose_xy::getLinearDistance(targett, p1) +  pose_xy::getLinearDistance(p1, robott);
	p2 = laserIndexToPose(359, laserdata[359], robott.heading, robott);
//std::cout<<"p2  x: "<<p2.x<<" y: "<<p2.y<<std::endl;
	d2 = pose_xy::getLinearDistance(targett, p2) +  pose_xy::getLinearDistance(p2, robott);
	if(d1<d2)
	{
		minPoint = p1;
		last_hDist = d1;
	}
	else
	{
		minPoint = p2;
		last_hDist = d2;
	}
//std::cout<<"minP from p1 and p1  x: "<<minPoint.x<<" y: "<<minPoint.y<<std::endl;
  }
  else
  {
	last_hDist = HUGE_VAL;
  }

  for(int i = 0; i<359; i++)
  {
  	if(std::abs(laserdata[i]-laserdata[i+1])>0.7)//detect Oi
	{
//std::cout<<"Find Oi "<<std::endl;
		point = laserIndexToPose(i, laserdata[i], robott.heading, robott);
		hDist = pose_xy::getLinearDistance(targett, point) +  pose_xy::getLinearDistance(point, robott); 
		if(hDist< last_hDist)
		{
			minPoint = point;
			last_dist = hDist;
		}
		point = laserIndexToPose(i+1, laserdata[i+1], robott.heading, robott);
		hDist = pose_xy::getLinearDistance(targett, point) +  pose_xy::getLinearDistance(point, robott); 
		if(hDist< last_hDist)
		{
			minPoint = point;
			last_dist = hDist;
		}
	}
  }

      
	    //std::cout<<"minimum Oi: "<<std::endl;
            //printf("%f %f \n", minPoint.x, minPoint.y);
  	    
	    return minPoint;
}
pose_xy Turtlebot3Drive::getRobotGoalVector(pose_xy p1, pose_xy p2)
{
       	pose_xy s;
       	s.x = p1.x - p2.x;
       	s.y = p1.y - p2.y;
       	s.heading  = atan2(s.y, s.x);
       	return s;
}
/*******************************************************************************
* Control Loop function
*******************************************************************************/
int Turtlebot3Drive::controlLoop()
{
  //move(0.0, 0.0);
  if (pose_xy::getLinearDistance(target, robot) < 0.1)
  {
  	std::cout<<"ah"<<std::endl;
	printf("goal->  x: %1.2f \t y: %1.2f\n robot-> x: %1.2f \t y: %1.2f\n", target.x, target.y, robot.x, robot.y);
updatecommandVelocity(0,0);
	return EXIT_SUCCESS;
  }
  else
  {
	if(obstacle_in_path(target, robot))
	{
		std::cout<<"obstacle in path!obstacle in path!obstacle in path!"<<std::endl;
		Oi = findClosestTangentPoint(target, robot);
	}
	else
	{
		printf("no obstacle!\n");
		Oi = target;
		move(Oi.x,Oi.y);
	}
	dist = pose_xy::getLinearDistance(target, Oi) + pose_xy::getLinearDistance(Oi, robot);
	//boundary following behavior!
	if(dist > last_dist+0.1)
	{
	   	dReached = pose_xy::getLinearDistance(target, robot);
	    	dFollowp = findClosestTangentPoint(target, robot);
	    	dFollow = pose_xy::getLinearDistance(dFollowp, target);
	    	while(dReached >= dFollow)
	    	{
			ros::Rate loop_ratee(125);
	    		loop_ratee.sleep();
	    		ros::spinOnce();
	    			

	    		goalRobotV = getRobotGoalVector(dFollow, robot);
			std::cout<<"SubGoal of following obs: ("<<goalRobotV.x<<", "<<goalRobotV.y<<")"<<std::endl;
	        	move(goalRobotV.x, goalRobotV.y);
	    	}
	 }
	 else if (obstacle_in_path(target, robot))
	 {
	        goalRobotV = getRobotGoalVector(Oi, robot);
		std::cout<<"SubGoal of motionTG: ("<<goalRobotV.x<<", "<<goalRobotV.y<<")"<<std::endl;
	        move(goalRobotV.x, goalRobotV.y);
	 }
	 ros::spinOnce();
    	 dist = pose_xy::getLinearDistance(target, Oi) + pose_xy::getLinearDistance(Oi, robot);
	 last_dist = dist;
  }
  
  return EXIT_SUCCESS;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "turtlebot3_drive");
  Turtlebot3Drive turtlebot3_drive;

  turtlebot3_drive.target.x = 3.0;
  turtlebot3_drive.target.y = 3.0;

  ros::Rate loop_rate(325);

  
  while (ros::ok())
  {
    turtlebot3_drive.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
