/* Authors: Jingru Feng, South China University of Technology */
/* Contact: feng_jingru@foxmail.com*/

#include "turtlebot3_gazebo/teleop_NTbug2.h"

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
  //std::cout<<"ready?";
  //printf("Getting position!!!! %f %f\n", robot.x, robot.y);
  tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, 
                                          msg->pose.pose.orientation.y, 
                                          msg->pose.pose.orientation.z, 
                                          msg->pose.pose.orientation.w); 
  tf::Matrix3x3(q).getRPY(roll, pitch, robot.heading);
  //std::cout<<"Getting heading!!!!  "<<robot.heading<<std::endl;
}

void Turtlebot3Drive::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  laser = *msg;
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
std::string getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  //int c = getchar();  // read character (non-blocking)
  std::string mystr;
  getline(std::cin,mystr);
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return mystr;
}
pose_xy Turtlebot3Drive::laserIndexToPose(int i, double dist, double heading, pose_xy robott)
{
  double ang_point = heading + laser_angle_increment * i;
  pose_xy point;
  point.x = robott.x + dist*cos(ang_point);
  point.y = robott.y + dist*sin(ang_point);
  point.heading = ang_point;
  //double ang_point = heading - (laser_angle_increment * i + laser_angle_min);

  return point;
}

void Turtlebot3Drive::moveStep()
{
  updatecommandVelocity(0.3,0);
  ros::Duration(0.3).sleep();//wait 0.5 second
  updatecommandVelocity(0,0);
}

/*void Turtlebot3Drive::move(double xvel, double yvel)
{
  geometry_msgs::Twist msg;
  double v, w;
//std::cout<<"To ("<<xvel<<", "<<yvel<<") "<<std::endl;
		double angVelz;// = atan2(yvel-robot.y, xvel-robot.x) - robot.heading; 
	
		double distance = sqrt(xvel*xvel + yvel*yvel); 
	
		angVelz = atan2(yvel-robot.y, xvel-robot.x) - robot.heading;
		//std::cout<<"robot.heading: "<<robot.heading<<std::endl;
		if (std::abs(angVelz)> 0.1){ 
		    v = 0;
		    w = 0.6*angVelz; 
//std::cout<<"turning"<<std::endl;
		} 
		else { 
		    v = 0.3;
		    w = 0; 
//std::cout<<"walking"<<std::endl;
        msg.linear.x = v; 
        msg.angular.z = w; 
	cmd_vel_pub_.publish(msg);}
	
}*/
void Turtlebot3Drive::moveAngle(double head)
{
  double angVelz = head - robot.heading;
  double v,w;
  while (std::abs(angVelz)> 0.1)
  { 
	v = 0;
	w = 0.6*angVelz;
	updatecommandVelocity(0,w);
	angVelz = head - robot.heading;
	//std::cout<<angVelz<<std::endl;
    ros::spinOnce();
  }
  updatecommandVelocity(0,0);
}
void Turtlebot3Drive::boundFollow()
{
  std::cout<<"Following boundry"<<std::endl;
  int index = 0;
  double startHeading = robot.heading;
  double startX = robot.x;
  double startY = robot.y;
  //m-line parameters
  double k = tan(robot.heading);
  double b = startY - k*startX;
  int step = 0;
  pose_xy oi;
  pose_xy m = laserIndexToPose(0,laserdata[0], robot.heading, robot);
  pose_xy moveVector;
  pose_xy s1;//s1, s2 is for obstacle following
  pose_xy s2;
  //determine from left or right side
  //if(laserdata[10]<laserdata[350])
  //{//to the right
  std::cout<<"1"<<std::endl;
	index = index + 360;
	bool foundOi = false;

	for(int i=index; i>0;i--)
	{
	  if(std::abs(laserdata[i]-laserdata[i-1])>0.7 && !foundOi)
	  {
		oi = laserIndexToPose(i,laserdata[i], robot.heading, robot);
	  	foundOi = true;
	  }
	}
	foundOi = false;
  std::cout<<"2:oi  x:"<<oi.x<<" y: "<<oi.y<<std::endl;	
	moveVector.x = oi.x - m.x;
	moveVector.y = oi.y - m.y;
  std::cout<<"3: angle"<<atan2(moveVector.y,moveVector.x)<<std::endl;
	moveAngle(atan2(moveVector.y,moveVector.x));

  //std::cout<<"3: I"<<(int)ceil((M_PI-atan2(moveVector.y,moveVector.x))*RAD2DEG)<<std::endl;
  std::cout<<"laserdata: "<<laserdata[90]<<std::endl; 
  for(int i=90; i>0;i--)
	{
	  if(std::abs(laserdata[i]-laserdata[i-1])>0.7 && !foundOi)
	  {
		oi = laserIndexToPose(i,laserdata[i], robot.heading, robot);
	  	foundOi = true;
	  }
	}
	foundOi = false;
	moveVector.x = oi.x - m.x;
	moveVector.y = oi.y - m.y;

	moveAngle(atan2(moveVector.y,moveVector.x));
  //if robot reach nicht m-line
  while(std::abs(k*robot.x + b - robot.y)<0.1)
  {
	ros::spinOnce();
	moveStep();
	s1 = laserIndexToPose(90,laserdata[90], robot.heading, robot);
	s2 = laserIndexToPose(85,laserdata[85], robot.heading, robot);
	moveVector.x = s1.x - s2.x;
	moveVector.y = s1.y - s2.y;
	moveAngle(atan2(moveVector.y,moveVector.x));
	
  }
	
  std::cout<<"4"<<std::endl;
	moveAngle(startHeading);
  std::cout<<"5"<<std::endl;
	//while(laserdata[90]<(step*0.1))
	//{  
	  //moveStep();
	//}
	
  //}
  //else
  //{//to the left

  //}
  //turtlebot3_state_num = TB3_DRIVE_FORWARD;
  updatecommandVelocity(0, 0);
}
/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool Turtlebot3Drive::controlLoop()
{
  std::cout<<"Input gesture: "<<std::endl;
  std::string c = getch();
//std::cout<<"Dist: "<<laserdata[0]<<std::endl;
  
  switch(turtlebot3_state_num)
  {
  	case STOP://////////STOP/////////
	if(c == "fs")//fingersSpread, make it simple with keyboard
	{
	  turtlebot3_state_num = TB3_DRIVE_FORWARD;
	  updatecommandVelocity(0.5, 0);
	}
	else if(c == "fist")
	{
	  turtlebot3_state_num = TB3_DRIVE_BACKWARD;
	  updatecommandVelocity(-0.5, 0);
	}
	else if(c == "wi")//waveIn
	{
	  turtlebot3_state_num = TB3_LEFT_TURN;
	  updatecommandVelocity(0,0.5);
	}
	else if(c == "wo")//waveOut
	{
	  turtlebot3_state_num = TB3_RIGHT_TURN;
	  updatecommandVelocity(0,-0.5);
	}	
	break;

	case TB3_DRIVE_FORWARD://////////GOING FORWARD/////////
	if(laserdata[0] <= check_forward_dist_)//obstacle detected
	{
	  updatecommandVelocity(0, 0);
	  boundFollow();
	}	
	if(c == "dt")//doubleTap
	{
	  turtlebot3_state_num = STOP;
	  updatecommandVelocity(0, 0);
	}
	else if(c == "wi")
	{
	  turtlebot3_state_num = TB3_LEFT_TURN;
	  updatecommandVelocity(0,0.5);
	}
	else if(c == "wo")
	{
	  turtlebot3_state_num = TB3_RIGHT_TURN;
	  updatecommandVelocity(0,-0.5);
	}
	break;

	case TB3_DRIVE_BACKWARD://///////GOING BACKWARD/////////
	if(c == "dt")//doubleTap
	{
	  turtlebot3_state_num = STOP;
	  updatecommandVelocity(0, 0);
	}
	else if(c == "wi")
	{
	  turtlebot3_state_num = TB3_LEFT_TURN;
	  updatecommandVelocity(0,0.5);
	}
	else if(c == "wo")
	{
	  turtlebot3_state_num = TB3_RIGHT_TURN;
	  updatecommandVelocity(0,-0.5);
	}
	break;

	case TB3_LEFT_TURN://////////TURNING LEFT/////////
	if(c == "dt")//doubleTap
	{
	  turtlebot3_state_num = STOP;
	  updatecommandVelocity(0, 0);
	}
	else if(c == "fs")//fingersSpread
	{
	  turtlebot3_state_num = TB3_DRIVE_FORWARD;
	  updatecommandVelocity(0.5, 0);
	}
	else if(c == "fist")
	{
	  turtlebot3_state_num = TB3_DRIVE_BACKWARD;
	  updatecommandVelocity(-0.5, 0);
	}
	break;

	case TB3_RIGHT_TURN://////////TURNING RIGHT////////
	if(c == "dt")//doubleTap
	{
	  turtlebot3_state_num = STOP;
	  updatecommandVelocity(0, 0);
	}
	else if(c == "fs")//fingersSpread
	{
	  turtlebot3_state_num = TB3_DRIVE_FORWARD;
	  updatecommandVelocity(0.5, 0);
	}
	else if(c == "fist")
	{
	  turtlebot3_state_num = TB3_DRIVE_BACKWARD;
	  updatecommandVelocity(-0.5, 0);
	}
	break;
  }
    /*switch(turtlebot3_state_num)
  {
    case STOP:
      if (scan_data_[CENTER] > check_forward_dist_)
      {
        if (scan_data_[LEFT] < check_side_dist_)
        {
          prev_tb3_pose_ = tb3_pose_;
          turtlebot3_state_num = TB3_RIGHT_TURN;
        }
        else if (scan_data_[RIGHT] < check_side_dist_)
        {
          prev_tb3_pose_ = tb3_pose_;
          turtlebot3_state_num = TB3_LEFT_TURN;
        }
        else
        {
          turtlebot3_state_num = TB3_DRIVE_FORWARD;
        }
      }

      if (scan_data_[CENTER] < check_forward_dist_)
      {
        prev_tb3_pose_ = tb3_pose_;
        turtlebot3_state_num = TB3_RIGHT_TURN;
      }
      break;

    case TB3_DRIVE_FORWARD:
      updatecommandVelocity(LINEAR_VELOCITY, 0.0);
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;

    case TB3_RIGHT_TURN:
      if (fabs(prev_tb3_pose_ - tb3_pose_) >= escape_range_)
        turtlebot3_state_num = GET_TB3_DIRECTION;
      else
        updatecommandVelocity(0.0, -1 * ANGULAR_VELOCITY);
      break;

    case TB3_LEFT_TURN:
      if (fabs(prev_tb3_pose_ - tb3_pose_) >= escape_range_)
        turtlebot3_state_num = GET_TB3_DIRECTION;
      else
        updatecommandVelocity(0.0, ANGULAR_VELOCITY);
      break;

    default:
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;
  }*/

  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "turtlebot3_drive");
  Turtlebot3Drive turtlebot3_drive;
  int turtlebot3_state_num = 0;//start with stop
  ros::Rate loop_rate(125);

  while (ros::ok())
  {
    turtlebot3_drive.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
