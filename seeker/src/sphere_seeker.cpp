/*

Sphere seeker will attempt to drive turtlebot into a sphere in an empty world.

Steps:
1. catkin_make
2. roslaunch turtlebot_gazebo turtlebot_world.launch
world_file:='"<path>/mini_world.world"'
3. roslaunch seeker interview.launch
4. rosservice call /enable “data: true”
5. rostopic echo /displacement

*/

#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include "math.h"
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>

#define PI 3.14159265358979323846

class Seeker
{
  public:

    //variables
    bool start;
    int path_current;
    int path_next;
    int path_state;
    double current_time;
    double x, y, vel, yaw;
    double x_i, y_i, yaw_i;
    double scan_theta, d_time;
    double target_theta, delta_theta;
    double dist_to_sphere, dist_traveled, dist_target;
    double omega;
    
    std::vector<int> path;
    std::vector<float> ranges;

    enum State { IDLE=0, DRIVE=1, SCAN=2, ROTATE=3, CHASE=4, SEEK=5, RESET=6 };
    State turtlebot_state;

    //state matrix
    int transition_mat[5][5] = {
    	{  1,  45, 135, 225, 315},
        {225,   1, 180,   1, 270},
        {315,   0,   1, 270,   1},
        { 45,   1,  90,   1,   0},
	{135,  90,   1, 180,   1}
        };

    //callbacks
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    //services
    bool serve(std_srvs::SetBool::Request&  req,
	       std_srvs::SetBool::Response& res);
    //methods
    void genPath();
    void setInitialPosition();
    bool getStart();
    bool checkScan(int, int);
    double guide();
    geometry_msgs::Twist updateState();
    geometry_msgs::Vector3 getDisplacement();

    Seeker();

};

Seeker::Seeker()
{
    start = false;
    ROS_INFO("Turtlebot initialized");
    ROS_INFO("Generating search path...");
    genPath();
    ROS_INFO("Executing search path: %i -> %i -> %i -> %i -> %i -> %i",\
             path[0], path[1], path[2], path[3], path[4], path[5]);
    turtlebot_state = RESET;
    dist_traveled = 0.0;
    dist_target = 14 / sqrt(2);
    scan_theta = 0.0;
    path_state = 0;
    path_current = path[path_state];
    path_next = path[path_state+1];
}

geometry_msgs::Twist Seeker::updateState()
{
    geometry_msgs::Twist msg;

    switch ( turtlebot_state )
    {
      case RESET:
      {
	setInitialPosition();
	turtlebot_state = SCAN;
	ROS_INFO("Scanning for sphere.");
	break;
      }

      case IDLE:
      {
	msg.linear.x=0.0;
	msg.linear.y=0.0;
	msg.angular.z=0.0;
	break;
      }

      case DRIVE:
      {
	target_theta = target_theta = transition_mat[path_next][path_current] * PI / 180;

	if ( dist_traveled < dist_target / 20.0 )
	{
	    double z = guide();
	    msg.linear.x = 0.2;
	    msg.angular.z = z;
	}
	else if ( dist_traveled < dist_target )
	{
	    double z = guide();
	    msg.linear.x = 1.0;
	    msg.angular.z = z;
	}
	else if ( dist_traveled >= dist_target && vel > 0.01 )
	{
	    msg.linear.x = 0.0;
	    msg.angular.z = 0.0;
	}
	else if ( dist_traveled >= dist_target && vel < 0.01 )
	{
	    dist_target = 14.0;
	    path_current = path_next;
            if ( path_state <= 4 )
            {
                path_state++;
                path_next = path[path_state+1];
	    }
	    scan_theta = 0.0;
	    turtlebot_state = SCAN;
	    ROS_INFO("Scanning for sphere.");
	}
	if ( checkScan(10, 630) == true )
	{
            dist_traveled = 0.0;
	    dist_target = 1.0;
            turtlebot_state = SEEK;
	    ROS_INFO("Sphere sighted. Trying to move closer.");
        }
	break;
      }

      case CHASE:
      {
	if ( checkScan(300, 340) == true )
	{
    	    target_theta = yaw;

	    if ( dist_traveled < 0.1 )
	    {
	        msg.linear.x = 0.1;
	    }
	    else if ( dist_to_sphere < 0.55)
	    {
	        msg.linear.x = 0.0;
	        turtlebot_state = IDLE;
		ROS_INFO("Sphere found!");
	    }
	    else
	    {
	        msg.linear.x = 0.5;
	        turtlebot_state = CHASE;
	    }
	}
	else
	{
	    turtlebot_state = SEEK;
	    dist_traveled = 0.0;
	    dist_target = 1.0;
	    ROS_INFO("Lost sight of sphere.");
	}
	double z = guide();
	msg.angular.z = z;
	break;
      }

      case SEEK:
      {
	msg.angular.z = 0.0;
	
	if ( dist_traveled < dist_target && omega < 0.01 )
	{
	    msg.linear.x = 1.0;
	}
	else if ( dist_traveled >= dist_target && vel > 0.01 )
	{
	    msg.linear.x = 0.0;
	}
	else if ( dist_traveled >= dist_target && vel < 0.01 )
	{
	    turtlebot_state = SCAN;
	    ROS_INFO("Scanning for sphere.");
	}
	break;
      }

      case SCAN:
      {
	msg.angular.z = 0.5;
	if ( scan_theta < 2*PI )
	{
	    if ( path_state == 5 )
	    {
                turtlebot_state = IDLE;
                ROS_INFO("Search path complete. Sphere not found.");
	    }
	    else if ( checkScan(300, 340) == true )
	    {
		dist_traveled = 0.0;
	        target_theta = yaw;
	        turtlebot_state = CHASE;
		scan_theta = 0.0;
		ROS_INFO("Sphere sighted. Trying to move closer.");
	    }
	}
	if ( scan_theta > 2*PI )
	{
	    scan_theta = 0.0;
	    turtlebot_state = ROTATE;
	    ROS_INFO("Sphere not found. Moving from state %i -> %i", path_current, path_next);
	}
        break;
      }
      case ROTATE:
      {
	target_theta = transition_mat[path_next][path_current] * PI / 180;
	double z = guide();
	msg.linear.x = 0.0;
	msg.angular.z = z;
	if ( fabs(delta_theta) < PI/360.0 )
	{
	    dist_traveled = 0.0;
	    turtlebot_state = DRIVE;
	}
        break;
      }
      default:
      {
	msg.linear.x = 0.0;
	msg.linear.y = 0.0;
	msg.angular.z = 0.0;
	break;
      }
    }
    return msg;
}

bool Seeker::checkScan(int low, int high)
{
    int found = 0;
    for (int i=low; i<=high; i++)
    {
        if (ranges[i] > 0.0)
	{
	    dist_to_sphere = ranges[i];
	    found = found + 1;
	}
    }

    bool yes = false;
    if ( found > 0 )
    {
	yes = true;
    }
    else
    {
	yes = false;
    }
    return yes;
}

double Seeker::guide()
{
    double delta_cos;
    double delta_sin;
    double z;
    delta_cos = cos(target_theta) - cos(yaw);
    delta_sin = sin(target_theta) - sin(yaw);
    delta_theta = atan(delta_sin/delta_cos);

    if ( fabs(delta_theta) > PI/45.0 )
    {
	if (delta_theta > 0)
	{
	    z = -0.6;
	}
	else
	{
	    z = 0.6;
	}
    }
    else if ( delta_theta > PI/360.0 )
    {
	if (delta_theta > 0)
	{
	    z = -0.3;
	}
	else
	{
	    z = 0.3;
	}
    }
    else if ( delta_theta < PI/360.0 )
    {
        if (delta_theta > 0)
        {
            z = -0.1;
        }
        else
        {
            z = 0.1;
        }
    }
    //uncomment to debug guidance
    //std::cout << delta_theta << " " << target_theta << " " << yaw << " " << path_current << " " << path_next << "\n";
    return z;
}

void Seeker::genPath()
{
    srand((unsigned)time(0));
    int curr = 0;
    int next;
    int proxy_mat[5][5] = {
        {  1,  45, 135, 225, 315},
        {225,   1, 180,   1, 270},
        {315,   0,   1, 270,   1},
        { 45,   1,  90,   1,   0},
        {135,  90,   1, 180,   1}
        };

    std::vector<int> current_row;
    std::vector<int> avail_states;

    while ( path.size() < 4 )
    {
	for (int j=0; j<5; j++)
	{
            int s = proxy_mat[curr][j];
	    current_row.push_back(s);
	    if ( s != 1 ) { avail_states.push_back(s); }
	}
	int rand_int = rand() % avail_states.size();
        int target = avail_states[rand_int];
	for (int i=0; i<current_row.size(); i++)
	{
            if (current_row[i] == target)
	    {
		for (int i=0; i<5; i++) { proxy_mat[i][curr]=1; }
		path.push_back(curr);
		curr = i;
		next = i;
	    }
	}
	current_row.clear();
	avail_states.clear();
    }
    path.push_back(next);
    path.push_back(path[1]);
}

void Seeker::setInitialPosition()
{
    x_i = x;
    y_i = y;
}

geometry_msgs::Vector3 Seeker::getDisplacement()
{
    geometry_msgs::Vector3 disp;

    if ( dist_to_sphere > 0.0 )
    {
        disp.x = dist_to_sphere * cos(target_theta);
        disp.y = dist_to_sphere * sin(target_theta);
    }
    else
    {
	disp.x = 0.0;
	disp.y = 0.0;
    }
    disp.z = 0.0;
    return disp;
}

void Seeker::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    double v_x = msg->twist.twist.linear.x;
    double v_y = msg->twist.twist.linear.y;
    vel = sqrt(v_x*v_x + v_y*v_y);

    double d_x, d_y;
    d_x = v_x * d_time;
    d_y = v_y * d_time;
    dist_traveled = dist_traveled + sqrt( d_x*d_x + d_y*d_y );
}

void Seeker::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ranges = msg->ranges;
}

void Seeker::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    d_time = ros::Time::now().toSec() - current_time;
    current_time = ros::Time::now().toSec();
    omega = msg->angular_velocity.z;

    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    double r, p, y;
    tf::Matrix3x3 m(q);
    m.getRPY(r, p, y);
    yaw = y;
    if ( fabs(d_time) < 1.0)
    {
        scan_theta = scan_theta + d_time * omega;
    }
}

bool Seeker::serve(std_srvs::SetBool::Request&  req,
	     	   std_srvs::SetBool::Response& res)
{
    start = req.data;
    if (req.data == true) {
    ROS_INFO("Turtlebot is starting search.");
    res.message = "Enable is set to True";
    }
    else {
    ROS_INFO("Turtlebot is stopping search.");
    res.message = "Enable is set to False";
    }
    res.success = true;
    return true;
}

bool Seeker::getStart()
{
    return start;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "seeker_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);
    Seeker robot;

    ros::ServiceServer service1 = nh.advertiseService("enable", &Seeker::serve, &robot);
    ros::Publisher pub1 = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);
    ros::Publisher pub2 = nh.advertise<geometry_msgs::Vector3>("/displacement", 100);
    ros::Subscriber sub1 = nh.subscribe("odom", 100, &Seeker::odomCallback, &robot);
    ros::Subscriber sub2 = nh.subscribe("scan", 100, &Seeker::laserCallback, &robot);
    ros::Subscriber sub3 = nh.subscribe("mobile_base/sensors/imu_data", 100, &Seeker::imuCallback, &robot);

    ROS_INFO("Please pass true to the /enable service to start search.");
    ROS_INFO("Echo /displacement to see relative distance.");

    geometry_msgs::Twist msg;
    geometry_msgs::Vector3 disp;

    while ( ros::ok() )
    {
        if ( robot.getStart() )
        {
            msg = robot.updateState();
	    disp = robot.getDisplacement();
	    pub1.publish(msg);
	    pub2.publish(disp);
        }
        ros::spinOnce();
	loop_rate.sleep();
    }

  return 0;
}
