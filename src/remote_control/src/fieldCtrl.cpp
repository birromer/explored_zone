#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Empty.h"
#include <cmath>
#include <proj.h>


struct mission
{
	double point_begin_lat;
	double point_begin_lon;
	double point_end_lat;
	double point_end_lon;
	double point_begin_x;
	double point_begin_y;
	double point_end_x;
	double point_end_y;
	bool lineFollowing;
	double dist;
};

std::vector<mission> missions;
int id_mission =  -1;
double pos_x,pos_y;
ros::Time last_msg_fix;
double speed = 0.;
bool start = false;
double max_cov = 5.;
double offset_validation = -3.;

bool isInHeadingFollowingMode = false;
double headingToFollow;


ros::Time last_msg_distToLineAndCurrentWp;
ros::Time last_msg_mode;
ros::Time time_mission_start;

ros::Publisher chatter_speed;
ros::Publisher chatter_distToLine;
ros::Publisher chatter_currentWp;
ros::Publisher chatter_mission;
ros::Publisher chatter_mode;
ros::Publisher chatter_heading;


PJ_CONTEXT *C = proj_context_create();
PJ *P;
PJ* P_for_GIS;
PJ_COORD a, b;

void latLonToXY(double lat_, double lon_, double &x_, double &y_)
{
    a = proj_coord(lon_,lat_,0,0);
   	b = proj_trans(P, PJ_FWD, a);
    x_ = b.enu.n;
    y_ = b.enu.e;
}

void xyToLatLon(double x_, double y_,double &lat_, double &lon_)
{
   	a = proj_coord(y_,x_,0,0);
   	b = proj_trans (P, PJ_INV, a);
	lat_ = b.lp.phi;
	lon_ = b.lp.lam;
}

void setProj(double lon = -4.519289017)
{
	int zone = std::floor((lon + 180.0) / 6) + 1;
	std::string s = "+proj=utm +zone="+std::to_string(zone)+" +datum=WGS84";

	P = proj_create_crs_to_crs(C,"EPSG:4326",s.c_str(), NULL);
	P_for_GIS = proj_normalize_for_visualization(C, P);
	proj_destroy(P);
	P = P_for_GIS;

}

mission createMission(double point_begin_lat, double point_begin_lon, double point_end_lat, double point_end_lon, bool lineFollowing)
{
	mission m;

	double x_0;
	double y_0;
	double x_1;
	double y_1;
	latLonToXY(point_begin_lat,point_begin_lon,x_0,y_0);
	latLonToXY(point_end_lat,point_end_lon,x_1,y_1);

	m.point_begin_lat = point_begin_lat;
	m.point_begin_lon = point_begin_lon;
	m.point_end_lat = point_end_lat;
	m.point_end_lon = point_end_lon;

	m.point_begin_x = x_0;
	m.point_begin_y = y_0;
	m.point_end_x = x_1;
	m.point_end_y = y_1;

	m.lineFollowing = lineFollowing;

	m.dist = sqrt((x_0-x_1)*(x_0-x_1)+(y_0-y_1)*(y_0-y_1));
	
	return m;
}

double sawtooth(double x)
{
	return 2.*atan(tan(x/2.));
}

void callbackSpeed(const std_msgs::Float64 &msg)
{
	speed = msg.data;
}

void callbackSetHeading(const std_msgs::Float64 &msg)
{
	headingToFollow = msg.data;
	isInHeadingFollowingMode = true;
	start = true;
}

void callbackStart(const std_msgs::Empty &msg)
{
	if(id_mission == -1 and missions.size() != 0)
	{
		id_mission = 0;
		time_mission_start = ros::Time::now();
	}
	isInHeadingFollowingMode = false;
	start = true;
}

void callbackStop(const std_msgs::Empty &msg)
{
	start = false;
	isInHeadingFollowingMode = false;
}

void callbackMission(const std_msgs::Float64MultiArray &msg)
{
	int nb_wp = msg.data[0];
	if(nb_wp < 2)
	{
		ROS_WARN("Not enought waypoints...");
		return;
	}
	id_mission = 0;
	time_mission_start = ros::Time::now();
	missions.clear();
	for(int i = 1; i < nb_wp; i++)
	{
		missions.push_back(createMission(msg.data[1+(i-1)*3],msg.data[2+(i-1)*3],msg.data[1+i*3],msg.data[2+i*3],(bool)msg.data[3+i*3]));
	}
}

void callbackFix(const sensor_msgs::NavSatFix& msg)
{
	if(!(isnan(msg.latitude) || isnan(msg.longitude)))
	{
		setProj(msg.longitude);
		latLonToXY(msg.latitude,msg.longitude,pos_x,pos_y);
		last_msg_fix = ros::Time::now();
	}
}

bool timeOutMsg(ros::Time last_msg, double timeout = 0.2)
{
	if((ros::Time::now()-last_msg).toSec() < timeout)
	{
		return true;
	}
	else
	{
		return false;
	}
}

double fieldCtrl(double x, double y,double point_begin_x, double point_begin_y, double point_end_x, double point_end_y, bool lineFollowing, double &distToLine ,double dist_to_increase_waypoint_over_line = 4., double alpha_increase = 0.5, double max_increase_waypoint_over_line = 1.5)
{
	double norm = sqrt((point_end_x - point_begin_x)*(point_end_x - point_begin_x)+(point_end_y - point_begin_y)*(point_end_y - point_begin_y));
	double det_mat_dist = (point_end_x - point_begin_x)*(y - point_begin_y) - (point_end_y - point_begin_y) * (x - point_begin_x);
	double dist_line = det_mat_dist/norm;
	distToLine = dist_line;
	double nx,ny,wx,wy;
	nx = -(point_end_y - point_begin_y);
	ny = (point_end_x - point_begin_x);
	double norm1 = sqrt(nx*nx+ny*ny);
	nx = nx / norm1;
	ny = ny / norm1;
	
	double coeff = 0.1;

	if(dist_line < dist_to_increase_waypoint_over_line)
	{
		coeff = (dist_to_increase_waypoint_over_line-dist_line)*alpha_increase+0.1;
	}
	if(lineFollowing)
	{
		coeff = std::min(coeff,max_increase_waypoint_over_line);
		wx = -2.*((nx*nx)*(x - point_begin_x)+(nx*ny)*(y - point_begin_y)) - coeff*(x - point_end_x);
		wy = -2.*((nx*ny)*(x - point_begin_x) + (ny*ny)*(y - point_begin_y)) - coeff*(y - point_end_y);
	}
	else
	{
		wx = -2.*(x - point_end_x);
		wy = -2.*(y - point_end_y);
	}
	
	return atan2(wy,wx);
}

double dist_to_end(double ax,double ay,double bx,double by)
{
	double norm = sqrt((bx - ax)*(bx - ax)+(by - ay)*(by - ay));
	return ((bx - ax)*(pos_x - bx)+(by - ay)*(pos_y - by))/norm;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Field_node");

	ros::NodeHandle n;

	mission current_mission;
	ros::Subscriber sub_pos = n.subscribe("/fix", 1000, callbackFix);
	ros::Subscriber sub_mission = n.subscribe("/comms/in/mission", 1000, callbackMission);
	ros::Subscriber sub_speed = n.subscribe("/comms/in/speed", 1000, callbackSpeed);
	ros::Subscriber sub_heading = n.subscribe("/comms/in/setHeading", 1000, callbackSetHeading);
	ros::Subscriber sub_start = n.subscribe("/comms/in/start", 1000, callbackStart);
	ros::Subscriber sub_stop = n.subscribe("/comms/in/stop", 1000, callbackStop);

	ros::Publisher chatter_cmd = n.advertise<geometry_msgs::Twist>("/cmdVelHeading", 1000);
	chatter_mode = n.advertise<std_msgs::Int8>("/comms/out/sendCurrentMode", 1000);
	chatter_currentWp = n.advertise<std_msgs::Float64MultiArray>("/comms/out/sendCurrentWp", 1000);
	chatter_distToLine = n.advertise<std_msgs::Float64>("/comms/out/sendDistToLine", 1000);
	chatter_heading = n.advertise<std_msgs::Float64>("/comms/out/sendHeadingObj", 1000);

	setProj();

  	ros::Rate loop_rate(10);
 
    while (ros::ok())
    {	
    	geometry_msgs::Twist msg;
		msg.linear.x = 0.;
		msg.angular.z = 0.;
    	if(isInHeadingFollowingMode and start)
    	{
			msg.linear.x = speed;
			msg.angular.z = headingToFollow;
			chatter_cmd.publish(msg);

			std_msgs::Float64 msg_heading_obj;
			msg_heading_obj.data = msg.angular.z;
			chatter_heading.publish(msg_heading_obj);
    	}
    	else if(timeOutMsg(last_msg_fix) and id_mission != -1)
		{
	    	current_mission = missions[id_mission];
		   	double dist_end = dist_to_end(current_mission.point_begin_x,current_mission.point_begin_y,current_mission.point_end_x,current_mission.point_end_y);
			if(dist_end > offset_validation)
			{
				//ROS_ERROR("Valide");
				id_mission++;
				if(id_mission >= missions.size())
				{
					id_mission = -1;
					start = false;
				}
			}
			else
			{
				double distToLine = -1.;
				double angle = fieldCtrl(pos_x,pos_y,current_mission.point_begin_x,current_mission.point_begin_y,current_mission.point_end_x,current_mission.point_end_y,current_mission.lineFollowing,distToLine,4.,0.5,1.5);

				if(start)
				{
	    			msg.linear.x = speed;
					msg.angular.z = angle;
					chatter_cmd.publish(msg);	
					std_msgs::Float64 msg_heading_obj;
					msg_heading_obj.data = msg.angular.z;
					chatter_heading.publish(msg_heading_obj);		
				}

				if(!timeOutMsg(last_msg_distToLineAndCurrentWp,0.5))
				{
					std_msgs::Float64MultiArray msg_currentWp;
					msg_currentWp.data.resize(5);
					msg_currentWp.data[0] = current_mission.point_end_lat;
					msg_currentWp.data[1] = current_mission.point_end_lon;
					msg_currentWp.data[2] = -dist_end;
					msg_currentWp.data[4] = (ros::Time::now() - time_mission_start).toSec();
					double distToEnd = -dist_end;
					for(int i = id_mission+1; i < missions.size();i++)
					{
						distToEnd += missions[i].dist;
					}
					msg_currentWp.data[3] = distToEnd;

					std_msgs::Float64 msg_distToLine;
					msg_distToLine.data = distToLine;	

					chatter_distToLine.publish(msg_distToLine);
					chatter_currentWp.publish(msg_currentWp);
					
					last_msg_distToLineAndCurrentWp = ros::Time::now();
				}
			}
		}
		
		if(!timeOutMsg(last_msg_mode,0.5))
		{
			std_msgs::Int8 msg_mode;
			if(start and !isInHeadingFollowingMode)
			{
				if(current_mission.lineFollowing)
				{
					msg_mode.data = 2;
				}
				else
				{
					msg_mode.data = 1;
				}
			}
			else if (start and isInHeadingFollowingMode)
			{
				msg_mode.data = 3;
			}
			else
			{
				msg_mode.data = 0;	
			}
			chatter_mode.publish(msg_mode);
			last_msg_mode = ros::Time::now();
		}
        loop_rate.sleep();
	    ros::spinOnce();
    }

  
	return 0;
}