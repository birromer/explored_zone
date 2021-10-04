#include "ros/ros.h"
#include "tf/tf.h"
#include "vibes.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include "visualization_msgs/Marker.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include <random>
#include <chrono>
#include <cmath>
#include <proj.h>

const double coeff_rotation = 1.;
const double motor_force = 100.;
const double friction_drag_coeff = 3.;

double water_current_vx = 0.;
double water_current_vy = 0.;


const int freq = 30;
const double dt = 1./(double)freq;


double lat = 48.40181351;
double lon = -4.519093024;

double x = 0.;
double y = 0.;
double vx = 0.;
double vy = 0.;
double heading = 0.;
double previous_heading = 0.;
double roll = 0.;
double pitch = 0.;

double motor_angle = 0.; // [-20 deg,20 deg]
double cmd_motor = 0.; //[0,1] or [-1,1]

unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
std::default_random_engine generator(seed);
std::normal_distribution<double> gauss_vel(0.1,0.02);//
	
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

double sawtooth(double x)
{
	return 2.*atan(tan(x/2.));
}

double sign(double x)
{
	if(x >= 0.)
	{
		return 1.;
	}
	else
	{
		return -1.;
	}
}

void state_model()
{
	water_current_vx = gauss_vel(generator);
	water_current_vy = gauss_vel(generator);
	previous_heading = heading;
	x = x + (vx+water_current_vx)*dt;
	y = y + (vy+water_current_vy)*dt;
	heading = sawtooth(heading + (vx*cos(heading)+vy*sin(heading))*sin(motor_angle)*coeff_rotation*dt);
	//vx = vx + (motor_force*cmd_motor*cos(heading) - friction_drag_coeff*vx)*dt;
	//vy = vy + (motor_force*cmd_motor*sin(heading) - friction_drag_coeff*vy)*dt;

	 vx = vx * (1.-friction_drag_coeff*(1.+std::fabs(motor_angle))*dt) + (motor_force*cmd_motor*cos(heading)*dt);
     vy = vy * (1.-friction_drag_coeff*(1.+std::fabs(motor_angle))*dt) + (motor_force*cmd_motor*sin(heading)*dt);

	xyToLatLon(x,y,lat,lon);
}

void callbackCmd(const geometry_msgs::Twist& msg)
{
	cmd_motor = 0.3*std::min(1.,std::max(0.,msg.linear.x));
	motor_angle = 0.4*std::min(1.,std::max(-1.,msg.angular.z));
	//ROS_WARN("%f",motor_angle*180./M_PI);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Simu_node");

	ros::NodeHandle n;
	ros::Subscriber sub_cmd_vel = n.subscribe("/auto/cmd", 1000, callbackCmd);
	ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu/data", 1000);
	ros::Publisher fix_pub = n.advertise<sensor_msgs::NavSatFix>("/fix", 1000);
	ros::Publisher vel_pub = n.advertise<geometry_msgs::TwistWithCovariance>("/overGround", 1000);
	ros::Publisher heading_pub = n.advertise<std_msgs::Float64>("/heading", 1000);
  	ros::Rate loop_rate(freq);

	setProj(lon);

	tf2_ros::TransformBroadcaster br;

	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.frame_id = "/map";
	transformStamped.child_frame_id = "/boat";

	latLonToXY(lat,lon,x,y);

	vibes::beginDrawing();
    vibes::newFigure("Simu");
    vibes::setFigureProperties("Simu",vibesParams("x", 100, "y", 100,"width", 1000, "height", 1000));
    vibes::axisLimits(-100.+x, 100.+x, -100.+y, 100.+y);

    while (ros::ok())
    {
    	state_model();

		tf2::Quaternion quaternion_tf2;
		quaternion_tf2.setRPY(roll, pitch, heading);
		sensor_msgs::Imu msg_imu;
		msg_imu.header.frame_id = "imu";
		msg_imu.header.stamp = ros::Time::now();
		msg_imu.orientation = tf2::toMsg(quaternion_tf2);
		msg_imu.orientation_covariance = {-1,0.,0.,0.,0.,0.,0.,0.,0.};
		msg_imu.angular_velocity.x = 0.;
		msg_imu.angular_velocity.y = 0.;
		msg_imu.angular_velocity.z = sawtooth(heading-previous_heading)/dt;
		msg_imu.angular_velocity_covariance = {-1,0.,0.,0.,0.,0.,0.,0.,0.};
		msg_imu.linear_acceleration.x = 0.;
		msg_imu.linear_acceleration.y = 0.;
		msg_imu.linear_acceleration.z = 0.;
		msg_imu.linear_acceleration_covariance = {-1,0.,0.,0.,0.,0.,0.,0.,0.};

		imu_pub.publish(msg_imu);

		geometry_msgs::TwistWithCovariance msg_vel;
		msg_vel.twist.linear.x = sqrt((vx+water_current_vx)*(vx+water_current_vx)+(vy+water_current_vy)*(vy+water_current_vy));
		msg_vel.twist.angular.z = atan2(vy+water_current_vy,vx+water_current_vx);
		msg_vel.covariance[0] = 0.1;	
		msg_vel.covariance[35] = 0.2;		
		vel_pub.publish(msg_vel);

		sensor_msgs::NavSatFix msg_navsatfix;
		msg_navsatfix.header.stamp = ros::Time::now();
		msg_navsatfix.latitude = lat;
		msg_navsatfix.longitude = lon;
		msg_navsatfix.altitude = 5.;
		msg_navsatfix.position_covariance[0] = 1.;
		msg_navsatfix.position_covariance[4] = 1.;
		msg_navsatfix.position_covariance[8] = 2.;
		msg_navsatfix.position_covariance_type = (uint8_t)2;

		fix_pub.publish(msg_navsatfix);

		std_msgs::Float64 msg_heading;
		msg_heading.data = heading;
		heading_pub.publish(msg_heading);

    	vibes::clearFigure("Simu");
		vibes::drawVehicle(x, y,(heading)*180./M_PI,1.);
		

		transformStamped.header.stamp = ros::Time::now();
		transformStamped.transform.translation.x = x;
		transformStamped.transform.translation.y = y;
		double cy = cos(heading * 0.5);
	    double sy = sin(heading * 0.5);
	    double cp = cos(pitch * 0.5);
	    double sp = sin(pitch * 0.5);
	    double cr = cos(roll * 0.5);
	    double sr = sin(roll * 0.5);

		transformStamped.transform.rotation.x = sr * cp * cy - cr * sp * sy;
		transformStamped.transform.rotation.y = cr * sp * cy + sr * cp * sy;
		transformStamped.transform.rotation.z = cr * cp * sy - sr * sp * cy;
		transformStamped.transform.rotation.w = cr * cp * cy + sr * sp * sy;

		br.sendTransform(transformStamped);


		visualization_msgs::Marker marker;
		marker.header.frame_id = "boat";
		marker.header.stamp = ros::Time();
		marker.id = 0;
		//marker.type = visualization_msgs::Marker::SPHERE;
		marker.type = visualization_msgs::Marker::MESH_RESOURCE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0.;


		marker.pose.orientation.x = 0.5;
		marker.pose.orientation.y = 0.5;
		marker.pose.orientation.z = 0.5;
		marker.pose.orientation.w = 0.5;
		marker.scale.x = 0.05;
		marker.scale.y = 0.05;
		marker.scale.z = 0.05;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		
		//only if using a MESH_RESOURCE marker type:
		marker.mesh_resource = "package://simu/meshes/boat.dae";
		vis_pub.publish( marker );

        loop_rate.sleep();
	    ros::spinOnce();
    }

    vibes::endDrawing();

  
	return 0;
}
