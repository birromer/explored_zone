#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/String.h"
#include <cmath>
#include <fstream>
#include "sensor_msgs/NavSatFix.h"
#include <proj.h>

std::ofstream log_file;
std::ofstream log_file2;
std::ofstream log_file3;
std::ofstream log_file4;

double cmd_lx;
double cmd_az;

bool mode_dvl = false;

double dvl_forward;
double dvl_starboard;

bool init = false;
double x0_;
double y0_;


PJ_CONTEXT *C = proj_context_create();
PJ *P;
PJ* P_for_GIS;
PJ_COORD a, b;

std::vector<double> lats = {48.4016736602982,48.4014949076654,48.4014750954458,48.4015154723762,48.4016942250776,48.4017346020217,48.4017147897851,48.4015360370152,48.4015162247007,48.4015566015823,48.4017353544208,48.4017757313161,48.4017559189845,48.4015771660775,48.4015573536681,48.4015977305009,48.4017764834764,48.401816860323,48.4017970478965,48.4016182948524,48.401598482348,48.4016388591321,48.4018176122447,48.4018579890424,48.401838176521,48.4016594233399,48.4016396107405,48.4016799874758,48.4018587407254,48.4018991174744,48.401879304858,48.401838928125,48.4016332833213,48.4015929063358,48.4015730941945,48.4017787387674,48.4017589261198,48.4017185494017,48.4015129050595,48.401472528089,48.4014527159164,48.4016583600278};
std::vector<double> lons = {-4.5192572275497,-4.5193489475193,-4.51940958457395,-4.5194393380978,-4.51934761841141,-4.51937737201851,-4.51943800934454,-4.51952972874772,-4.51959036577842,-4.51962011946905,-4.51952840034909,-4.51955815412297,-4.51961879142506,-4.5197105102618,-4.51977114726855,-4.51980090112596,-4.51970918257244,-4.51973893651309,-4.51979957379124,-4.51989129206155,-4.51995192904435,-4.51998168306854,-4.51988996508145,-4.51991971918889,-4.51998035644308,-4.52007207414695,-4.52013271110581,-4.52016246529678,-4.52007074787613,-4.52010050215035,-4.5201611393806,-4.52013138507067,-4.51922747407368,-4.51919772064534,-4.51925835787622,-4.52016226791957,-4.52022290498303,-4.52019315069705,-4.51928924160734,-4.51925948820295,-4.51932012526705,-4.52022403340312};
std::vector<int> isLine = {0,1,0,0,1,0,0,1,0,0,1,0,0,1,0,0,1,0,0,1,0,0,1,0,0,1,0,0,1,0,0,0,1,0,0,1,0,0,1,0,0,1};



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

double normalize(double x)
{
	return std::max(-1.,std::min(1.,x));
}

std::vector<std::string> split(const std::string& s, char delimiter)
{
   //https://www.fluentcpp.com/2017/04/21/how-to-split-a-string-in-c/
   std::vector<std::string> tokens;
   std::string token;
   std::istringstream tokenStream(s);
   while (std::getline(tokenStream, token, delimiter))
   {
      tokens.push_back(token);
   }
   return tokens;
}

void callbackFix(const sensor_msgs::NavSatFix& msg)
{
	if(!(isnan(msg.latitude) || isnan(msg.longitude)))
	{
		double x,y;
		latLonToXY(msg.latitude,msg.longitude,x,y);
		x -= x0_;
		y -= y0_;
		log_file4 << std::setprecision(12) << x << ";" << std::setprecision(12) << y <<"\n";
;
	}
}

void callbackCmd(const geometry_msgs::Twist& msg)
{
	cmd_lx = normalize(msg.linear.x);
	cmd_az = normalize(msg.angular.z);
}

void callbackDvl(const geometry_msgs::Vector3& msg)
{
	dvl_forward = sqrt(2.)/2.*msg.y - sqrt(2.)/2.*msg.x;
	dvl_starboard = sqrt(2.)/2.*msg.y + sqrt(2.)/2.*msg.x;
}

void callbackData(const std_msgs::String& msg)
{
	std::vector<std::string> elems = split(msg.data,',');
	log_file << std::setprecision(12) << std::stod(elems[1]) << ";"<< std::setprecision(12) << std::stod(elems[2]) << ";" << std::setprecision(8)<< std::stod(elems[20]) << ";" << std::setprecision(8)<< std::stod(elems[21]) << ";" << std::setprecision(8)<< cmd_lx << ";" << std::setprecision(8)<< cmd_az << ";" << std::setprecision(8)<< std::stod(elems[23]);
	if(mode_dvl)
	{
		log_file <<";"<< dvl_forward << ";" << dvl_starboard << "\n"; 
	}
	else
	{
		log_file << "\n";
	}

	double x,y;
	latLonToXY(std::stod(elems[1]),std::stod(elems[2]),x,y);
	if(!init)
	{
		x0_ = x;
		y0_ = y;
		init = true;
	}
	x -= x0_;
	y -= y0_;
	log_file2 << std::setprecision(12) << x << ";" << std::setprecision(12) << y << "\n";




}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Dist_node");

	ros::NodeHandle n;
	setProj();

	ros::Subscriber sub_data = n.subscribe("/str_data", 1000, callbackData);
	ros::Subscriber sub_cmd = n.subscribe("/auto/cmd", 1000, callbackCmd);
	ros::Subscriber sub_dvl = n.subscribe("/wayfinder/data/velocity", 1000, callbackDvl);
	ros::Subscriber sub_pos = n.subscribe("/fix", 1000, callbackFix);
    std::string name = "extract.csv";
  	log_file.open(name, std::ios::out);
  	std::string name2 = "extract_m.csv";
  	log_file2.open(name2, std::ios::out);


  	std::string name3 = "extract_p.csv";
  	log_file3.open(name3, std::ios::out);

  	std::string name4 = "extract_s.csv";
  	log_file4.open(name4, std::ios::out);
  	for(int i = 0; i < lats.size(); i++)
  	{

		double x,y;
		latLonToXY(lats[i],lons[i],x,y);
		if(!init)
		{
			x0_ = x;
			y0_ = y;
			init = true;
		}
		x -= x0_;
		y -= y0_;
		log_file3 << std::setprecision(12) << x << ";" << std::setprecision(12) << y << ";" << isLine[i] <<"\n";

  	}
	log_file3.close();


	ros::spin();
	log_file.close();
	log_file2.close();
	log_file4.close();
  
	return 0;
}