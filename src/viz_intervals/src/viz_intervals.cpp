#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <cmath>
#include "UTM.h"
#undef pi

#include <proj.h>
#include <iostream>
#include <codac.h>



codac::VIBesFig *fig;

bool switchOn = false;
bool autoOn = false;
int state = 0;

float pos_x_init;
float pos_y_init;

float ancien_pos_x;
float ancien_pos_y;

double current_heading;
//rosbag play bag_2021-05-18-15-31-36.bag -r 5  -s 70 -u 300


void callbackRcEnabled(const std_msgs::Bool& msg)
{
    switchOn = msg.data;
}

void callbackCmd(const geometry_msgs::Twist& msg)
{
    autoOn = true;
}

void callbackHeading(const std_msgs::Float64& msg)
{
    current_heading = msg.data;
}

std::vector<double> rotate_pt(vector<double> pt, double alpha, float c_x, float c_y) {
     double c_a = cos(alpha);
     double s_a = sin(alpha);

     double x1_tmp =  pt[0]*c_a + pt[1]*s_a + (1-c_a)*c_x - s_a*c_y;
     double y1_tmp = -pt[0]*s_a + pt[1]*c_a +     s_a*c_x + (1-c_a)*c_y;

     return {x1_tmp, y1_tmp};
}

float radius(float c_x, float c_y, std::vector<double> p1, std::vector<double> p2){ 
//float radius(float c_x, float c_y, std::vector<double> p1){ 
	//compute an approximation of the radius of a circle passing by 2 points
	float d1;
	float d2;
	float moy;
	d1 = std::sqrt(std::pow(c_x-p1[0], 2) + std::pow(c_y-p1[1], 2));
	d2 = std::sqrt(std::pow(c_x-p2[0], 2) + std::pow(c_y-p2[1], 2));
	moy = (d1+d2)/2;
	return moy;
	//return d1;

}



/*float angle(std::vector<double> p1, std::vector<double> p2, std::vector<double> p3, std::vector<double> p4){ 
	//compute the angle between two lines knowing 2 points for each line
	
	float a;
	float c;
	float a1;
	float c1;
	a = (p1[1]-p3[1])/(p1[0]-p3[0]);
	c = (p1[1] - a*p1[0]);

	a1 = (p2[1]-p4[1])/(p2[0]-p4[0]);
	c1 = (p2[1] - a*p2[0]);

	return atan2(a-a1, a*a1 + 1);

}*/

float angle(float c_x, float c_y, std::vector<double> p1, std::vector<double> p2){ 
	//compute the angle between the robot and 2 points
	float p12 = std::sqrt(std::pow(c_x-p1[0], 2) + std::pow(c_y-p1[1], 2));
	float p13 = std::sqrt(std::pow(c_x-p2[0], 2) + std::pow(c_y-p2[1], 2));
	float p23 = std::sqrt(std::pow(p1[0]-p2[0], 2) + std::pow(p1[1]-p2[1], 2));

	return (std::acos((std::pow(p12, 2)+std::pow(p13, 2)-std::pow(p23, 2))/(2*p12*p13))) * 180/M_PI;


}

std::pair<float, float> center(std::vector<double> p1, std::vector<double> p2, std::vector<double> p3, std::vector<double> p4){ 
	//compute the intersection of 2 lines knowing 2 pts for each line
	float a;
	float c;
	float a1;
	float c1;
	float c_x;
	float c_y;
	a = (p1[1]-p3[1])/(p1[0]-p3[0]);
	c = (p1[1] - a*p1[0]);

	a1 = (p2[1]-p4[1])/(p2[0]-p4[0]);
	c1 = (p2[1] - a*p2[0]);

	c_x = (c1-c)/(a-a1);
	c_y = a*c_x+c;
	return std::make_pair(c_x, c_y);
	
}

void callbackFix(const sensor_msgs::NavSatFix& msg)
{

    //if(autoOn and !switchOn and state != 2)
        state = 1;
	std::vector<std::vector<double>> zone;

        //ROS_WARN("lat: %f | long: %f | stamp: %f", msg.latitude, msg.longitude, msg.header.stamp);
        float pos_y,pos_x;
        LatLonToUTMXY(msg.latitude,msg.longitude,0,pos_y,pos_x);
        pos_x -= pos_x_init;
        pos_y -= pos_y_init;

        

        if ((ancien_pos_x != pos_x) || (ancien_pos_y != pos_y))
        {

	zone.push_back({pos_x + 3,pos_y + 2});
	zone.push_back({pos_x + 6,pos_y + 4});
   	zone.push_back({pos_x - 6,pos_y + 4});
	zone.push_back({pos_x-3,pos_y + 2});
    	zone.push_back({pos_x + 3,pos_y + 2});

        vibes::drawVehicle(pos_x, pos_y,(current_heading)*180./M_PI,1., vibesParams("figure", "Vision") );
        vibes::drawVehicle(pos_x, pos_y,(current_heading)*180./M_PI,1., vibesParams("figure", "Trajectory") );

        codac::Interval r(3.,4.);
        double r_min = 3, r_max=4;
        double th_min = -35 + (current_heading)*180./M_PI;
        double th_max = 35 + (current_heading)*180./M_PI;
        codac::Interval theta(-M_PI/10.,M_PI/10.);
	
	//std::pair<float, float> centre = center(zone[3], zone[0], zone[2], zone[1]);
	//float r1 = radius(centre.first, centre.second, zone[3], zone[0]);
	//float r2 = radius(centre.first, centre.second, zone[1], zone[2]);
	float r1 = radius(pos_x, pos_y, zone[0], zone[3]);
	float r2 = radius(pos_x, pos_y, zone[1], zone[2]);
	//float Angle = angle(zone[3], zone[0], zone[2], zone[1]);
	float Angle = angle(pos_x, pos_y, zone[3], zone[0]);
	zone[0] = rotate_pt(zone[0], -current_heading + M_PI/2, pos_x, pos_y);
	zone[1] = rotate_pt(zone[1], -current_heading+ M_PI/2, pos_x, pos_y);
	zone[2] = rotate_pt(zone[2], -current_heading+ M_PI/2, pos_x, pos_y);
	zone[3] = rotate_pt(zone[3], -current_heading+ M_PI/2, pos_x, pos_y);
	zone[4] = rotate_pt(zone[4], -current_heading+ M_PI/2, pos_x, pos_y);
        //fig->draw_pie(pos_x, pos_y, r, theta, "blue[cyan]");
        //vibes::drawPie(pos_x, pos_y, r_min, r_max, th_min, th_max, "blue[blue]", vibesParams("figure", "Vision"));
	vibes::drawPie(pos_x, pos_y, r1, r2, -(Angle/2)+(current_heading)*180./M_PI, Angle/2+(current_heading)*180./M_PI, "blue[blue]", vibesParams("figure", "Trajectory"));
	vibes::drawLine(zone,"red[red]", vibesParams("figure", "Trajectory"));
        }

        ancien_pos_x = pos_x;
        ancien_pos_y = pos_y;
    //}

    if(state == 1 and switchOn)
    {
        state = 2;
    }
}


int main(int argc, char **argv)
{

  vibes::beginDrawing();

  vibes::newFigure("Vision");


/*
    PJ_CONTEXT *C;
    PJ *P;
    PJ* P_for_GIS;
    PJ_COORD a, b;

    /* or you may set C=PJ_DEFAULT_CTX if you are sure you will     */
    /* use PJ objects from only one thread                          */
    //C = proj_context_create();

   //P = proj_create_crs_to_crs (C,                                "EPSG:4326",                                "+proj=utm +zone=32 +datum=WGS84", /* or EPSG:32632 */                                NULL);

    //if (0==P) {
        //fprintf(stderr, "Oops\n");
        //return 1;
    //}

    /* This will ensure that the order of coordinates for the input CRS */
    /* will be longitude, latitude, whereas EPSG:4326 mandates latitude, */
    /* longitude */
   // P_for_GIS = proj_normalize_for_visualization(C, P);
    //if( 0 == P_for_GIS )  {
    //    fprintf(stderr, "Oops\n");
    //    return 1;
   // }
    //proj_destroy(P);
   // P = P_for_GIS;

    /* a coordinate union representing Copenhagen: 55d N, 12d E    */
    /* Given that we have used proj_normalize_for_visualization(), the order of
    /* coordinates is longitude, latitude, and values are expressed in degrees. */
    //a = proj_coord (-4.519051,48.401692, 0, 0);

    /* transform to UTM zone 32, then back to geographical */
   //b = proj_trans (P, PJ_FWD, a);
   // printf ("easting: %.3f, northing: %.3f\n", b.enu.e, b.enu.n);
    //b = proj_trans (P, PJ_INV, b);
    //printf ("longitude: %g, latitude: %g\n", b.lp.lam, b.lp.phi);

    /* Clean up */
    //proj_destroy (P);
  //  proj_context_destroy (C); /* may be omitted in the single threaded case */

    ros::init(argc, argv, "Dist_node");

    ros::NodeHandle n;

    ros::Subscriber sub_pos = n.subscribe("/fix", 1000, callbackFix);
    //ros::Subscriber sub_cmd = n.subscribe("/cmd_vel", 1000, callbackCmd);
    ros::Subscriber sub_cmd = n.subscribe("/rc/cmd", 1000, callbackCmd);
    ros::Subscriber sub_heading = n.subscribe("/heading", 1000, callbackHeading);
    ros::Subscriber sub_rc_enabled = n.subscribe("/rc/enabled", 1000, callbackRcEnabled);

    std::vector<std::vector<double>> points;

    // penfeld
    /*double lat_0  = 48.40157318;
    double lon_0 = -4.519289017;
    double lat_1 = 48.40166092;
    double lon_1 = -4.519164562;
    double lat_2 = 48.40177155;
    double lon_2 = -4.519581318;
    double lat_3  = 48.40175247;
    double lon_3 = -4.51984024;*/


    // guerledan
    double lat_0 = 48.200100;
    double lon_0 = -3.016789;
    double lat_1 = 48.200193;
    double lon_1 = -3.015264;
    double lat_2 = 48.198639;
    double lon_2 = -3.015064;
    double lat_3 = 48.198763;
    double lon_3 = -3.016315;

    float x_wp[4];
    float y_wp[4];

    LatLonToUTMXY(lat_0,lon_0,0,y_wp[0],x_wp[0]);
    LatLonToUTMXY(lat_1,lon_1,0,y_wp[1],x_wp[1]);
    LatLonToUTMXY(lat_2,lon_2,0,y_wp[2],x_wp[2]);
    LatLonToUTMXY(lat_3,lon_3,0,y_wp[3],x_wp[3]);

    pos_x_init = x_wp[0];
    pos_y_init = y_wp[0];

    for(int i = 0; i < 4;i++)
    {
        x_wp[i] -= pos_x_init;
        y_wp[i] -= pos_y_init;
        points.push_back({x_wp[i],y_wp[i]});
    }
    points.push_back({x_wp[0],y_wp[0]});

    vibes::newFigure("Trajectory");
    vibes::setFigureProperties("Trajectory",vibesParams("x", 40, "y", -80,"width", 500, "height", 500));
    vibes::axisLimits(-100., 100., -100., 100.);
    vibes::drawLine(points,"red[red]");
    ros::spin();

    return 0;
}
