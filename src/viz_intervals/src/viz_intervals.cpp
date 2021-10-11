#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

#include <iostream>
#include <cmath>

#include "UTM.h"
#undef pi

#include <proj.h>
#include <codac.h>
#include <Eigen/Core>
#include <Eigen/Geometry>


codac::VIBesFig *fig;

bool switchOn = false;
bool autoOn = false;
int state = 0;

double pos_x_init;
double pos_y_init;

double ancien_pos_x;
double ancien_pos_y;

//rosbag play bag_2021-05-18-15-31-36.bag -r 5  -s 70 -u 300
double current_heading;

// angle of the camera view
float angle_camera_x = 70.*0.5*M_PI/180.;
float angle_camera_y = 60.*0.5*M_PI/180.;

Eigen::Vector2f proj_cam_water(Eigen::Vector3f pos, Eigen::Quaternion<float> R, Eigen::Vector2f pos_img, float dist_max) {
    Eigen::Vector3f target {1, -pos_img[0] * tan(angle_camera_x), -pos_img[1]*tan(angle_camera_y)};
    target = target.normalized();
    target = R * target;

    Eigen::Vector3f center;

    if (target[2] < 0) {  // z < 0
        if (-pos[2] / target[2] < dist_max) {
            center = -pos[2]/target[2]*target + pos;
            return Eigen::Vector2f(center[0], center[1]);
        }
    }

    target[2] = 0.;

    if (target.norm() == 0) {
        return Eigen::Vector2f(pos[0], pos[1]);
    }

    target = target.normalized();

    center = dist_max*target + pos;

    return Eigen::Vector2f(center[0], center[1]);
}


void callbackRcEnabled(const std_msgs::Bool& msg) {
    switchOn = msg.data;
}

void callbackCmd(const geometry_msgs::Twist& msg) {
    autoOn = true;
}

void callbackHeading(const std_msgs::Float64& msg) {
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

float angle_3_pts(float c_x, float c_y, std::vector<double> p1, std::vector<double> p2){
    //compute the angle between 3 points, the center being the robot in this context
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

void callbackFix(const sensor_msgs::NavSatFix& msg) {
    //if(autoOn and !switchOn and state != 2) {
    state = 1;
    std::vector<std::vector<double>> zone;

    float pos_y,pos_x;
    LatLonToUTMXY(msg.latitude, msg.longitude, 0, pos_y, pos_x);
    pos_x -= pos_x_init;
    pos_y -= pos_y_init;

    if ((ancien_pos_x != pos_x) || (ancien_pos_y != pos_y)) {
        ROS_WARN("lat: %f | long: %f | stamp: %d", msg.latitude, msg.longitude, msg.header.stamp.sec);


        codac::Interval r(3.,4.);
        double r_min = 3, r_max=4;
        double th_min = -35 + (current_heading)*180./M_PI;
        double th_max =  35 + (current_heading)*180./M_PI;
        codac::Interval theta(-M_PI/10.,M_PI/10.);


        // rotation matrix of the robot, where is the camera
        float rot_x=0., rot_y=0., rot_z=0.;  // angle in radians
        auto R = Eigen::AngleAxis<float>(rot_z, Eigen::Vector3f::UnitZ())
          * Eigen::AngleAxis<float>(rot_y, Eigen::Vector3f::UnitY())
          * Eigen::AngleAxis<float>(rot_x, Eigen::Vector3f::UnitX());

        // points in the image that we want to get the position in real world
        std::vector<Eigen::Vector2f> pts_img = {
            Eigen::Vector2f(-1, 1),
            Eigen::Vector2f(-1,-1),
            Eigen::Vector2f( 1,-1),
            Eigen::Vector2f( 1, 1),
            Eigen::Vector2f(-1, 1)
        };

        std::vector<Eigen::Vector2f> pts_water;
        pts_water.reserve(5);  // change here according to the name of lines beings used in the camera approximation

        Eigen::Vector3f pos{pos_x, pos_y, 1.0};

        double dist_max = 10.;

        for (Eigen::Vector2f pt_img : pts_img) {
            Eigen::Vector2f pt_water = proj_cam_water(pos, R, pt_img, dist_max);
            pts_water.push_back(pt_water);
        }

        std::cout << "extremes of the area seen ";
        for (Eigen::Vector2f pt_water : pts_water) {
            std::cout << pt_water[0] << " " << pt_water[1] << " | ";
        }
        std::cout << endl;

        zone.push_back({pos_x + 3,pos_y + 2});
        zone.push_back({pos_x + 6,pos_y + 4});
        zone.push_back({pos_x - 6,pos_y + 4});
        zone.push_back({pos_x-3,pos_y + 2});
        zone.push_back({pos_x + 3,pos_y + 2});

        float r1 = radius(pos_x, pos_y, zone[0], zone[3]);
        float r2 = radius(pos_x, pos_y, zone[1], zone[2]);
        float angle = angle_3_pts(pos_x, pos_y, zone[3], zone[0]);

        zone[0] = rotate_pt(zone[0], -current_heading+M_PI/2, pos_x, pos_y);
        zone[1] = rotate_pt(zone[1], -current_heading+M_PI/2, pos_x, pos_y);
        zone[2] = rotate_pt(zone[2], -current_heading+M_PI/2, pos_x, pos_y);
        zone[3] = rotate_pt(zone[3], -current_heading+M_PI/2, pos_x, pos_y);
        zone[4] = rotate_pt(zone[4], -current_heading+M_PI/2, pos_x, pos_y);

        vibes::drawVehicle(pos_x, pos_y,(current_heading)*180./M_PI,1., vibesParams("figure", "Vision") );
        vibes::drawVehicle(pos_x, pos_y,(current_heading)*180./M_PI,1., vibesParams("figure", "Trajectory") );

        vibes::drawPie(pos_x, pos_y, r1, r2, -(angle/2)+(current_heading)*180./M_PI, angle/2+(current_heading)*180./M_PI, "blue[blue]", vibesParams("figure", "Trajectory"));
        vibes::drawLine(zone,"red[red]", vibesParams("figure", "Trajectory"));

//        vibes::drawPie(pos_x, pos_y, r_min, r_max, th_min, th_max, "blue[blue]", vibesParams("figure", "Vision"));
    }

    ancien_pos_x = pos_x;
    ancien_pos_y = pos_y;
    //}

    if(state == 1 and switchOn)
    {
        state = 2;
    }
}


int main(int argc, char **argv) {
    vibes::beginDrawing();
    vibes::newFigure("Vision");

    ros::init(argc, argv, "viz_intervals_node");

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

    for(int i = 0; i < 4;i++) {
        x_wp[i] -= pos_x_init;
        y_wp[i] -= pos_y_init;
        points.push_back({x_wp[i],y_wp[i]});
    }
    points.push_back({x_wp[0],y_wp[0]});

    vibes::newFigure("Trajectory");
    vibes::setFigureProperties("Trajectory",vibesParams("x", 100, "y", 100,"width", 800, "height", 800));
    vibes::axisLimits(-250., 250., -250., 250.);
    vibes::drawLine(points,"red[red]");
    ros::spin();

    return 0;
}
