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

using namespace codac;
VIBesFig *fig;

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
#define angle_camera_x 70.*0.5*M_PI/180.
#define angle_camera_y 60.*0.5*M_PI/180.

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

void callbackFix(const sensor_msgs::NavSatFix& msg) {
    //if(autoOn and !switchOn and state != 2) {
    state = 1;

    ROS_WARN("lat: %f | long: %f | stamp: %d", msg.latitude, msg.longitude, msg.header.stamp.sec);
    float pos_y,pos_x;
    LatLonToUTMXY(msg.latitude, msg.longitude, 0, pos_y, pos_x);
    pos_x -= pos_x_init;
    pos_y -= pos_y_init;

    if ((ancien_pos_x != pos_x) || (ancien_pos_y != pos_y)) {
        vibes::drawVehicle(pos_x, pos_y,(current_heading)*180./M_PI,1., vibesParams("figure", "Vision") );
        vibes::drawVehicle(pos_x, pos_y,(current_heading)*180./M_PI,1., vibesParams("figure", "Trajectory") );

        Interval r(3.,4.);
        double r_min = 3, r_max=4;
        double th_min = -35 + (current_heading)*180./M_PI;
        double th_max =  35 + (current_heading)*180./M_PI;
        Interval theta(-M_PI/10.,M_PI/10.);

        // rotation matrix of the robot, where is the camera
        float rot_x=0., rot_y=0., rot_z=0.;  // angle in radians
        auto R = Eigen::AngleAxis<float>(rot_z, Eigen::Vector3f::UnitZ())
          * Eigen::AngleAxis<float>(rot_y, Eigen::Vector3f::UnitY())
          * Eigen::AngleAxis<float>(rot_x, Eigen::Vector3f::UnitX());

        std::vector<Eigen::Vector2f> pts_img = {
            Eigen::Vector2f(-1, 1),
            Eigen::Vector2f(-1,-1),
            Eigen::Vector2f( 1,-1),
            Eigen::Vector2f( 1, 1),
            Eigen::Vector2f(-1, 1)
        };

//        std::vector<Eigen::Vector2f>* pts_water = new std::vector<Eigen::Vector2f>;
        std::vector<Eigen::Vector2f> pts_water;
        pts_water.reserve(5);

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

        vibes::drawPie(pos_x, pos_y, r_min, r_max, th_min, th_max, "blue[blue]", vibesParams("figure", "Vision"));
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
//    double lat_0  = 48.40157318;
//    double lon_0 = -4.519289017;
//    double lat_1 = 48.40166092;
//    double lon_1 = -4.519164562;
//    double lat_2 = 48.40177155;
//    double lon_2 = -4.519581318;
//    double lat_3  = 48.40175247;
//    double lon_3 = -4.51984024;


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
    vibes::setFigureProperties("Trajectory",vibesParams("x", 100, "y", 100,"width", 1000, "height", 1000));
    vibes::axisLimits(-100., 100., -100., 100.);
    vibes::drawLine(points,"red[red]");
    ros::spin();

    return 0;
}
