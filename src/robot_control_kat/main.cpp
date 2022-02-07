#include <QApplication>
#include "robotcontrol.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <cmath>
//char *argv_2[1];
//int *argc_2;
//argc_2 = 1;
/*
QApplication app2;

void callbackFix(const sensor_msgs::NavSatFix& msg)
{
    ROS_WARN("lat: %f | long: %f | stamp: %f", msg.latitude, msg.longitude);
    RobotControl rbctrl;
    Q_EMIT rbctrl.drawCircleSignal(QGeoCoordinate(48.202037,-3.016226),3,"kat","blue");
    rbctrl.show();
   // app2.exec();
}

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    
    //app2 = app;
    std::cout<<argv[0]<<std::endl;
    ros::init(argc, argv, "Dist_node");
    ros::NodeHandle n;

    

    
    //QApplication app(argc, argv);
    //RobotControl rbctrl;
    
    
    //app.exec();
    ros::Subscriber sub_pos = n.subscribe("/fix", 1000, callbackFix);
    //app.exit();
    ros::spin();
    return 0;
}
*/
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    RobotControl rbctrl;
    rbctrl.show();
    app.exec();
    return 0;
}

