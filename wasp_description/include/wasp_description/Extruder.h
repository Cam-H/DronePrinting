#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <mavros_msgs/ActuatorControl.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <thread>

class Extruder {
public:

    Extruder(ros::NodeHandle nh);
    void loadParameters();

    void publish();

private:
    void extrusion_cb(const std_msgs::Bool::ConstPtr& msg);
private:

    ros::Subscriber m_SubExtrusion;
    ros::Publisher m_PubExtrusion;

};
