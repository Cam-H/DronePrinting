#include <ros/ros.h>

#include <std_msgs/String.h>
#include <thread>

#include <iostream>
#include <fstream>
#include <string>
#include <regex>
#include <vector>
#include <cmath>

#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/CommandCode.h>

#include <GeographicLib/LocalCartesian.hpp>

#include "wasp_description/RequestMission.h"

struct Pose{
    double x;
    double y;
    double z;
    bool active;

    bool operator==(const Pose& a){
        return x == a.x && y == a.y && z == a.z;
    }

    bool operator!=(const Pose& a){
        return !(*this == a);
    }

    Pose operator/=(double scalar) const {
        return {x / scalar, y / scalar, z / scalar, active};
    }

    static double distance(const Pose& a, const Pose& b){
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
    }

};

class MissionServer{
public:

    MissionServer();
    void loadParameters();

    void command_monitor();

    void stats();

    std::vector<Pose> load(const std::string& filepath, bool fallback = false);
    std::vector<Pose> square(double x, double y, double length, double width, double height);

private:

    bool mission(wasp_description::RequestMission::Request &req, wasp_description::RequestMission::Response& res);

    mavros_msgs::Waypoint poseWaypoint(const Pose& pose, double acceptRadius = -1);
    mavros_msgs::Waypoint speedWaypoint(double speed);

    double acceptanceRadius(double speed);

private:

    ros::ServiceServer m_MissionService;

    std::vector<Pose> m_Path;

    uint32_t m_Step;

    GeographicLib::LocalCartesian transform;


    /** CONFIGURATION PARAMETERS **/
    std::string filepath_;
    double m_TargetSpeed;
    double m_LayerHeight;
    double m_AcceptRadius;
    double zOffset;

    double ox = 47.39774;
    double oy = 8.54559;
};
