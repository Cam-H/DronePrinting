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

#include "wasp_description/WaspUtils.h"

struct Segment{
    uint32_t m_I0;
    uint32_t m_I1;

    uint32_t level;
    bool complete;
};

class MissionServer{
public:

    MissionServer();
    void loadParameters();

    void command_monitor();

    void stats();

    void load(std::vector<vec3>& vertices, std::vector<Segment>& segments, const std::string& filepath, bool fallback = false);
    void square(std::vector<vec3>& vertices, std::vector<Segment>& segments, double x, double y, double length, double width, double height);

private:

    bool mission(wasp_description::RequestMission::Request &req, wasp_description::RequestMission::Response& res);

    mavros_msgs::Waypoint poseWaypoint(const vec3& pose, double acceptRadius = -1);
    mavros_msgs::Waypoint speedWaypoint(double speed);

    double acceptanceRadius(double speed);

private:

    ros::ServiceServer m_MissionService;

    std::vector<vec3> m_Vertices;
    std::vector<Segment> m_Segments;

    uint32_t m_Step;

    GeographicLib::LocalCartesian transform;


    /** CONFIGURATION PARAMETERS **/
    std::string filepath_;
    double m_TargetSpeed;
    double m_LayerHeight;
    double m_AcceptRadius;
    double m_ZOffset;

    double ox = 47.39774;
    double oy = 8.54559;
};
