/**
 * @file build_path.cpp
 * @brief
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/String.h>
#include <thread>

#include <iostream>
#include <fstream>
#include <string>
#include <regex>
#include <vector>

#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/CommandCode.h>

#include <GeographicLib/LocalCartesian.hpp>

#include "wasp_description/NextPose.h"
#include "wasp_description/RequestMission.h"

std::vector<geometry_msgs::Vector3> s_Path;
int step = 0;
double TARGET_SPEED = 0.1;

bool isNum(const std::string& str){
        std::string::const_iterator it = str.begin();
        while (it != str.end() && (std::isdigit(*it) || (it == str.begin() && *it == '-') || *it == '.')) ++it;
        return !str.empty() && it == str.end();
    }

    void updateField(std::string value, double& field){
        if(isNum(value)){
            field = std::stod(value) / 15;//TODO handle proper scaling
        }
    }

double distance(const geometry_msgs::Vector3& a, const geometry_msgs::Vector3& b){
    double dx = a.x - b.x,
           dy = a.y - b.y,
           dz = a.z - b.z;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

geometry_msgs::Vector3 vec(double x, double y, double z){
    geometry_msgs::Vector3 v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
}

std::vector<geometry_msgs::Vector3> load(const std::string& filepath){

    std::fstream file;
    std::vector<geometry_msgs::Vector3> path = {geometry_msgs::Vector3()};

    // ROS_INFO("Loading: %s", filepath);
    std::cout << filepath << "\n";
    file.open(filepath, std::ios::in);
    if (file.is_open()){
        std::string line;
        while(std::getline(file, line)){ //read data from file object and put it into string.

            // Remove g-code comments
            std::string::size_type idx = line.find(';');
            line = (idx == std::string::npos) ? line : line.substr(0, idx);
            if(line.length() == 0){
                continue;
            }

            geometry_msgs::Vector3 pos = path[path.size() - 1];

            // Separate line content into separate control items
            std::regex word_regex("(\\w+\\d+\\.?\\d*)");
            auto words_begin = std::sregex_iterator(line.begin(), line.end(), word_regex);
            auto words_end = std::sregex_iterator();

            for (std::sregex_iterator i = words_begin; i != words_end; ++i){
                std::string key = (*i).str();

                // Handle co-ordinate definitions
                switch(key[0]){
                    case 'X':
                        updateField(key.substr(1), pos.x);
                        break;
                    case 'Y':
                        updateField(key.substr(1), pos.y);
                        break;
                    case 'Z':
                        updateField(key.substr(1), pos.z);
                        break;
                }

            }

            // Only record new positions
            if(pos != path[path.size() - 1]){
                path.push_back(pos);
            }

        }

        // Close the file for cleanup
        file.close();

    }

    return path;
}

// bool next(wasp_description::NextPose::Request &req, wasp_description::NextPose::Response& res){
//     if(step == s_Path.size()){
//         step = 0;
//     }
//
//     std::cout << "[" << (step + 1) << " / " << s_Path.size() << "]\n";
//     // ROS_INFO("[%ld / %ld]", step + 1, s_Path.size());
//     res.pos = s_Path[step++];
//     res.pos.z += 0.5;//TODO z offset
//     return true;
// }

    // path.push_back(vec(47.3978206, 8.545987, 5));
    // path.push_back(vec(47.3972527, 8.5457917, 5));
    // path.push_back(vec(47.3977783, 8.545906, 5));

double lx = 47.39774, ly = 8.54559;
GeographicLib::LocalCartesian transform(lx, ly);
bool mission(wasp_description::RequestMission::Request &req, wasp_description::RequestMission::Response& res){

    // Calculate an appropriate mission length based on requested flight time
    uint32_t last = step;
    double travel = 0, target = req.flighttime * TARGET_SPEED;
    for(; last < s_Path.size() - 1; last++){
        double delta = distance(s_Path[last], s_Path[last + 1]);
        if(travel + delta > target) break;
        travel += delta;
    }

    if(travel <= 0) return false;
    std::cout << step << "-" << last << " / " << s_Path.size() << " | Travel: " << travel << " / " << target << "\n";

    /*************** PREPARE MISSION ********************/
    mavros_msgs::Waypoint wp;
    wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    wp.autocontinue = true;

    // Launch waypoint
    wp.is_current = true;
    wp.command = mavros_msgs::CommandCode::NAV_TAKEOFF;
    wp.x_lat = lx;
    wp.y_long = ly;
    wp.z_alt = 1;
    res.waypoints.push_back(wp);

    // Build path waypoints
    double lat, lon, alt, llat = -1, llon = -1, lalt = -1;
    wp.is_current = false;
    wp.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
    for(uint32_t i = step; i <= last; i++){
        transform.Reverse(s_Path[i].x, s_Path[i].y, s_Path[i].z + 0.2/*TODO get proper offset*/, lat, lon, alt);

        // Skip duplicate positions (May arise due to accuracy errors)
        if(lat == llat && lon == llon && alt == lalt){
            continue;
        }

        wp.x_lat = llat = lat;
        wp.y_long = llon = lon;
        wp.z_alt = lalt = alt;
        res.waypoints.push_back(wp);
        // std::cout << i << " " << s_Path[i].x << " " << s_Path[i].y << " " << s_Path[i].z << " -> " << lat << " " << lon << " " << alt << "\n";
    }

    // Create RTL waypoint
    wp.frame = mavros_msgs::Waypoint::FRAME_MISSION;
    wp.autocontinue = false;
    wp.command = mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
    wp.x_lat = wp.y_long = wp.z_alt = 0;
    res.waypoints.push_back(wp);

    step = last;

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_server");
    ros::NodeHandle nh;

    std::string filepath = "src/wasp_description/res/t2.gcode";
    if(argc == 2){
        filepath = argv[1];
    }

    s_Path = load(filepath);

    ros::ServiceServer service = nh.advertiseService("request_mission", mission);

    ROS_INFO("Ready to serve build paths");
    ros::spin();


    return 0;
}
