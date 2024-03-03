#include "wasp_description/MissionServer.h"

#include "wasp_description/WaspUtils.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_server");

    MissionServer ms;

    ros::spin();

    return 0;
}

MissionServer::MissionServer() : m_Step(0) {

    ros::NodeHandle nh;
    m_MissionService = nh.advertiseService("request_mission", &MissionServer::mission, this);
    loadParameters();

    if(filepath_.length() > 0){
        m_Path = load(filepath_);
    }else{

        // Generate square path when non gcode is provided
        for(double z = 0.0; z < 1; z+=0.01){
            m_Path.push_back({0, 5, z, 1});
            m_Path.push_back({0, 10, z, 1});
            m_Path.push_back({5, 10, z, 1});
            m_Path.push_back({5, 5, z, 1});
        }
    }
    std::cout << acceptanceRadius(targetSpeed) << "\n";
    ROS_INFO("Ready to serve build paths");
}

void MissionServer::loadParameters(){

    ros::param::param <std::string>("~path", filepath_, "");

    ros::param::param <double>("~speed", targetSpeed, 0.1);
    ros::param::param <double>("~zOffset", zOffset, 0.3);

    ros::param::param <double>("~gx", ox, 47.39774);
    ros::param::param <double>("~gy", oy, 8.54559);
    transform = GeographicLib::LocalCartesian(ox, oy);
}

std::vector<Pose> MissionServer::load(const std::string& filepath){

    std::fstream file;
    std::vector<Pose> path = {Pose{0, 0, 0, false}};

    // ROS_INFO("Loading: %s", filepath);
    std::cout << filepath << "\n";
    file.open(filepath, std::ios::in);
    if (file.is_open()){
        std::string line;
        bool active = false, activated = false;
        while(std::getline(file, line)){ //read data from file object and put it into string.

            // Remove g-code comments
            std::string::size_type idx = line.find(';');
            line = (idx == std::string::npos) ? line : line.substr(0, idx);
            if(line.length() == 0){
                continue;
            }

            Pose pose = path[path.size() - 1];

            // Separate line content into separate control items
            std::regex word_regex("(\\w+\\d+\\.?\\d*)");
            auto words_begin = std::sregex_iterator(line.begin(), line.end(), word_regex);
            auto words_end = std::sregex_iterator();

            int ignores = 0;
            for (std::sregex_iterator i = words_begin; i != words_end; ++i){
                std::string key = (*i).str();

                // Handle co-ordinate definitions
                switch(key[0]){
                    case 'X':
                        updateField(key.substr(1), pose.x);
                        break;
                    case 'Y':
                        updateField(key.substr(1), pose.y);
                        break;
                    case 'Z':
                        updateField(key.substr(1), pose.z);
                        break;
                    case 'E':
                        active = activated = true;
                        break;
                    case 'F':
                        active = false;
                        break;
                    default:
                        ignores++;
                }

            }

            // Only record new positions
            pose.active = active;
            if(activated && ignores != std::distance(words_begin, words_end) && pose != path[path.size() - 1]){
                // std::cout << pose.x << " " << pose.y << " " << pose.z << " " << pose.active << "\n";
                path.push_back(pose /= 15);
            }

        }

        // Close the file for cleanup
        file.close();

    }

    return path;
}

bool MissionServer::mission(wasp_description::RequestMission::Request &req, wasp_description::RequestMission::Response& res){

    // Calculate an appropriate mission length based on requested flight time
    uint32_t last = m_Step + 1;//TODO properly calculate heading time
    double travel = 0, target = req.flighttime * targetSpeed;
    for(; last < m_Path.size() - 1; last++){
        double delta = Pose::distance(m_Path[last], m_Path[last + 1]);
        if(travel + delta > target) break;
        travel += delta;
    }

    if(travel <= 0) return false;
    std::cout << m_Step << "-" << last << " / " << m_Path.size() << " | Travel: " << travel << " / " << target;

    /*************** PREPARE MISSION ********************/
    mavros_msgs::Waypoint wp;
    wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    wp.autocontinue = true;

    // Launch waypoint
    wp.is_current = true;
    wp.command = mavros_msgs::CommandCode::NAV_TAKEOFF;
    wp.x_lat = ox;
    wp.y_long = oy;
    wp.z_alt = 1;
    res.waypoints.push_back(wp);

    wp.is_current = false;

    // Fly to initial print position
    res.waypoints.push_back(poseWaypoint(m_Path[m_Step]));

    bool active = false;
    std::cout << "NPATH:\n";
    // Print path waypoints
    for(uint32_t i = m_Step + 1; i <= last; i++){
        if(m_Path[i].active != active){
            active = m_Path[i].active;
            res.waypoints.push_back(speedWaypoint(active ? targetSpeed : -2));
            res.ctrl.push_back(res.waypoints.size() - 1);
        }

        if(active) res.waypoints.push_back(poseWaypoint(m_Path[i], acceptanceRadius(targetSpeed)));
        else res.waypoints.push_back(poseWaypoint(m_Path[i]));

    }

    // Return to default speed
    res.waypoints.push_back(speedWaypoint(-2));
    res.ctrl.push_back(res.waypoints.size() - 1);// Indicate print termination

    // Create RTL waypoint
    wp.frame = mavros_msgs::Waypoint::FRAME_MISSION;
    wp.autocontinue = false;
    wp.command = mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
    wp.x_lat = wp.y_long = wp.z_alt = 0;
    res.waypoints.push_back(wp);

    m_Step = last;

    std::cout << " | Served: " << res.waypoints.size() << " waypoints\n";
    for(uint32_t idx : res.ctrl){
        std::cout << idx << " ";
    }std::cout << "\n";

    return true;
}

mavros_msgs::Waypoint MissionServer::poseWaypoint(const Pose& pose, double acceptRadius){
    mavros_msgs::Waypoint wp;
    wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    wp.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
    wp.autocontinue = true;

    if(acceptRadius != -1) wp.param2 = acceptRadius;
    wp.param4 = NAN;

    double lat, lon, alt;
    transform.Reverse(pose.x, pose.y, pose.z + zOffset, lat, lon, alt);
    std::cout << pose.x << " " << pose.y << " " << pose.z << "\n";

    wp.x_lat = lat;
    wp.y_long = lon;
    wp.z_alt = alt;

    return wp;
}

mavros_msgs::Waypoint MissionServer::speedWaypoint(double speed){
    mavros_msgs::Waypoint wp;
    wp.frame = mavros_msgs::Waypoint::FRAME_MISSION;
    wp.command = mavros_msgs::CommandCode::DO_CHANGE_SPEED;
    wp.autocontinue = true;
    wp.param1 = 1;// Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
    wp.param2 = speed;// Speed (m/s, -1 indicates no change)
    wp.param3 = -1;// Throttle ( Percent, -1 indicates no change)
    return wp;
}

double MissionServer::acceptanceRadius(double speed){
    return 0.2 * (1 - exp(-speed)) + 0.05;
}
