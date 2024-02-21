/**
* @file mavros_fly_mission_node.cpp
* @brief Demonstration of Flying Waypoint missions using MAVROS
* @author Shakthi Prashanth M <shakthi.prashanth.m@intel.com>
* @version 0.0.1
* @date 2017-09-01
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/CommandCode.h>
#include <list>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <geometry_msgs/Vector3.h>


mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
	bool connected = current_state.connected;
	bool armed = current_state.armed;
	ROS_INFO("%s", armed ? "" : "Disarmed");
}

geometry_msgs::Vector3 vec(double x, double y, double z){
    geometry_msgs::Vector3 v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_set_mode");
    ros::NodeHandle nh;
    
    mavros_msgs::SetMode auto_set_mode;
    auto_set_mode.request.custom_mode = "AUTO.MISSION";
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient wp_client = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
    mavros_msgs::WaypointPush wp_push_srv; // List of Waypoints
    mavros_msgs::Waypoint wp;
    /*
        uint8 FRAME_GLOBAL=0
        uint8 FRAME_LOCAL_NED=1
        uint8 FRAME_MISSION=2
        uint8 FRAME_GLOBAL_REL_ALT=3
        uint8 FRAME_LOCAL_ENU=4
        uint8 frame
        uint16 command
        bool is_current
        bool autocontinue
        float32 param1
        float32 param2
        float32 param3
        float32 param4
        float64 x_lat
        float64 y_long
        float64 z_alt
    */

    try{
        GeographicLib::LocalCartesian transform(47.39, 8.545);
        // GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();
            // GeographicLib::Geocentric earth;//(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
    // {
    // double lat = 27.99, lon = 86.93, h = 8820; // Mt Everest
    // double X, Y, Z;
    // earth.Forward(lat, lon, h, X, Y, Z);
    // std::cout << floor(X / 1000 + 0.5) << " "
    //        << floor(Y / 1000 + 0.5) << " "
    //        << floor(Z / 1000 + 0.5) << "\n";
    // }
    {
      // Sample reverse calculation
      double X = 10, Y = 20, Z = 5;
      double lat, lon, h;
      transform.Reverse(X, Y, Z, lat, lon, h);
      std::cout << lat << " " << lon << " " << h << "\n";
    }
    }catch(const std::exception& e){
        std::cerr << "Caught exception: " << e.what() << "\n";
    }



    std::vector<geometry_msgs::Vector3> path;
    path.push_back(vec(47.3978206, 8.545987, 5));
    path.push_back(vec(47.3972527, 8.5457917, 5));
    path.push_back(vec(47.3977783, 8.545906, 5));

    for(uint32_t i = 0; i < path.size(); i++){
        wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        wp.command = i == 0 ? mavros_msgs::CommandCode::NAV_TAKEOFF : mavros_msgs::CommandCode::NAV_WAYPOINT;
        wp.is_current = i == 0;
        wp.autocontinue = true;
        wp.x_lat = path[i].x;
        wp.y_long = path[i].y;
        wp.z_alt = path[i].z;
        wp_push_srv.request.waypoints.push_back(wp);
    }

    // Create RTL waypoint
    wp.frame = mavros_msgs::Waypoint::FRAME_MISSION;
    wp.command = mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
    wp.x_lat = wp.y_long = wp.z_alt = 0;
    wp_push_srv.request.waypoints.push_back(wp);

    // WP 0
//     wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
//     wp.command        = mavros_msgs::CommandCode::NAV_TAKEOFF;
//     wp.is_current     = true;
//     wp.autocontinue   = true;
//     wp.x_lat          = 47.3978206;
//     wp.y_long         = 8.543987;
//     wp.z_alt          = 10;
//     wp_push_srv.request.waypoints.push_back(wp);
//     // WP 1
//     wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
//     wp.command        = mavros_msgs::CommandCode::NAV_LOITER_TIME;
//     wp.is_current     = false;
//     wp.autocontinue   = true;
//     wp.x_lat          = 47.3962527;
//     wp.y_long         = 8.5467917;
//     wp.z_alt          = 20;
// 	wp.param1			= 10;
// 	wp.param3			= 2;
// 	wp.param4			= 1;
//     wp_push_srv.request.waypoints.push_back(wp);
//
//     // WP 2
//     wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
//     wp.command        = mavros_msgs::CommandCode::NAV_WAYPOINT;
//     wp.is_current     = false;
//     wp.autocontinue   = true;
//     wp.x_lat          = 47.3977783;
//     wp.y_long         = 8.547906;
//     wp.z_alt          = 20;
//     wp_push_srv.request.waypoints.push_back(wp);
//
//     // WP 3
//     wp.frame          = mavros_msgs::Waypoint::FRAME_MISSION;
//     wp.command        = mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
//     wp.is_current     = false;
//     wp.autocontinue   = true;
//     wp.x_lat          = 0;
//     wp.y_long         = 0;
//     wp.z_alt          = 0;
//     wp_push_srv.request.waypoints.push_back(wp);

    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected to PX4!");
    // ARM
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) &&
        arm_cmd.response.success){
        ROS_INFO("Vehicle armed");
    }

    // Send WPs to Vehicle
    if (wp_client.call(wp_push_srv)) {
        ROS_INFO("Send waypoints ok: %d", wp_push_srv.response.success);
        if (current_state.mode != "AUTO.MISSION") {
            if( set_mode_client.call(auto_set_mode) && auto_set_mode.response.mode_sent){
                ROS_INFO("AUTO.MISSION enabled");
            }
        }
    }
    else
        ROS_ERROR("Send waypoints FAILED.");

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
