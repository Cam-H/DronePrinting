/**
 * @file build_path.cpp
 * @brief
 */

#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/String.h>
#include <thread>

#include <iostream>
#include <fstream>
#include <string>
#include <regex>
#include <vector>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "build_path_server");
    ros::NodeHandle nh;

    ros::ServiceClient gazebo_client = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel spawn_cmd;
    spawn_cmd.request.model_xml = "<sdf version ='1.4'>\
          <model name ='sphere'>\
            <pose>1 0 0 0 0 0</pose>\
            <static>false</static>\
          </model>\
        </sdf>";
        // spawn_cmd.request.initial_pose = geometry_msgs::PoseStamped();
    // spawn_cmd

    for(int i = 0; i < 100; i++){
        spawn_cmd.request.model_name = "particles_" + std::to_string(i);
        std::cout << spawn_cmd.request.model_name << "\n";
        gazebo_client.call(spawn_cmd);

    }

    // arm_cmd.request.value = true;
    // std::cout << "|" << spawn_cmd.response.success << "=" << spawn_cmd.response.status_message << "|\n";

    return 0;
}
