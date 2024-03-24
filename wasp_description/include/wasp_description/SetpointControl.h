#include <ros/ros.h>

#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <mavros_msgs/ActuatorControl.h>

#include <mavros_msgs/CommandCode.h>

#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointReached.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <std_msgs/Bool.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <thread>

#include "wasp_description/RequestMission.h"

class SetpointControl {
public:

    enum Mode {
        DISARM = 0, PRINTING, POSITION, VELOCITY, ACCELERATION
    };

    SetpointControl(ros::NodeHandle nh);
    void loadParameters();

    void start();
    void command_monitor();

    void update();
    void publish();

    bool isReady();
    bool inProcess();
private:
    void swapMode(const std::string& str);
    void reqMission();

    void state_cb(const mavros_msgs::State::ConstPtr& msg);

    void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);

    void waypoint_reached_cb(const mavros_msgs::WaypointReached::ConstPtr& msg);

private:

    Mode m_Mode;
    bool m_Ready;
    bool m_InProcess;

    geometry_msgs::Vector3 m_LinearTarget;
    geometry_msgs::Vector3 m_AngularTarget;

    geometry_msgs::PoseStamped m_LocalPose;
    geometry_msgs::TwistStamped m_LocalVel;

    /** ROS COMMUNICATIONS **/
    ros::Subscriber m_SubLocalPose;
    ros::Subscriber m_SubLocalVel;

    ros::Subscriber m_SubState;
    mavros_msgs::State m_CurrentState;

    ros::Subscriber m_SubWaypointReached;
    std::vector<uint32_t> m_WaypointCtrl;
    uint32_t m_WaypointCtrlIdx;
    uint32_t m_WaypointCount;
    uint32_t m_LastWaypoint;

    int m_BaseFlightTime;

    ros::Publisher m_PubActuator;
    double gm;
    double control;

    ros::Publisher m_PubLocalPose;
    ros::Publisher m_PubLocalVel;

    ros::Publisher m_PubExtrusion;

    ros::ServiceClient m_HomeClient;
    ros::ServiceClient m_ArmingClient;
    ros::ServiceClient m_SetModeClient;


    ros::ServiceClient m_MissionClient;

    ros::ServiceClient m_WPPushClient;
    ros::ServiceClient m_WPClearClient;

};
