#include "wasp_description/SetpointControl.h"

#include "wasp_description/WaspUtils.h"

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "wasp_control");

    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // std::thread th1(&PoseEntry::monitor, &pe);
    ROS_INFO("FCU Connection wait...");


    // Wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("FCU Connection successful");

    SetpointControl sc(nh);

    // Send a few setpoints before starting
    for(int i = 0; ros::ok() && i < 10; i++){
        sc.publish();
        ros::spinOnce();
        rate.sleep();
    }

    // Call main control update thread periodically
    while(ros::ok()){
        sc.update();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

SetpointControl::SetpointControl(ros::NodeHandle nh) : m_Mode(Mode::DISARM){

    m_LinearTarget = m_AngularTarget = geometry_msgs::Vector3();

    m_SubLocalPose = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &SetpointControl::local_pose_cb, this);
    m_SubLocalVel = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity", 10, &SetpointControl::local_vel_cb, this);

    m_SubState = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &SetpointControl::state_cb, this);

    m_SubWaypointReached = nh.subscribe<mavros_msgs::WaypointReached>("mavros/mission/reached", 10, &SetpointControl::waypoint_reached_cb, this);

    m_PubLocalPose = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    m_PubLocalVel = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    m_PubExtrusion = nh.advertise<std_msgs::Bool>("wasp/extrusion", 10);

    m_ArmingClient = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    m_SetModeClient = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    m_MissionClient = nh.serviceClient<wasp_description::RequestMission>("request_mission");

    m_WPPushClient = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
    m_WPClearClient = nh.serviceClient<mavros_msgs::WaypointClear>("mavros/mission/clear");

    loadParameters();

    init();
}

void SetpointControl::init(){
    m_Extruding = m_InProcess = false;

    if(!m_MonitorAlive){
        m_Thread = std::thread(&SetpointControl::command_monitor, this);
        m_MonitorAlive = true;
    }

    m_WaypointCtrlIdx = m_LastWaypoint = m_WaypointCount = 0;
    m_WaypointCtrl.clear();

    swapMode(m_Manual ? "mode position" : "mode print");
}

void SetpointControl::loadParameters(){
    ros::param::param <int>("~flighttime", m_BaseFlightTime, 240);
    ros::param::param<bool>("~manual", m_Manual, false);
    ros::param::param<bool>("~sim", m_Sim, false);
}

void SetpointControl::local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    m_LocalPose = *msg;
}

void SetpointControl::local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    m_LocalVel = *msg;
}

void SetpointControl::state_cb(const mavros_msgs::State::ConstPtr& msg){
    m_CurrentState = *msg;

    if(m_CurrentState.mode == "AUTO.MISSION"){
        m_InProcess |= m_CurrentState.armed;

        if(!m_CurrentState.armed && missionComplete()){
            finish();
        }
    }
}

void SetpointControl::waypoint_reached_cb(const mavros_msgs::WaypointReached::ConstPtr& msg){
    if(m_LastWaypoint != msg->wp_seq){
        m_LastWaypoint = msg->wp_seq;

        if(m_WaypointCtrl[m_WaypointCtrlIdx] <= m_LastWaypoint){
            m_WaypointCtrlIdx++;
            m_Extruding = !m_Extruding;
        }

        std::cout << m_LastWaypoint << " " << m_WaypointCount << " " << m_WaypointCtrlIdx << " " << m_WaypointCtrl[m_WaypointCtrlIdx] << "\n";
    }
}

void SetpointControl::command_monitor(){
    int i = 0;

    std::string input;
    while(!checkToken(input, "q")){

        std::getline(std::cin, input);

        // Convert to lower case to eliminate duplicate conditions
        std::transform(input.begin(), input.end(), input.begin(), [](unsigned char c){ return std::tolower(c); });

        if(input.length() > 0){
            switch(input[0]){
                case 'd':
                    break;
                case 'm':
                    swapMode(input);
                    break;
                case 'x':
                    updateField(input.substr(1), m_LinearTarget.x);
                    break;
                case 'y':
                    updateField(input.substr(1), m_LinearTarget.y);
                    break;
                case 'z':
                    updateField(input.substr(1), m_LinearTarget.z);
                    break;
                case 'i':
                    updateField(input.substr(1), m_AngularTarget.x);
                    break;
                case 'j':
                    updateField(input.substr(1), m_AngularTarget.y);
                    break;
                case 'k':
                    updateField(input.substr(1), m_AngularTarget.z);
                    break;
                case 'p':
                    if(!inProcess()){
                        m_Extruding = !m_Extruding;
                    }
                    break;
            }

            std::cout << m_LinearTarget.x << " " << m_LinearTarget.y << " " << m_LinearTarget.z << "\n";
        }
    }

    m_MonitorAlive = false;
    finish();
}

void SetpointControl::update(){
    static ros::Time last_request = ros::Time::now();

    // if(m_Mode == Mode::DISARM){
    //     return;
    // }

    mavros_msgs::SetMode set_mode;
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    std::string mode = "MANUAL";
    switch(m_Mode){
        case Mode::PRINTING:
            mode = "AUTO.MISSION";
            arm_cmd.request.value = !inProcess();
            break;
        case Mode::POSITION: case Mode::VELOCITY: case Mode::ACCELERATION:
            mode = "OFFBOARD";
            break;
    }
    set_mode.request.custom_mode = mode;


    if(m_CurrentState.mode != mode && (ros::Time::now() - last_request > ros::Duration(5.0))){
        if(m_SetModeClient.call(set_mode) && set_mode.response.mode_sent){
            ROS_INFO("%s enabled", mode.c_str());
        }
        last_request = ros::Time::now();
    } else {
        if(m_CurrentState.armed != arm_cmd.request.value && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if(!inProcess() && m_ArmingClient.call(arm_cmd) && arm_cmd.response.success){

            }

            last_request = ros::Time::now();
        }
    }

    publish();
}


void SetpointControl::swapMode(const std::string& str){
    if(m_Mode == Mode::PRINTING && inProcess()) return;// Prevent mode changing when completing a print run

    if(str == "mode disarm"){
        m_Mode = Mode::DISARM;
    }else if(str == "mode print"){
        if(m_Mode == Mode::PRINTING) return;
        reqMission();
        m_Mode = Mode::PRINTING;
    }else if(str == "mode position"){
        if(m_Mode == Mode::POSITION) return;
        m_LinearTarget.x = m_LocalPose.pose.position.x;
        m_LinearTarget.y = m_LocalPose.pose.position.y;
        m_LinearTarget.z = m_LocalPose.pose.position.z;
        m_Mode = Mode::POSITION;
    }else if(str == "mode velocity"){
        if(m_Mode == Mode::VELOCITY) return;
        m_Mode = Mode::VELOCITY;
        m_LinearTarget = geometry_msgs::Vector3();
    }else if(str == "mode acceleration"){
        if(m_Mode == Mode::ACCELERATION) return;
        m_Mode = Mode::ACCELERATION;
    }
}

bool SetpointControl::clearMission(){
    mavros_msgs::WaypointClear clrsrv;
    return m_WPClearClient.call(clrsrv) && clrsrv.response.success;
}

void SetpointControl::reqMission(){

    if(!clearMission()){
        ROS_WARN("Failed to clear old waypoints");
        return;
    }

    wasp_description::RequestMission srv;
    srv.request.flighttime = m_BaseFlightTime;//TODO get an actually appropriate number
    if(m_MissionClient.call(srv)){
        m_WaypointCount = srv.response.waypoints.size();
        m_LastWaypoint = 0;

        m_WaypointCtrl = srv.response.ctrl;
        m_WaypointCtrlIdx = 0;

        if(!srv.response.waypoints.empty()){
            // std::cout << "seq: "  << " " << srv.response.waypoints.size() << "\n";
            // for(uint32_t i = 0; i < srv.response.waypoints.size(); i++){
            //     mavros_msgs::Waypoint wp = srv.response.waypoints[i];
            //     std::cout << wp.frame << " " << wp.command << " " << wp.is_current << " " << wp.autocontinue << " " << wp.param1 << " " << wp.param2 << " " << wp.param3 << " " << wp.param4 << " " << wp.x_lat << " " << wp.y_long << " " << wp.z_alt << "\n";
            // }

            mavros_msgs::WaypointPush pushsrv;
            pushsrv.request.start_index = 0;
            pushsrv.request.waypoints.insert(pushsrv.request.waypoints.begin(), srv.response.waypoints.begin(), srv.response.waypoints.end());
            if(m_WPPushClient.call(pushsrv)){
                ROS_INFO("Send waypoints ok: %d", pushsrv.response.wp_transfered);
            }
        }
    }
}

void SetpointControl::publish(){

    switch(m_Mode){
        case Mode::PRINTING:
            break;
        case Mode::POSITION:
        {
            geometry_msgs::PoseStamped msg;
            msg.pose.position.x = m_LinearTarget.x;
            msg.pose.position.y = m_LinearTarget.y;
            msg.pose.position.z = m_LinearTarget.z;
            msg.pose.orientation.x = m_AngularTarget.x;
            msg.pose.orientation.y = m_AngularTarget.y;
            msg.pose.orientation.z = m_AngularTarget.z;
            m_PubLocalPose.publish(msg);
        }
            break;
        case Mode::VELOCITY:
        {
            geometry_msgs::TwistStamped msg;
            msg.twist.linear.x = m_LinearTarget.x;
            msg.twist.linear.y = m_LinearTarget.y;
            msg.twist.linear.z = m_LinearTarget.z;
            msg.twist.angular.x = m_AngularTarget.x;
            msg.twist.angular.y = m_AngularTarget.y;
            msg.twist.angular.z = m_AngularTarget.z;
            m_PubLocalVel.publish(msg);
        }
            break;
        case Mode::ACCELERATION:
            break;
    }

    std_msgs::Bool val;
    val.data = m_Extruding;
    m_PubExtrusion.publish(val);
}

void SetpointControl::finish(){
    ROS_INFO("Terminating WASP control");

    // TODO notification to mission server
    clearMission();

    if(m_Sim){// In sim, restarting manually is preferred
        init();
        reqMission();
    }else{
        // TODO confirm safe landing / disarm
        system("poweroff");
    }
}

bool SetpointControl::inProcess(){
    return m_InProcess;
}

bool SetpointControl::missionComplete(){
    return m_WaypointCount > 0 && m_LastWaypoint == m_WaypointCount - 2;
}
