#include "wasp_description/VIOInterface.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "wasp_extruder");

    ros::NodeHandle nh;
    VIOInterface vio(nh);

    ros::Rate rate(20.0);

    while(ros::ok()){
      vio.publish();

      ros::spinOnce();
      rate.sleep();
    }

    return 0;
}

VIOInterface::VIOInterface(ros::NodeHandle nh){
    m_SubOdom = nh.subscribe<nav_msgs::Odometry>("/wasp/vio/odom", 10, &VIOInterface::odom_cb, this);
    m_PubOdom = nh.advertise<nav_msgs::Odometry>("/mavros/odometry/out", 10);
    m_PubOdom2 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/mavros/vision/pose_cov", 10);
    m_PubVIOState = nh.advertise<mavros_msgs::CompanionProcessStatus>("/mavros/companion_process/status", 10);

    m_FeatureCount = 0;

    m_LastPostTime = std::chrono::steady_clock::now();

    loadParameters();
}

void VIOInterface::loadParameters(){
    ros::param::param <bool>("~report", m_ReportOdometry, false);

    ros::param::param <int>("~term_limit", m_TerminationLimit, 8);
    ros::param::param <int>("~critical_limit", m_CriticalLimit, 20);
    if(m_CriticalLimit < m_TerminationLimit) m_CriticalLimit = m_TerminationLimit;

    ros::param::param <double>("~timeout", m_Timeout, 0.5);

    ros::param::param <double>("~cumulative_limit", m_CumulativeLimit, 5.0);
}

void VIOInterface::odom_cb(const nav_msgs::Odometry::ConstPtr& msg){


    m_LastPostTime = std::chrono::steady_clock::now();

    nav_msgs::Odometry tmsg = *msg;
    m_PubOdom.publish(tmsg);

    // geometry_msgs::PoseWithCovarianceStamped vmsg;
    // vmsg.pose = tmsg.pose;
    // m_PubOdom2.publish(vmsg);

    if(m_ReportOdometry){
        std::string pos = "Odometry Position: [" + std::to_string(tmsg.pose.pose.position.x) + ", " +
                                                   std::to_string(tmsg.pose.pose.position.y) + ", " +
                                                   std::to_string(tmsg.pose.pose.position.z) + "]\n";
        std::cout << pos;
    }
}

void VIOInterface::reset_vio(){
    // ROS_WARN("");
    m_CumulativeError = 0;
}

void VIOInterface::publish(){
    mavros_msgs::CompanionProcessStatus cmsg;
    cmsg.component = mavros_msgs::CompanionProcessStatus::MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;
    cmsg.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_FLIGHT_TERMINATION;

    const std::chrono::duration<double> elapsed_seconds{std::chrono::steady_clock::now() - m_LastPostTime};

    if(elapsed_seconds.count() > m_Timeout){
        ROS_WARN("VIO Timeout. No Odometry received");
    } else {
        if(m_FeatureCount < m_TerminationLimit || m_CumulativeError > m_CumulativeLimit){
            m_CumulativeError += elapsed_seconds.count();
        }else if(m_FeatureCount < m_CriticalLimit){
            cmsg.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_CRITICAL;
        }else{
            cmsg.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_ACTIVE;
        }
    }

    if(m_CumulativeError > m_CumulativeLimit) reset_vio();

    m_PubVIOState.publish(cmsg);
}
