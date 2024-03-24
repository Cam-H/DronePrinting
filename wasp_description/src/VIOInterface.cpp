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
    m_SubOdom = nh.subscribe<nav_msgs::Odometry>("vio/odom", 10, &VIOInterface::odom_cb, this);
    m_PubOdom = nh.advertise<nav_msgs::Odometry>("/mavros/odometry/out", 10);
    m_PubOdom2 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/mavros/vision/pose_cov", 10);
    m_PubVIOState = nh.advertise<mavros_msgs::CompanionProcessStatus>("/mavros/companion_process/status", 10);

    loadParameters();
}

void VIOInterface::loadParameters(){
    ros::param::param <bool>("~report", m_ReportOdometry, false);

}

void VIOInterface::odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    mavros_msgs::CompanionProcessStatus cmsg;
    cmsg.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_ACTIVE;
    cmsg.component = mavros_msgs::CompanionProcessStatus::MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;
    m_PubVIOState.publish(cmsg);

    nav_msgs::Odometry tmsg = *msg;
    tmsg.child_frame_id = "base_link";
    m_PubOdom.publish(tmsg);

    geometry_msgs::PoseWithCovarianceStamped vmsg;
    vmsg.pose = tmsg.pose;
    m_PubOdom2.publish(vmsg);

    if(m_ReportOdometry){
        std::string pos = "Odometry Position: [" + std::to_string(tmsg.pose.pose.position.x) + ", " +
                                                   std::to_string(tmsg.pose.pose.position.y) + ", " +
                                                   std::to_string(tmsg.pose.pose.position.z) + "]\n";
        // ROS_INFO(pos.c_str());
        std::cout << pos;

    }
}

void VIOInterface::publish(){

}
