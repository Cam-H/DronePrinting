#include <ros/ros.h>

#include <mavros_msgs/CompanionProcessStatus.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <nav_msgs/Odometry.h>


#include <iostream>
#include <fstream>
#include <string>

class VIOInterface {
public:

    VIOInterface(ros::NodeHandle nh);
    void loadParameters();

    void publish();

private:
    void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
private:

    ros::Subscriber m_SubOdom;
    ros::Publisher m_PubOdom;
    ros::Publisher m_PubOdom2;
    ros::Publisher m_PubVIOState;

    bool m_ReportOdometry;
};
