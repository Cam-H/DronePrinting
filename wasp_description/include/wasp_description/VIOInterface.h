#include <ros/ros.h>


#include <mavros_msgs/CompanionProcessStatus.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <nav_msgs/Odometry.h>

#include <chrono>
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

    void reset_vio();
private:

    ros::Subscriber m_SubOdom;
    ros::Publisher m_PubOdom;
    ros::Publisher m_PubOdom2;
    ros::Publisher m_PubVIOState;

    int m_FeatureCount;

    bool m_ReportOdometry;
    int m_TerminationLimit;
    int m_CriticalLimit;

    double m_Timeout;
    std::chrono::time_point<std::chrono::steady_clock> m_LastPostTime;

    double m_CumulativeError;
    double m_CumulativeLimit;
};
