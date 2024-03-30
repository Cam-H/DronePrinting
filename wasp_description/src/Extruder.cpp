#include "wasp_description/Extruder.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "wasp_extruder");

    ros::NodeHandle nh;
    Extruder extruder(nh);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(100.0);

    while(ros::ok()){
      extruder.publish();

      ros::spinOnce();
      rate.sleep();
    }

    return 0;
}

Extruder::Extruder(ros::NodeHandle nh){
  m_SubExtrusion = nh.subscribe<std_msgs::Bool>("wasp/extrusion", 10, &Extruder::extrusion_cb, this);
  m_PubExtrusion = nh.advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control", 10);
}

void Extruder::loadParameters(){

}

void Extruder::extrusion_cb(const std_msgs::Bool::ConstPtr& msg){
  std::cout << "x\n";
}

void Extruder::publish(){
  static float t = 0;

  for(int j = 1; j < 4; j++){
      mavros_msgs::ActuatorControl val;
  val.group_mix = j;

  for(int i = 0; i < 8; i++){
    val.controls[i] = cos(t);
  }

  m_PubExtrusion.publish(val);
  }


    std::cout << t << " -> " << cos(t) << " za\n";

  t += 0.05;

}
