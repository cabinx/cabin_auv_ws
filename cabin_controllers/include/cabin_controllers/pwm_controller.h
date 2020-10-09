#ifndef PWM_CONTROLLER_H
#define PWM_CONTROLLER_H

#include "ros/ros.h"
#include <yaml-cpp/yaml.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>

#include "cabin_msgs/PwmStamped.h"
#include "cabin_msgs/ThrustStamped.h"
#include "cabin_msgs/SwitchState.h"
#include "cabin_msgs/ResetControls.h"

class PWMController{
private:
    ros::NodeHandle nh;
    ros::Subscriber cmd_sub, kill_sub, reset_sub;
    ros::Publisher pwm_pub;
    cabin_msgs::PwmStamped pwm_msg;

    YAML::Node properties;
    std::string properties_file;
    int thrusterType[8];
    float startup_config[2][4],primary_config[2][4];  //Slopes and y-intercepts
    float critical_thrusts[2][2];  //Minimum and startup thrusts

    bool dead, silent, reset_pwm;
    ros::Time last_alive_time;
    ros::Duration alive_timeout;

public:
    PWMController();
    template<typename T>
    void LoadParam(std::string param, T &var);
    void LoadThrusterProperties();
    void ThrustCB(const cabin_msgs::ThrustStamped::ConstPtr &thrust);
    void SwitchCB(const cabin_msgs::SwitchState::ConstPtr &state);
    void ResetController(const cabin_msgs::ResetControls::ConstPtr &reset_msg);
    void PublishZeroPWM();
    int Thrust2pwm(double raw_force, int type, std::string name);
    void Loop(); //A loop fxn is needed b/c copro can only read msgs so quickly
};
#endif