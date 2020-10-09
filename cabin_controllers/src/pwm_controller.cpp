#include "cabin_controllers/pwm_controller.h"

//Critical Thrust Indeces
#define MIN_THRUST 0
#define STARTUP_THRUST 1

//Slope and y-intercept Indeces
#define POS_SLOPE 0
#define POS_YINT 1
#define NEG_SLOPE 2
#define NEG_YINT 3

#define MIN_PWM 1230
#define MAX_PWM 1770
#define NEUTRAL_PWM 1500

PWMController::PWMController() : nh("~"){
    cmd_sub =nh.subscribe("/command/thrust", 1, &PWMController::ThrustCB, this);
    kill_sub = nh.subscribe("/state/switches", 1, &PWMController::SwitchCB, this);
    reset_sub = nh.subscribe("/controls/reset", 1, &PWMController::ResetController, this);

    pwm_pub = nh.advertise<cabin_msgs::PwmStamped>("/command/pwm", 1);

    PWMController::LoadParam<std::string>("properties_file", properties_file);
    properties = YAML::LoadFile(properties_file);
    PWMController::LoadThrusterProperties();

    alive_timeout = ros::Duration(2);
    last_alive_time = ros::Time::now();
    silent = true; //Silent refers to not receiving commands from the control stack
    dead = true;  //Dead refers to the kill switch being pulled
    reset_pwm = false;  //Refers to controller being reset via reset command	
}

//Load parameter from namespace
template <typename T>
void PWMController::LoadParam(std::string param, T &var){
    try{
        if(!nh.getParam(param, var)){
            throw 0;
        }
    }
    catch(int e){
        std::string ns = nh.getNamespace();
        ROS_INFO("PWM Controller Namespace: %s", ns.c_str());
        ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
        ros::shutdown();
    }
}

void PWMController::LoadThrusterProperties(){
    //Load thruster types
    int numThrusters = properties["properties"]["thrusters"].size();
    for(int i = 0; i < numThrusters; i++){
        int type = properties["properties"]["thrusters"][i]["type"].as<int>();
        thrusterType[i] = type;
    }
    //There are 3 regions of thrust: deadband, startup, and primary
    //Deadband is a regin where the propeller will not move due to internal friction and inertia
    //Startup is nonlinear, but is approximated as a line (for now)
    //Primary is linear, but has different slope than startup

    //Critical Thrusts
    PWMController::LoadParam<float>("TYPE0/MIN_THRUST", critical_thrusts[0][MIN_THRUST]);
    PWMController::LoadParam<float>("TYPE0/STARTUP_THRUST", critical_thrusts[0][STARTUP_THRUST]);
    PWMController::LoadParam<float>("TYPE1/MIN_THRUST", critical_thrusts[1][MIN_THRUST]);
    PWMController::LoadParam<float>("TYPE1/STARTUP_THRUST", critical_thrusts[1][STARTUP_THRUST]);

    //Startup config (Linear Fit: y = mx +b)
    PWMController::LoadParam<float>("TYPE0/SU_POS_THRUST/SLOPE", startup_config[0][POS_SLOPE]);
    PWMController::LoadParam<float>("TYPE0/SU_POS_THRUST/YINT", startup_config[0][POS_YINT]);
    PWMController::LoadParam<float>("TYPE0/SU_NEG_THRUST/SLOPE", startup_config[0][NEG_SLOPE]);
    PWMController::LoadParam<float>("TYPE0/SU_NEG_THRUST/YINT", startup_config[0][NEG_YINT]);

    PWMController::LoadParam<float>("TYPE1/SU_POS_THRUST/SLOPE", startup_config[1][POS_SLOPE]);
    PWMController::LoadParam<float>("TYPE1/SU_POS_THRUST/YINT", startup_config[1][POS_YINT]);
    PWMController::LoadParam<float>("TYPE1/SU_NEG_THRUST/SLOPE", startup_config[1][NEG_SLOPE]);
    PWMController::LoadParam<float>("TYPE1/SU_NEG_THRUST/YINT", startup_config[1][NEG_YINT]);

    //Primary Config (Linear Fit: y = mx + b)
    PWMController::LoadParam<float>("TYPE0/POS_THRUST/SLOPE", primary_config[0][POS_SLOPE]);
    PWMController::LoadParam<float>("TYPE0/POS_THRUST/YINT", primary_config[0][POS_YINT]);
    PWMController::LoadParam<float>("TYPE0/NEG_THRUST/SLOPE", primary_config[0][NEG_SLOPE]);
    PWMController::LoadParam<float>("TYPE0/NEG_THRUST/YINT", primary_config[0][NEG_YINT]);

    PWMController::LoadParam<float>("TYPE1/POS_THRUST/SLOPE", primary_config[1][POS_SLOPE]);
    PWMController::LoadParam<float>("TYPE1/POS_THRUST/YINT", primary_config[1][POS_YINT]);
    PWMController::LoadParam<float>("TYPE1/NEG_THRUST/SLOPE", primary_config[1][NEG_SLOPE]);
    PWMController::LoadParam<float>("TYPE1/NEG_THRUST/YINT", primary_config[1][NEG_YINT]);
}

void PWMController::ThrustCB(const cabin_msgs::ThrustStamped::ConstPtr &thrust){
    if(!dead && !reset_pwm){
        pwm_msg.header.stamp = thrust->header.stamp;
        
        pwm_msg.pwm.vector_port_fwd = Thrust2pwm(thrust->force.vector_port_fwd, thrusterType[thrust->force.VPF], "VPF");
        pwm_msg.pwm.vector_stbd_fwd = Thrust2pwm(thrust->force.vector_stbd_fwd, thrusterType[thrust->force.VSF], "VSF");
        pwm_msg.pwm.vector_port_aft = Thrust2pwm(thrust->force.vector_port_aft, thrusterType[thrust->force.VPA], "VPA");
        pwm_msg.pwm.vector_stbd_aft = Thrust2pwm(thrust->force.vector_stbd_aft, thrusterType[thrust->force.VSA], "VSA");

        pwm_msg.pwm.heave_port_fwd = Thrust2pwm(thrust->force.heave_port_fwd, thrusterType[thrust->force.HPF], "HPF");
        pwm_msg.pwm.heave_stbd_fwd = Thrust2pwm(thrust->force.heave_stbd_fwd, thrusterType[thrust->force.HSF], "HSF");
        pwm_msg.pwm.heave_port_aft = Thrust2pwm(thrust->force.heave_port_aft, thrusterType[thrust->force.HPA], "HPA");
        pwm_msg.pwm.heave_stbd_aft = Thrust2pwm(thrust->force.heave_stbd_aft, thrusterType[thrust->force.HSA], "HSA");

        pwm_pub.publish(pwm_msg);
        last_alive_time = ros::Time::now();
        silent = false;
    }
}

int PWMController::Thrust2pwm(double raw_force, int type, std::string name){
    int pwm = NEUTRAL_PWM;

    if (abs(raw_force) < critical_thrusts[type][MIN_THRUST])
        pwm = NEUTRAL_PWM;
    
    else if (raw_force > 0 && raw_force <= critical_thrusts[type][STARTUP_THRUST]) //  +Startup Thrust
        pwm = (int)(startup_config[type][POS_SLOPE] * raw_force + startup_config[type][POS_YINT]);

    else if (raw_force > 0 && raw_force > critical_thrusts[type][STARTUP_THRUST]) //  +Thrust
        pwm = (int)(primary_config[type][POS_SLOPE] * raw_force + primary_config[type][POS_YINT]);

    else if (raw_force < 0 && raw_force >= -critical_thrusts[type][STARTUP_THRUST]) //  -Startup Thrust
        pwm = (int)(startup_config[type][NEG_SLOPE] * raw_force + startup_config[type][NEG_YINT]);

    else if (raw_force < 0 && raw_force < -critical_thrusts[type][STARTUP_THRUST]) //  -Thrust
        pwm = (int)(primary_config[type][NEG_SLOPE] * raw_force + primary_config[type][NEG_YINT]);

    else
    {
        pwm = NEUTRAL_PWM;
    }

    //Constrain pwm output due to physical limitations of the ESCs
    if(pwm > MAX_PWM){
        pwm = MAX_PWM;
        ROS_WARN("Thruster Saturating. %s capped at MAX PWM = %i", name.c_str(), MAX_PWM);
    }
    if(pwm < MIN_PWM){
        pwm = MIN_PWM;
        ROS_WARN("Thruster Saturating. %s capped at MIN PWM = %i", name.c_str(), MIN_PWM);
    }

    return pwm;    
}

// lock the pwm controll
void PWMController::SwitchCB(const cabin_msgs::SwitchState::ConstPtr &state){
    dead = !state->kill;
}

void PWMController::ResetController(const cabin_msgs::ResetControls::ConstPtr &reset_msg){
    //if (reset_msg->reset_pwm)
    //    reset_pwm = true;
    //else
    //    reset_pwm = false;

    reset_pwm = true;   
}

void PWMController::PublishZeroPWM(){
    pwm_msg.header.stamp = ros::Time::now();

    pwm_msg.pwm.vector_port_fwd = NEUTRAL_PWM;
    pwm_msg.pwm.vector_stbd_fwd = NEUTRAL_PWM;
    pwm_msg.pwm.vector_port_aft = NEUTRAL_PWM;
    pwm_msg.pwm.vector_stbd_aft = NEUTRAL_PWM;
    pwm_msg.pwm.heave_port_fwd = NEUTRAL_PWM;
    pwm_msg.pwm.heave_stbd_fwd = NEUTRAL_PWM;
    pwm_msg.pwm.heave_port_aft = NEUTRAL_PWM;
    pwm_msg.pwm.heave_stbd_aft = NEUTRAL_PWM;

    pwm_pub.publish(pwm_msg);
}

void PWMController::Loop(){
    //IMPORTANT: You must have a delay in publishing pwm msgs because copro can only process data so fast
    ros::Rate rate(50);
    while(ros::ok){
        ros::spinOnce();
        ros::Duration quiet_time = ros::Time::now() - last_alive_time;
        if(quiet_time >= alive_timeout){
            silent = true;
        }

        if(silent || dead || reset_pwm){
            PWMController::PublishZeroPWM();
            reset_pwm = false;
        }
        rate.sleep();
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "pwm_controller");
    PWMController pwm_controller;
    pwm_controller.Loop();
}