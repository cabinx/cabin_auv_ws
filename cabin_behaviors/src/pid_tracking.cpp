#include "cabin_behaviors/pid_tracking.h"

PIDTracking::PIDTracking() : nh("~"){
    deviation_sub = nh.subscribe<geometry_msgs::PointStamped>("/cabin_vision/deviation", 1, &PIDTracking::DiviationSub, this);
    control_value_pub = nh.advertise<cabin_msgs::NetLoad>("/command/netLoad", 1);
    //Load the pid parament
    nh.param<double>("pid_p_mz", pid_p_mz, 0.1);
    nh.param<double>("pid_i_mz", pid_i_mz, 0.01);
    nh.param<double>("pid_d_mz", pid_d_mz, 0.01);
    nh.param<double>("pid_p_z", pid_p_z, 0.1);
    nh.param<double>("pid_i_z", pid_i_z, 0.01);
    nh.param<double>("pid_d_z", pid_d_z, 0.01);
    //No action when the target is in the safe area, the area is limited by x,y to the center of the image
    nh.param<double>("safe_area_x", safe_area_x, 50);
    nh.param<double>("safe_area_y", safe_area_y, 50);
    //The switch of the tracking mode
    nh.param<bool>("tracking_switch", tracking_switch, false);

    PIDTracking::InitDynamicReconfigure();
}

double PIDTracking::PIDController(double err, double pid_p, double pid_i, double pid_d, double last_err, double *integral, double safe_area){
    double u = 0.0;
    //double t = *integral;
    if(abs(err) > safe_area){
        *integral += err;
        //t += err;
        //u = pid_p * err + pid_i * t + pid_d * (err - last_err);
        u = pid_p * err + pid_i * (*integral) + pid_d * (err - last_err);
        last_err = err;
        //*integral = t;
    }
    return u;
}

void PIDTracking::InitDynamicReconfigure(){
    //Reset server
    param_reconfig_server.reset(new DynamicReconfigServer(param_reconfig_mutex, nh));
    //Set the callback
    param_reconfig_callback = boost::bind(&PIDTracking::DynamicReconfigCallback, this, _1, _2);
    param_reconfig_server->setCallback(param_reconfig_callback);
}

//Callback for dynamic reconfigure
void PIDTracking::DynamicReconfigCallback(cabin_behaviors::PIDTrackingConfig &config, uint32_t levels){
    pid_p_mz = config.pid_p_mz;
    pid_i_mz = config.pid_i_mz;
    pid_d_mz = config.pid_d_mz;
    pid_p_z = config.pid_p_z;
    pid_i_z = config.pid_i_z;
    pid_d_z = config.pid_d_z;
    safe_area_x = config.safe_area_x;
    safe_area_y = config.safe_area_y;

    tracking_switch = config.tracking_switch;
}

void PIDTracking::DiviationSub(const geometry_msgs::PointStamped msg){
    double diviation_x = msg.point.x;
    double diviation_y = msg.point.y;
    moment_z = PIDTracking::PIDController(diviation_x, pid_p_mz, pid_i_mz, pid_d_mz, last_diviation_x, &integral_mz, safe_area_x);
    force_z = PIDTracking::PIDController(diviation_y, pid_p_z, pid_i_z, pid_d_z, last_diviation_y, &integral_z, safe_area_y);
    
    control_value_msg.header.stamp = ros::Time::now();
    control_value_msg.force.z = force_z;
    control_value_msg.moment.z = moment_z;

    if(tracking_switch){
        control_value_pub.publish(control_value_msg);
    }
}

void PIDTracking::Loop(){
    ros::Rate rate(50);
    while(!ros::isShuttingDown()){
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "pid_tracking");
    PIDTracking tc;
    tc.Loop();
}