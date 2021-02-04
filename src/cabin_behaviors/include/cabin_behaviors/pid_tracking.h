#ifndef PID_TRACKING_H
#define PID_TRACKING_H
#include "ros/ros.h"
#include "dynamic_reconfigure/server.h"
#include "cabin_behaviors/PIDTrackingConfig.h"
#include "boost/thread/mutex.hpp"
#include "geometry_msgs/PointStamped.h"
#include "cabin_msgs/NetLoad.h"

class PIDTracking{
    private:
        ros::NodeHandle nh;
        //geometry_msgs::PointStamped object_deviation_msg;
        cabin_msgs::NetLoad control_value_msg;
        ros::Subscriber deviation_sub;
        ros::Publisher control_value_pub;
        double pid_p_mz, pid_i_mz, pid_d_mz;
        double pid_p_z, pid_i_z, pid_d_z;
        double last_diviation_x, last_diviation_y;
        double integral_mz; 
        double integral_z;
        double moment_z;
        double force_z;
        double safe_area_x;
        double safe_area_y;
        bool tracking_switch;

        //Dynamic Reconfigure Setup
        typedef dynamic_reconfigure::Server<cabin_behaviors::PIDTrackingConfig> DynamicReconfigServer;
        boost::shared_ptr<DynamicReconfigServer> param_reconfig_server;
        DynamicReconfigServer::CallbackType param_reconfig_callback;
        boost::recursive_mutex param_reconfig_mutex;
    
    public:
        PIDTracking();
        void DiviationSub(const geometry_msgs::PointStamped msg);
        double PIDController(double err, double pid_p, double pid_i, double pid_d, double last_err, double *integral, double safe_area);
        void InitDynamicReconfigure();
        void DynamicReconfigCallback(cabin_behaviors::PIDTrackingConfig &config, uint32_t levels);
        void Loop();
};
#endif