#ifndef THRUSTER_CONTROLLER_H
#define THRUSTER_CONTROLLER_H

#include "math.h"
#include "ceres/ceres.h"

#include "ros/ros.h"
#include "dynamic_reconfigure/server.h"
#include "cabin_controllers/VehiclePropertiesConfig.h"
#include "boost/thread/mutex.hpp"
#include "cabin_msgs/ThrustStamped.h"
#include "cabin_msgs/Imu.h"
#include "cabin_msgs/NetLoad.h"
#include "geometry_msgs/Vector3Stamped.h"

#include "yaml-cpp/yaml.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"

using namespace Eigen;
using namespace std;

#define PI 3.141592653
#define GRAVITY 9.81        //[m/s^2]
#define WATER_DENSITY 1000  //[kg/m^3]

typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, Dynamic, RowMajor> RowMatrixXd;

class ThrusterController{
    private:
        ros::NodeHandle nh;
        ros::Subscriber state_sub, cmd_sub;
        ros::Publisher thrust_pub, cob_pub;

        cabin_msgs::ThrustStamped thrust_msg;
        geometry_msgs::Vector3Stamped cob_msg;

        //Dynamic Reconfigure Setup
        typedef dynamic_reconfigure::Server<cabin_controllers::VehiclePropertiesConfig> DynamicReconfigServer;
        boost::shared_ptr<DynamicReconfigServer> param_reconfig_server;
        DynamicReconfigServer::CallbackType param_reconfig_callback;
        boost::recursive_mutex param_reconfig_mutex;

        YAML::Node properties;
        string properties_file;
        vector<int> thrustersEnabled;
        Vector3d CoB;
        double mass, Fg, Fb, Ixx, Iyy, Izz,depth_fully_submerged;
        bool isSubmerged;

        //Variables that get passed to class EOM and FindCoB
        MatrixXd thrustCoeffs, thrusters;
        Vector6d weightLoad_eig;
        int numThrusters;
        double inertia[6], weightLoad[6], transportThm[6], command[6], Fb_vector[3];
        double solver_forces[8];   //Solved forces go here
        double solver_cob[3];      //Solved buoyancy positions go here
        double center_of_mass[3];

        //EOMs
        ceres::Problem problemEOM;
        ceres::Solver::Options optionsEOM;
        ceres::Solver::Summary summaryEOM;

        //Locate Buoyancy Position
        ceres::Problem problemBuoyancy;
        ceres::Solver::Options optionsBuoyancy;
        ceres::Solver::Summary summaryBuoyancy;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ThrusterController();
        template<typename T>
        void LoadParam(std::string param, T &var);
        void LoadVehicleProperties();
        void InitDynamicReconfigure();
        void SetThrusterCoeffs();
        void InitThrustMsg();
        void DynamicReconfigCallback(cabin_controllers::VehiclePropertiesConfig &config, uint32_t levels);
        void ImuCB(const cabin_msgs::Imu::ConstPtr &imu_msg);
        void NetLoadCB(const cabin_msgs::NetLoad::ConstPtr &load_msg);
        void Loop();
};

//Class EOM defines the 6 equations of motiion that ceres needs to solve
class EOM{
    private:
        int numThrusters;           //Number of thrusters
        MatrixXd thrustCoeffs;      //Thrust coefficients(effective contributions of each thruster for force and moments)
        double *inertia;            //(Pointer) Inertia-related values (mass, mass, Ixx, Iyy, Izz)
        double *weightLoad;         //(Pointer) Forces/moments due to weight forces (gravity and buoyancy)
        double *transportThm;       //(Pointer) Vector containng terms related to the transport theorem
        double *command;            //Command for each EOM

    public:
        EOM(int &numThrust, const Ref<const MatrixXd> &thrustCoeffsIn, double *inertiaIn, double *weightLoadIn, double *transportThmIn, double *commandIn){
            numThrusters = numThrust;
            thrustCoeffs = thrustCoeffsIn;
            inertia = inertiaIn;
            weightLoad = weightLoadIn;
            transportThm = transportThmIn;
            command = commandIn;
        }

        template <typename T>
        bool operator()(const T *const forces, T *residual) const{
            for (int i = 0; i < 6; i++){
                residual[i] = T(0);

                //Account for each thruster's contributioin to force/moment
                for(int j = 0; j < numThrusters; j++){
                    residual[i] = residual[i] + T(thrustCoeffs(i,j)) * forces[j];
                }

                //Account for weight-related forces/moments and transportThm
                residual[i] = residual[i] + T(weightLoad[i] + transportThm[i]);
                residual[i] = residual[i] - T(command[i]);
            }
            return true;
        }
};

//Class FindCoB will determine an estimate for the location of the center of buoyancy relative to the center of mass
class FindCoB{
    private:
        int numThrusters;        //Number of thrusters
        int x, y, z;             //Axis indeces
        MatrixXd thrustCoeffs;   //Thrust coefficients(effective contributions of each thruster for force and moments)
        double *Fb;              //(Pointer) Buoyant forces along body-frame x, y, and z axes
        double *forces;          //(Pointer) Solved thruster forces from class EOM

    public:
        FindCoB(int &numThrust, const Ref<const MatrixXd> &thrustCoeffsIn, double *FbIn, double *forcesIn){
            numThrusters = numThrust;
            thrustCoeffs = thrustCoeffsIn;
            Fb = FbIn;
            forces = forcesIn;
            x = 0, y = 1, z =2;
        }
        //rFb is the position vector for the center of buoyancy from the center of mass
        template<typename T>
        bool operator()(const T *const rFb, T *residual) const{
            for(int i = 0; i < 3; i++){
                residual[i] = T(0);
                for(int j = 0; j < numThrusters; j++){
                    residual[i] = residual[i] +T(thrustCoeffs(i+3, j) * forces[j]);
                }
            }
            residual[x] = residual[x] + (rFb[y] * T(Fb[z]) - rFb[z] * T(Fb[y]));
            residual[y] = residual[y] + (rFb[z] * T(Fb[x]) - rFb[x] * T(Fb[z]));
            residual[z] = residual[z] + (rFb[x] * T(Fb[y]) - rFb[y] * T(Fb[x]));
            return true;
        }
};

#endif