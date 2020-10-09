#include "cabin_controllers/thruster_controller.h"

ThrusterController::ThrusterController() : nh("~"){
    //Load parameters from .yaml files or launch files
    ThrusterController::LoadParam<string>("properties_file", properties_file);
    properties = YAML::LoadFile(properties_file);
    ThrusterController::LoadVehicleProperties();
    ThrusterController::SetThrusterCoeffs();
    weightLoad_eig.setZero();
    isSubmerged = false;

    for(int i = 0; i < 6; i++){
        weightLoad[i] = 0;
        transportThm[i] = 0;
        command[i] = 0;
        //solver_forces[i] = 0;
    }
    
    for(int i = 0; i < 8; i++){
        solver_forces[i] = 0;
    }

    for(int i = 0; i < 3; i++){
        solver_cob[i] = 0;
        Fb_vector[i] = 0;
    }

    state_sub = nh.subscribe<cabin_msgs::Imu>("/state/imu", 1, &ThrusterController::ImuCB, this);
    cmd_sub = nh.subscribe<cabin_msgs::NetLoad>("/command/netLoad", 1, &ThrusterController::NetLoadCB, this);
    thrust_pub = nh.advertise<cabin_msgs::ThrustStamped>("/command/thrust", 1);
    cob_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/properties/cob", 1);

    ThrusterController::InitDynamicReconfigure();
    ThrusterController::InitThrustMsg();

    //EOM problem
    problemEOM.AddResidualBlock(new ceres::AutoDiffCostFunction<EOM, 6, 8>(new EOM(numThrusters, thrustCoeffs, inertia, weightLoad, transportThm, command)), NULL, solver_forces);
    optionsEOM.max_num_iterations = 100;
    optionsEOM.linear_solver_type = ceres::DENSE_QR;

    //Buoyancy Problem
    problemBuoyancy.AddResidualBlock(new ceres::AutoDiffCostFunction<FindCoB, 3, 3>(new FindCoB(numThrusters, thrustCoeffs, Fb_vector, solver_forces)), NULL, solver_cob);
    optionsBuoyancy.max_num_iterations = 100;
    optionsBuoyancy.linear_solver_type = ceres::DENSE_QR;
}

//Load parameter from namespace
template<typename T>
void ThrusterController::LoadParam(std::string param, T &var){
    try{
        if(!nh.getParam(param, var)){
            throw 0;
        }
    }
    catch (int e){
        std::string ns = nh.getNamespace();
        ROS_ERROR("Thruster Controller Namespace: %s", ns.c_str());
        ROS_ERROR("Critical! Param \"%s/%s\" does not exist or iss not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
        ros::shutdown();
    }
}

void ThrusterController::LoadVehicleProperties(){
    mass = properties["properties"]["mass"].as<double>();

    double comX = properties["properties"]["center_of_mass"][0].as<double>();
    double comY = properties["properties"]["center_of_mass"][1].as<double>();
    double comZ = properties["properties"]["center_of_mass"][2].as<double>();
    center_of_mass[0] = comX;
    center_of_mass[1] = comY;
    center_of_mass[2] = comZ;
    Fg = mass * GRAVITY;
    depth_fully_submerged = properties["properties"]["depth_fully_submerged"].as<double>();

    Ixx = properties["properties"]["inertia"][0].as<double>();
    Iyy = properties["properties"]["inertia"][1].as<double>();
    Izz = properties["properties"]["inertia"][2].as<double>();

    inertia[0] = mass;
    inertia[1] = mass;
    inertia[2] = mass;
    inertia[3] = Ixx;
    inertia[4] = Iyy;
    inertia[5] = Izz;
    //std::cout<<"the mass is "<<mass<<" !"<<std::endl;
}

void ThrusterController::InitDynamicReconfigure(){
    //Reset server
    param_reconfig_server.reset(new DynamicReconfigServer(param_reconfig_mutex, nh));
    //Set the callback
    param_reconfig_callback = boost::bind(&ThrusterController::DynamicReconfigCallback, this, _1, _2);
    param_reconfig_server->setCallback(param_reconfig_callback);
}

void ThrusterController::SetThrusterCoeffs(){
    numThrusters = properties["properties"]["thrusters"].size();
    for(int i =0; i < numThrusters; i++){
        bool enabled = properties["properties"]["thrusters"][i]["enable"].as<bool>();
        thrustersEnabled.push_back((int)enabled);
    }

    //Each COLUMN contains a thruster's info
    int numThrustParams = properties["properties"]["thrusters"][0]["pose"].size();
    thrusters.resize(numThrustParams, numThrusters);
    thrustCoeffs.resize(6, numThrusters);
    thrusters.setZero();
    thrustCoeffs.setZero();

    for(int i = 0; i < numThrusters; i++){
        if(thrustersEnabled[i]){
            for(int j = 0; j < 5; j++){
                //Transform X, Y, Z to COM reference frame
                if(j < 3){
                    thrusters(j, i) = properties["properties"]["thrusters"][i]["pose"][j].as<double>() - center_of_mass[j];
                }
                else{
                    thrusters(j, i) = properties["properties"]["thrusters"][i]["pose"][j].as<double>();
                }
            }
        }
    }

    for(int i = 0; i < numThrusters; i++){
        if(thrustersEnabled[i]){
            //rotate around z, y axis
            float psi = thrusters(3, i) * PI / 180;
            float theta = thrusters(4, i) * PI / 180;
            thrustCoeffs(0, i) = cos(psi) * cos(theta);  //Effective contribution along X-axis
            thrustCoeffs(1, i) = sin(psi) * cos(theta);  //Effective contribution along Y-axis
            thrustCoeffs(2, i) = -sin(theta);            //Effective contribution along Z-axis

            //cross-product
            //Determine the effective moment arms for each thruster about the B-frame axes
            thrustCoeffs.block<3, 1>(3, i) = thrusters.block<3, 1>(0, i).cross(thrustCoeffs.block<3, 1>(0, i));
        }
    }
}

void ThrusterController::InitThrustMsg(){
    thrust_msg.header.stamp = ros::Time::now();
    thrust_msg.force.vector_port_fwd = 0;
    thrust_msg.force.vector_stbd_fwd = 0;
    thrust_msg.force.vector_port_aft = 0;
    thrust_msg.force.vector_stbd_aft = 0;
    thrust_msg.force.heave_port_fwd = 0;
    thrust_msg.force.heave_stbd_fwd = 0;
    thrust_msg.force.heave_port_aft = 0;
    thrust_msg.force.heave_stbd_aft = 0;

    thrust_pub.publish(thrust_msg);
}

//Callback for dynamic reconfigure
void ThrusterController::DynamicReconfigCallback(cabin_controllers::VehiclePropertiesConfig &config, uint32_t levels){
    CoB(0) = config.Buoyancy_X_POS;
    CoB(1) = config.Buoyancy_Y_POS;
    CoB(2) = config.Buoyancy_Z_POS;
    Fb = config.Buoyant_Force;

    isSubmerged = config.IS_Submerged;
}

void ThrusterController::ImuCB(const cabin_msgs::Imu::ConstPtr &imuMsg){
    float phi = imuMsg->rpy_deg.x * PI /180;
    float theta = imuMsg->rpy_deg.y * PI / 180;

    Vector3d angular_vel;
    angular_vel[0] = imuMsg->ang_vel_deg.x * PI / 180;
    angular_vel[1] = imuMsg->ang_vel_deg.y * PI / 180;
    angular_vel[2] = imuMsg->ang_vel_deg.z * PI / 180;

    transportThm[3] = -angular_vel[1] * angular_vel[2] * (Izz - Iyy);
    transportThm[4] = -angular_vel[0] * angular_vel[2] * (Ixx - Izz);
    transportThm[5] = -angular_vel[0] * angular_vel[1] * (Iyy - Ixx);

    Vector3d Fb_eig;
    Fb_eig(0) = Fb * sin(theta);
    Fb_eig(1) = -Fb * sin(phi) * cos(theta);
    Fb_eig(2) = -Fb * cos(phi) * cos(theta);

    weightLoad_eig(0) = -(Fg - Fb) * sin(theta);
    weightLoad_eig(1) = (Fg - Fb) * sin(phi) * cos(theta);
    weightLoad_eig(2) = (Fg - Fb) * cos(phi) * cos(theta);
    weightLoad_eig.segment<3>(3) = CoB.cross(Fb_eig);
    weightLoad_eig = weightLoad_eig * ((int)(isSubmerged));

    //Convert Eigen::VectorXd to c++ double[X]
    Map<RowMatrixXd>(&weightLoad[0], weightLoad_eig.rows(), weightLoad_eig.cols()) = weightLoad_eig;
    Map<Vector3d>(&Fb_vector[0], Fb_eig.rows(), Fb_eig.cols()) = Fb_eig;
}

void ThrusterController::NetLoadCB(const cabin_msgs::NetLoad::ConstPtr &load_msg){
    command[0] = load_msg->force.x;
    command[1] = load_msg->force.y;
    command[2] = load_msg->force.z;
    command[3] = load_msg->moment.x;
    command[4] = load_msg->moment.y;
    command[5] = load_msg->moment.z;

    //These initial guesses don't make much of a difference
    for(int i = 0; i < numThrusters; i++){
        solver_forces[i] = 0.0;
    }

    //solve all my problems
    ceres::Solve(optionsEOM, &problemEOM, &summaryEOM);

    thrust_msg.header.stamp = ros::Time::now();
    thrust_msg.force.vector_port_fwd = solver_forces[thrust_msg.force.VPF];
    thrust_msg.force.vector_stbd_fwd = solver_forces[thrust_msg.force.VSF];
    thrust_msg.force.vector_port_aft = solver_forces[thrust_msg.force.VPA];
    thrust_msg.force.vector_stbd_aft = solver_forces[thrust_msg.force.VSA];
    thrust_msg.force.heave_port_fwd = solver_forces[thrust_msg.force.HPF];
    thrust_msg.force.heave_stbd_fwd = solver_forces[thrust_msg.force.HSF];
    thrust_msg.force.heave_port_aft = solver_forces[thrust_msg.force.HPA];
    thrust_msg.force.heave_stbd_aft = solver_forces[thrust_msg.force.HSA];

    thrust_pub.publish(thrust_msg);
}

void ThrusterController::Loop(){
    ros::Rate rate(50);
    while(!ros::isShuttingDown()){
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "thruster_controller");
    ThrusterController tc;
    tc.Loop();
}
