#include "ros/ros.h"
#include "cabin_teleop/t4_button_mapping.h"
#include "sensor_msgs/Joy.h"
#include "cabin_msgs/NetLoad.h"
#include "cabin_msgs/SwitchState.h"
#include "cabin_msgs/ResetControls.h"

double joy_force[3];                  //The force input along the x, y, z axis
double joy_moment[3];                 //The moment input around the x, y, axis
bool joy_switch = false;
bool joy_switch_state = false;
bool joy_reset = false;
bool joy_reset_state = false;
double max_forward_thrust = 33.0;
double max_backward_thrust = 25.0;
double max_forward_thrust_ud = 60.0;
double max_backward_thrust_ud = 60.0;

double current_axes_factor[4];

void JoyCallback(const sensor_msgs::Joy msg){
    //Increase/decrease 5.0 eachtime
    if(1 == msg.axes[AXES_CROSS_UD]){
        joy_force[0] += 5.0;
        std::cout<<"current joy input thrust is "<<joy_force[0]<<"!!!"<<std::endl;
    }
    else if(-1 == msg.axes[AXES_CROSS_UD]){
        joy_force[0] -= 5.0;
        std::cout<<"current joy input thrust is "<<joy_force[0]<<"!!!"<<std::endl;
    }
    else{
        joy_force[0] += 0.0;
    }

    if(1 == msg.axes[AXES_CROSS_LR]){
        joy_moment[2] += 5.0;
        std::cout<<"current joy input moment is "<<joy_moment[2]<<"!!!"<<std::endl;
    }
    else if(-1 == msg.axes[AXES_CROSS_LR]){
        joy_moment[2] -= 5.0;
        std::cout<<"current joy input moment is "<<joy_moment[2]<<"!!!"<<std::endl;
    }
    else{
        joy_moment[2] += 0.0;
    }

    if(msg.axes[AXES_STICK_LEFT_UD]){
        current_axes_factor[0] = msg.axes[AXES_STICK_LEFT_UD];
        if(current_axes_factor[0] > 0){
            joy_force[0] = current_axes_factor[0] * max_forward_thrust;
        }
        else{
            joy_force[0] = current_axes_factor[0] * max_backward_thrust;
        }
    }

    if(msg.axes[AXES_STICK_LEFT_LR]){
        current_axes_factor[1] = msg.axes[AXES_STICK_LEFT_LR];
        if(current_axes_factor[1] > 0){
            joy_moment[2] = current_axes_factor[1] * max_forward_thrust;
        }
        else{
            joy_moment[2] = current_axes_factor[1] * max_backward_thrust;
        }
    }

    if(msg.axes[AXES_STICK_RIGHT_UD]){
        current_axes_factor[2] = msg.axes[AXES_STICK_RIGHT_UD];
        if(current_axes_factor[2] < 0){
            joy_force[2] = current_axes_factor[2] * max_forward_thrust_ud;
        }
        else{
            joy_force[2] = current_axes_factor[2] * max_backward_thrust_ud;
        }
    }

    if(1 == msg.buttons[BUTTON_SHAPE_A]){
        joy_switch = true;
        joy_switch_state = !joy_switch_state;
        std::cout<<"current joy switch state is : "<<joy_switch_state<<"!!!"<<&std::endl; 
    }

    if(1 == msg.buttons[BUTTON_SHAPE_B]){
        joy_reset = true;
        std::cout<<"current joy reset state is : "<<joy_reset_state<<"!!!"<<&std::endl; 
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "t4_controller");
    ros::NodeHandle n;
    ros::Rate loopRate(50);

    ros::Subscriber joy_sub = n.subscribe("/joy",1000,JoyCallback);
    ros::Publisher joy_netLoad_pub = n.advertise<cabin_msgs::NetLoad>("/command/netLoad",1);
    ros::Publisher joy_swtich_state_pub = n.advertise<cabin_msgs::SwitchState>("/state/switches",1);
    ros::Publisher joy_reset_state_pub = n.advertise<cabin_msgs::ResetControls>("/command/reset",1);

    cabin_msgs::NetLoad output_netLoad;
    cabin_msgs::SwitchState output_switch_state;
    cabin_msgs::ResetControls output_reset_state;

    while(ros::ok()){
        if(joy_switch){
            output_switch_state.kill = joy_switch_state;
            joy_swtich_state_pub.publish(output_switch_state);
            joy_switch = false;
        }
        
        if(joy_reset){
            output_reset_state.reset_pwm = joy_reset_state;
            joy_reset_state_pub.publish(output_reset_state);
            for(int i = 0; i < 3; i++){
                joy_force[i] = 0.0;
                joy_moment[i] = 0.0;
            }
            joy_reset = false;
        }

        output_netLoad.header.stamp = ros::Time::now();
        //Move forward
        output_netLoad.force.x = joy_force[0];
        output_netLoad.force.y = joy_force[1];
        output_netLoad.force.z = joy_force[2];
        //Rotate around z axis (yaw)
        output_netLoad.moment.x = joy_moment[0];
        output_netLoad.moment.y = joy_moment[1];
        output_netLoad.moment.z = joy_moment[2];
        joy_netLoad_pub.publish(output_netLoad);
        ros::spinOnce();
        loopRate.sleep();
    }
}
