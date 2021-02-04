# cabin_auv_ws 
  As rarely opensource rov project, ardusub is powerful. It works perfectly on bluerov. But I'm not familier to develop embedded applications. So I find it difficult to achieve my goals on the pixhawk to me, such as arbitrarily adjust the thrusters layout, use diffrent types of thrusters and so on. On the other hand, I think the bluerov framewok, pixhawk(ardusub)---respberry 3B(can not upgrade)---QGroundControl, is very redundancy. The framework is not friendly both to some developers like me. It is also not friendly to some AI applications development.
  
  ROS is widely used in the robot area. It's easy to develop, and has many successful applications in AI area. I hope to DIY my own ROV/AUV based on ros in the future. So I develop a simple demo for rov tracking object to verify the feasibility.
  
  I test the demo on a bluerov, replace the pixhawk with a stm32 to process the pwm wave to control the thrusters. The main target is to verify the feasibility of developing rov/auv soft framework on ros, since I can change the hardware freely in the future. So I ignore many problems, such as depth hold, the time delay caused by video transmission and so on. From this perspective, the test result is not bad.
  
  As the test environment, the ground pc is ubuntu 18.04, and the raspberry 3B on the bluerov is ubuntu mate 18.04. The ros version is melodic.

  
  ![image](https://github.com/cabinx/cabin_auv_ws/blob/master/src/image/yaw.gif)![image](https://github.com/cabinx/cabin_auv_ws/blob/master/src/image/sink.gif)
     
  
  The whole software framwork as below:
 ![image](https://github.com/cabinx/cabin_auv_ws/blob/master/src/image/Vision_Frame.png)
   
                                   The vision module
 ![image](https://github.com/cabinx/cabin_auv_ws/blob/master/src/image/Control_Frame.png)
   
                                The base control module
  
  cabin_controllers: The movement module. This module refer from the Ohio State University Underwater Robotics Team's project. For more information: https://github.com/osu-uwrt
  
  cabin_teleop: Manual mode. Control the rov with a joystick.
  
  usb_cam: Device for the camera. For more information: http://wiki.ros.org/usb_cam
  
  cabin_vision: Process the picture info. In this demo, the error between the center of the object and the picture is calculated. 
  
  cabin_behaviors: Strategy layer. In this demo, a PID controller is used to process the error for the control value.
  
  The cabin_controllers and the usb_cam shuold work on the rov, the others can work on the ground pc or the rov.
  
  Attention: [darknet_ros](https://github.com/leggedrobotics/darknet_ros) is needed for object recognition.
  
