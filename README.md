# cabin_auv_ws
    As rarely opensource rov project, ardusub is powerful. It works perfectly on bluerov.   
    But I'm not familier to develop embedded applications. So I find it difficult to achieve my goals on the pixhawk to me, such as arbitrarily adjust the thrusters layout, use diffrent types of thrusters and so on.   
    On the other hand, I think the bluerov framewok, pixhawk(ardusub)---respberry 3B(can not upgrade)---QGroundControl, is very redundancy. The framework is not friendly both to some developers and some AI applications.
    ROS works well in the robot area. It's easy to develop, and has many successful applications in AI. I hope to DIY my own ROV/AUV based on ros in the future. So I develop a simple demo for rov tracking object to verify the feasibility.
    I test the demo on a bluerov, replace the pixhawk with a stm32 to process the pwm wave to control the thrusters. The main target is to verify the feasibility of developing rov/auv soft framework on ros, since I can change the hardware freely in the future. So I ignore many problems, such as depth hold, the time delay caused by video transmission and so on. From this perspective, the test result is not bad.
![image](https://github.com/cabinx/cabin_auv_ws/blob/master/image/yaw.gif)
![image](https://github.com/cabinx/cabin_auv_ws/blob/master/image/sink.gif)
     
     
     The whole software framwork as below:
 ![image](https://github.com/cabinx/cabin_auv_ws/blob/master/image/Vision_Frame.png)
 ![image](https://github.com/cabinx/cabin_auv_ws/blob/master/image/Control_Frame.png)
