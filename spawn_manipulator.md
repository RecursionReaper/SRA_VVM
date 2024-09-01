# Install
**Software**

 - ROS2 Humble
 - Gazebo
 - Rviz2
 - OpenCV

**Steps to run simulation**
First copy both the packages simulation_vvm and image_recognition in your ros2_ws

    colcon build 
 <br>
		

    src install/setup.bash
<br>    

    ros2 launch simulation_vvm basic_gazebo.launch.py
 <br>
		

    ros2 run simulation_vvm current_angle.py
<br>    

    ros2 run simulation_vvm inverse_script_seperate.py
   <br>
		

    ros2 run simulation_vvm trajectory_2.py
<br>    

    ros2 run image_recognition image_subscriber_2.py
    
 ![Image with OpenCV](https://media.discordapp.net/attachments/1257709326164819989/1279439984348237937/image.png?ex=66d472d7&is=66d32157&hm=6fb77057dc0bd24a12d0413d248a09fe50542b4a62181d64a0edceba11edbc1c&=&format=webp&quality=lossless&width=1069&height=584)

 ![enter image description here](https://media.discordapp.net/attachments/1257709326164819989/1279409439333548062/image.png?ex=66d4ff24&is=66d3ada4&hm=655c34edd28c893142e3779ed8c4d9b334115406a1bd8156c2573f51b56cce2e&=&format=webp&quality=lossless)
