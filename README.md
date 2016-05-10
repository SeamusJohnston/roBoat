#Arduino Boat Visualisation Package

This package depends on the razor_imu_9dof package, and robot_pose_ekf, as well as using the teleop_twist_keyboard package to control motor direction and speed. This package was also written on Ubuntu 14.04.04 on ROS Indigo.

The package lets you control the rotation of the prop and the motor speed with the keyboard.  It also lets you visualize gyro data in RVIZ, aswell as displaying ultrasonic sensor data as a cone.  The arduino code prints the gyro data and ultrasonic data through serial for the package to read and then display in RVIZ.  The imu sensor is an MPU6050 gyro/accel and the ultrasonic is an HC-SR04 sensor.  They use the MPU6050 and NewPing libraries respectively.  The MPU6050 must be downloaded from here: https://github.com/jarzebski/Arduino-MPU6050 and the NewPing from https://github.com/PaulStoffregen/NewPing.  Both must be put in your library folder for the arduino IDE. 

#Installing razor_imu_9dof, robot_pose_ekf, and teleop_twist_keyboard

Paste the following code to install the razor imu package in your catkin_ws.

    $ cd catkin_ws/src
	$ git clone https://github.com/KristofRobot/razor_imu_9dof.git
    $ cd ..
    $ catkin_make
    $ cd
    $ sudo apt-get install ros-indigo-robot-pose-ekf
    $ sudo apt-get install ros-indigo-teleop-twist-keyboard
    You will also need to edit the robot_pose_ekf launch file
    
    $ roscd robot_pose_ekf
    $ sudo nano robot_pose_ekf
    
    Change imu_used to true, odom_used to true and vo_used to false.
    Also delete this line: <remap from="odom" to="pr2_base_odometry/odom" /> 
    
#Installing roBoat 

    $ cd catkin_ws/src
    $ git clone https://github.com/SeamusJohnston/roBoat.git
    $ cd ..
    $ catkin_make
    
#Modifying razor_imu_9dof to work with RoBoat 

    $ cd
    $ rm ~/catkin_ws/razor_imu_9dof/nodes/imu_node.py
    $ mv ~/catkin_ws/src/roBoat/nodes/imu_node.py ~/catkin_ws/src/razor_imu_9dof/nodes
    
#Using roBoat

1)Load the boat_serial.ino file in the arduino folder onto your arduino.

2)All the bash scripts are in the bashFiles folder, so you can edit master.sh to launch whichever nodes or programs you want.  Make sure all the scripts are executable and run:
   
    $ ./catkin_ws/src/roBoat/bashFiles/master.sh

This will launch the whole series of nodes.

You can now tilt your arduino or boat to see the change, and move the ultrasonic to visualize it with a cone!  Also if you click on the terminal window that launch all the nodes (node.launch), it will have instructions a bit further up the window on using the keyboard to control the prop.

#Making it wireless

If you want the boat to be wireless from your computer and still get all the data from the arduino to your computer, you either need a bluetooth serial communication to your computer or you need the arduino attached to a Raspberry Pi, and have your computer and the Raspberry Pi run ROS on the same network.  All the visualisation software will be run on the computer, but the Raspberry Pi will run:
1. Upload the arduino code to the arduino and have all the arduino libraries. 
2. Must have the modified razor_imu_9dof and run the imu_node.py code with this code:
	$ rosrun razor_imu_9dof imu_node.py

You can then comment out the line in the node.launch file that runs the razor_imu_9dof node on your computer.  

Make sure your network is configured correctly to use ROS over wifi.  Follow these links to make sure it is:
http://wiki.ros.org/ROS/Tutorials/MultipleMachines
http://wiki.ros.org/ROS/NetworkSetup

Please feel free to contact me at seamusbjohnston@gmail.com or post anything in the issues tab.  There are also videos of it working on my website, http://seamusbjohnston.com.

Thanks!
