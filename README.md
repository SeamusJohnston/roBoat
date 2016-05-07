#Arduino Boat Visualisation Package

This package depends on the razor_imu_9dof package, and robot_pose_ekf. This package was also written on Ubuntu 14.04.04 on ROS Indigo.
The package lets you control the rotation of the prop and the motor speed with the keyboard.  It also lets you visualize gyro data in RVIZ, aswell as displaying ultrasonic sensor data as a cone.  The arduino code prints the gyro data and ultrasonic data through serial for the package to read and then display in RVIZ.  The imu sensor is an MPU6050 gyro/accel and the ultrasonic is an HC-SR04 sensor.  They use the MPU6050 and NewPing libraries respectively.  The MPU6050 must be downloaded from here: https://github.com/jarzebski/Arduino-MPU6050 and the NewPing from https://github.com/PaulStoffregen/NewPing.  Both must be put in your library folder for the arduino IDE. 

#Installing razor_imu_9dof and robot_pose_ekf

Paste the following code to install the razor imu package in your catkin_ws.

    $ cd catkin_ws/src
	$ git clone https://github.com/KristofRobot/razor_imu_9dof.git
    $ cd ..
    $ catkin_make
    $ cd
    $ sudo apt-get install ros-indigo-robot-pose-ekf
    
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

Please feel free to contact me at seamusbjohnston@gmail.com or post anything in the issues tab.

Thanks!
