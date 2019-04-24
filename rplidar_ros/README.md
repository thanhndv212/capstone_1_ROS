RPLIDAR ROS package
=====================================================================

ROS node and test application for RPLIDAR

Visit following Website for more details about RPLIDAR:

rplidar roswiki: http://wiki.ros.org/rplidar

rplidar HomePage:   http://www.slamtec.com/en/Lidar

rplidar Tutorial:  https://github.com/robopeak/rplidar_ros/wiki

How to build rplidar ros package
=====================================================================
    1) Clone this project to your catkin's workspace src folder
    2) Running catkin_make to build rplidarNode and rplidarNodeClient

How to run rplidar ros package
=====================================================================
There're two ways to run rplidar ros package

I. Run rplidar node and view in the rviz
------------------------------------------------------------
roslaunch rplidar_ros view_rplidar.launch

You should see rplidar's scan result in the rviz.

II. Run rplidar node and view using test application
------------------------------------------------------------
roslaunch rplidar_ros rplidar.launch

rosrun rplidar_ros rplidarNodeClient

You should see rplidar's scan result in the console

RPLidar frame
=====================================================================
RPLidar frame must be broadcasted according to picture shown in
rplidar-frame.png

when you have usb error command to terminal them

ls -l /dev |grep ttyUSB
sudo chmod 666 /dev/ttyUSB0

rplidar 노드의 경우 매 실행시마다 실행하는 터미널에서 sudo chmod 로 /dev/ttyUSB0에 권한을 주어야하는 문제가 있으니,
sudo gedit /etc/udev/rules.d/50-ttyusb.rules 후
KERNEL=="ttyUSB0", NAME="tts/USB%n", GROUP="capstone",MODE="0666"
을 입력하고 저장
