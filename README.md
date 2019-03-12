# capstone_1

 Install ROS
 
    follow webpage : http://wiki.ros.org/kinetic/Installation/Ubuntu
 ```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 
sudo apt-get update
sudo apt-get install ros-kinetic-desktop
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
```
Verify ROS
```
roscore
```

# Caution when catkin_make
----------------------------------------------------------------------------------------------------

you should install the xbox controller driver before catkin_make.

please commend that

sudo apt-get install libudev-dev ncurses-dev

# biuld dependency issue
----------------------------------------------------------------------------------------------------
If you downloaded this files and catkin_make, there could be a 'fatal error: core_msgs/ball_position.h: No such file or directory)

If so, just
catkin_make --pkg core_msgs

and then,

catkin_make

again. 

# Sponsors
----------------------------------------------------------------------------------------------------
![sponsor_image](https://user-images.githubusercontent.com/47877833/54169800-90c45280-44b7-11e9-9878-3048ad1050c0.png)

