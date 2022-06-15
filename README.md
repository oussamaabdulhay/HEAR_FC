# How to Setup Workspaces
## _Setup the Mission Control PC (HEAR_Navio2_MC)_
- Create a catkin workspace for HEAR_MC:
    ``` 
    mkdir -p catkin_ws_hear_mc/src
    cd ~/catkin_ws_hear_mc/src
    git clone https://github.com/AhmedHumais/HEAR_Navio2_MC.git
    git clone https://github.com/AhmedHumais/HEAR_Architecture.git
    git clone https://github.com/AhmedHumais/HEAR_msgs.git
    git clone https://github.com/AhmedHumais/HEAR_ROS_BRIDGE.git
    cd ..
    catkin_make
    source devel/setup.bash
    ```
- In the .bashrc file export ROS_MASTER_URI as the IP of the Ground Station and export ROS_IP as the IP of the Raspberry PI.



- Create a catkin workspace for xsense:
    ```
    mkdir -p catkin_ws_xsense/src
    cd ~/catkin_ws_xsense/src
    git clone https://github.com/AhmedHumais/xsens_ros_mti_driver.git
    cd ..
    pushd src/xsens_ros_mti_driver/lib/xspublic && make && popd
    catkin_make
    source devel/setup.bash
    ```

- Create a catkin workspace for mocab:
    ```
    mkdir -p catkin_ws_other/src
    cd ~/catkin_ws_other/src
    git clone https://github.com/AhmedHumais/mocap_optitrack.git
    cd ..
    catkin_make
    source devel/setup.bash
    ```
- In the .bashrc file export ROS_MASTER_URI as the IP of the Ground Station and export ROS_IP as the IP of the Raspberry PI.

# How to Fly
> ⚠️ Note:
>-In order to work properly, all hardware connections should be checked carefully. 
>-Check the motors connections and their rotation convention.
>-Attach the proper propellers relative to the correct rotation convention of the motors.
>-Check the battery level before flight.
>-Check the lab safety requirements before conducting any flight tests.

# _On Ground station_
1. run `roscore`.
2. Go to step 1 in (On Rasspberry PI) section.
3. Run the mission control: `rosrun hear_navio_mc_example mc_test_node` (choose your mission control file)
4. Check that the sensors are working properly: `rostopic echo /Inner_sys/body_ori` (run `rostopic list` to know all the published topics).
5. To save the data use: `rosbag record -a` 
6. Run the heartbeat command: `rostopic pub -r 10 /heartbeat std_msgs/Empty "{}"`.
7. Go to (Operation) section.

# _On Rasspberry PI_
1. Open rasspberry pi using `ssh pi@10.0.0.XXX` (insert the IP address of the Raspberry PI).
2. Password: raspberry.
3. `sudo su`.
4. `source .bashrc`.
5. Run the flight controller: `roslaunch flight_controller flight_controller.launch`.
6. `roslaunch xsens_mti_driver xsens_mti_node.launch` (not needed, should launch automatically with flight_controller.launch).
7. `roslaunch mocap_optitrack mocap.launch` (not needed, should launch automatically with flight_controller.launch).
8. Go to step 3 in (On Ground Station) section.

# _Operation_
1. Keep a terminal open with the following command: `rosservice call /arm "data: false"`.
> ⚠️ **This is extremely important as this is considered as the kill switch to abort the flight any time due to any unforeseen circumstances**

2. Use the following command to send the required user commands to the flight controller following the mission scenario designed: `rosservice call /flight_command "{}"`
