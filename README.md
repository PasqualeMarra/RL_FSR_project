# RL_FSR_project

technical project 3 for RL and FSR courses

## Developed packages

- **custom_msg_pkg** : used to define a custom message for the communication between the planner and the controller.
- **mav_control** : containing the low level control based on the passivity based method.
- **mav_planner** : containing the planner based on the potential field technique.
- **mav_description** : containing the description of the drone, modified from [mav_description](https://github.com/rl2021/mav_description.git)
- **my_scenario** : used to launch the enviroment and spawn the drone. It contains also the force plugin requested by the project.

## Dependecies

- ROS Noetic
- Gazebo
- RViz
- [aruco_ros](https://github.com/pal-robotics/aruco_ros)
- [rotors_simulator](https://github.com/ethz-asl/rotors_simulator)
- [mav_comm](https://github.com/ethz-asl/mav_comm)
- Catkin
- Eigen


<details>
  <summary>How to install</summary>

Create a catkin workspace and clone the repositories in the src folder: 

```bash
$ cd [path to workspace]/src
$ git clone https://github.com/PasqualeMarra/RL_FSR_project.git
$ git clone https://github.com/rl2021/aruco_ros.git
$ git clone https://github.com/jocacace/px4_gazebo_standalone.git
$ git clone https://github.com/ethz-asl/rotors_simulator.git
$ git clone https://github.com/ethz-asl/mav_comm.git
$ cd ..
$ catkin_make
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```
  ## Note
  Compiling this repository you will generate the shared library: libmy_force_plugin.so. Including this plugin in your UAV model, you can apply on the UAV a force of 1 N directed along the x direction of the world frame.  
</details>

<details>
  <summary>Low level control test</summary>

- Launch the scenario and spawn the drone:
```bash
$ roslaunch my_scenario my_scenario.launch
```
- Launch the low level controller:
```bash
$ roslaunch mav_control mav_control.launch
```

Now you can publish the desired trajectory on the topic _/planner_des_:
```bash
$ rostopic pub /planner_des custom_msg_pkg/planner_msg
```
Note: take advantage of the auto-completion by means of the tab key if you want to publish only a desired pose. The reference frame is world NED.

</details>

<details>
  <summary>Project execution</summary>


- Launch the scenario and spawn the drone:
```bash
$ roslaunch my_scenario my_scenario.launch
``` 
- Launch the low level controller and the planner:
```bash
$ roslaunch mav_planner mav_planner.launch
``` 
You will see the drone reaching a predefined set of waypoints, adjusting its trajectory on the base of the surrounding enviroment.

</details>
