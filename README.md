
# F1-10 2025 Summer RCPSL Lab

The Summer F1-10 RCPSL Project attempts to autonomously control a racecar scaled to one-tenth the size. 

## Project Features:

Currently, the project features a Wall-PID control system which aims to keep the car approximately at the middle of the track at all times.

It also uses Hector-SLAM to map the area while operating under the Wall-PID control.

Future endeavors will include:

    1. Localization
    2. Path Planning
    3. Obstacle avoidance
    4. Loop-enclosed SLAM if necessary
    5. NN control for autonomous racing decision making
    


## Resources for Getting Started

The following describes resources required that fueled this endeavor. 

### Vesc Motor Control
In order to get started, this project uses a Vesc Power Board for Brushless Motor control and Servo Motor control. The repository for the motor controller can be found here: [f1tenth/vesc](https://github.com/f1tenth/vesc). It is under a BSD-3-Clause license under the F1TENTH Foundation.

### Hokuyo URG 20-LX LiDAR

URG-Node was used to register Hokuyo LiDAR scans.

``` sudo apt install ros-noetic-urg-node ```

### Hector-SLAM
A basic, open source Hector SLAM algorithm provided as a ros-noetic package was used to create a map based on the scans performed by the LiDAR.

## Project Build

> **Note:** To utilize the project, there are multiple packages that need to be installed and organized in a specific way. This is made for Ros Noetic, and will not work for other distributions of ROS.

A workspace is required for this project, for example in the home directory:

```bash 
mkdir -p catkin_ws/src
cd ~/catkin_ws/src
```
### LiDAR Package && Hector Slam

If you haven't done so already, install the URG node and the Hector Slam package.

```bash
sudo apt update
sudo apt install ros-noetic-urg-node
```
After this, install the Hector SLAM package.
```bash
sudo apt update
sudo apt install ros-noetic-hector-slam
```

### Building the Prerequisite Vesc package
Clone the prerequisite vesc directory inside the src folder. 
[f1tenth/vesc](https://github.com/f1tenth/vesc)

After cloning the repository, you should see a ```./vesc``` folder under ```~/catkin_ws/src/```.

```bash
cd ~/catkin_ws 
catkin_make 
```

### Building the entire project

Now, clone this repository under the ```./src``` folder.
```bash
cd ~/catkin_ws/src
git clone https://github.com/youngwc7/RCPSL-F1-10.git initial
catkin_make
```

Then, build the workspace 

```bash 
cd ~/catkin_ws
catkin_make 
source devel/setup.bash
```
It is recommended to add the 'source' line inside the ```~/.bashrc``` file.

## Running the Launch File

```bash 
roslaunch initial wall_follower.launch
```
## Cautions

### LiDAR Errors
When attempting to launch the file, you may encounter an error where the LiDAR is failing to initialize. If the ethernet to the LiDAR is suitably connected, check the following:

```bash
ifconfig
```
If your ethernet address does not output ```192.168.0.*```, then you will be unable to communicate to the Hokuyo LiDAR which has a default address of ```192.168.0.10```. You should also check that the Hokuyo LiDAR is indeed residing at the ```192.168.0.10``` address. If not, please reconfigure this change in the launch file under ```~/catkin_ws/src/initial/```. Inside the launch file you will see the following line:

```bash
...
<param name="ip_address" value="192.168.0.10"/>  <!-- adjust for your Hokuyo -->
...
```
Another note is that your computer's ethernet address should not be identical to the Hokuyo LiDAR's address as that would be a conflict. 

### Package Errors
If you have changed the package name (currently ```initial```) then you must reflect this change in the launch file for the executables to be detectable. 



