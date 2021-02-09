This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.
[image1]: ./imgs/overview.png
### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the "uWebSocketIO Starter Guide" found in the classroom (see Extended Kalman Filter Project lesson).

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

### Other library/driver information
Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|        | Simulator | Carla  |
| :-----------: |:-------------:| :-----:|
| Nvidia driver | 384.130 | 384.130 |
| CUDA | 8.0.61 | 8.0.61 |
| cuDNN | 6.0.21 | 6.0.21 |
| TensorRT | N/A | N/A |
| OpenCV | 3.2.0-dev | 2.4.8 |
| OpenMP | N/A | N/A |

We are working on a fix to line up the OpenCV versions between the two.



### Project OVerview
The Udacity Carla ROS environment for this project uses the following nodes and topics.

![alt text][image1]

The `/waypoint_loader` reads the map or trajectory information and publishes a list of waypoints that the vehicle can follow safely. The `/tl_detector` node takes this information and the camera image from the vehicle - either simulation or real - and publishes the state of the traffic light ahead. The `/waypoint_updater` node determines the desired speed for the waypoints ahead. The drive-by-wire commands for the vehicle are determined by `/dbw_node` node based on the information about the waypoints necessary for steering, throttle and braking control signals

## Project Steps

### Waypoint Updater 

The `/waypoint_updater` node constantly looks ahead for the next `LOOKAHEAD_WPS = 150` waypoints. If the next traffic light is either `GREEN` or `UNKNOWN`, it will not change the desired speed at the waypoints ahead. If the next traffic light is either `RED` or `YELLOW`, it will use decelerating speed for the waypoints ahead. The vehicle might detect a `RED` or `YELLOW` traffic light, but will only react if the stop line of this traffic light is within distance of the next `LOOKAHEAD_WPS` waypoints. The decelerated speed is calculated as a minimum of the desired speed at the waypoints ahead and a square root function with a given `MAX_DECEL` deceleration so that zero speed at the stop line of the traffic light is achieved without violating the jerk limit.
### DBW Node

The `/twist_controller` node uses a yaw PID controller and throttle PID controller to determine steering, throttle and braking to keep the desired heading and speed. If the speed gets close to zero, the throttle is set to zero and the brake is engaged to hold the vehicle in place. Throttle PID control uses a low pass filtered vehicle speed to calculate the error term in order to produce a smooth output.
### Traffic light detection

The `/tl_detector` node constantly determines the processing of the incoming needs to be done or not. The function either returns the waypoint index of the stop line when the traffic light is `RED` or `YELLOW` or it returns `-1` if no traffic light stop state has been detected (`get_light_state`). The label of the images provided along with camera images is been used to classify the light color.

