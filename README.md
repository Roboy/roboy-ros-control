This illustrates ros hierarchy in simple examples

# Dependencies #

```
#!bash
sudo apt-get install ros-indigo-desktop-full
sudo apt-get install ros-indigo-controller-interface ros-indigo-controller-manager
```

# Build #

```
#!bash

cd path/to/ros_hierarchy
catkin_init_workspace
rm CMakeLists.txt
catkin_make
```

# Run it #
```
#!bash

cd path/to/ros_hierarchy
source devel/setup.bash
roscore &
rosrun hw_interface multiJoint &
```
The controller_manager now runs in the asyncronous thread, you can query his services with:
```
#!bash

rosservice list
```
This should output these services:
```
#!bash

/controller_manager/list_controller_types
/controller_manager/list_controllers
/controller_manager/load_controller
/controller_manager/reload_controller_libraries
/controller_manager/switch_controller
/controller_manager/unload_controller
```
If you call for the controller types:
```
#!bash

rosservice call /controller_manager/list_controller_types
```
This should show our custom controller plugin:
```
#!bash
types: ['hw_controller/singleJointController']
base_classes: ['controller_interface::ControllerBase']
```
### Running a controller from commandline ###
If you want to use our custom controller, you need to set up the ros parameter server:
```
#!bash
rosparam set test_controller/type hw_controller/singleJointController
rosparam set test_controller/joint_name motor0
```
Loading and starting this controller via spawn:
```
#!bash
rosrun controller_manager controller_manager spawn test_controller
```
### Using ros launch file for more convenient controller startup ###
In the path/to/ros_hierarchy/src/hw_controller/config folder is a launch file, this can be executed via:
```
#!bash
roslaunch hw_controller test_controller.launch
```

### Status of the controller ###
The status of the controller can be queried via:
```
#!bash
rosservice call /controller_manager/list_controllers
```
This should show the running controllers, together with the resources they have been assigned to
```
#!bash
controller: 
  - 
    name: test_controller0
    state: running
    type: hw_controller/singleJoint_controller
    hardware_interface: hardware_interface::PositionJointInterface
    resources: ['motor0']
  - 
    name: test_controller1
    state: running
    type: hw_controller/singleJoint_controller
    hardware_interface: hardware_interface::PositionJointInterface
    resources: ['motor1']
  - 
    name: test_controller2
    state: running
    type: hw_controller/singleJoint_controller
    hardware_interface: hardware_interface::PositionJointInterface
    resources: ['motor2']

```


# Test with hardware #
controller_manager needs to be run as root, don't know why yet (otherwise flexray won't connect)
```
#!bash
sudo -s
rosrun hw_interface multiJoint
```
start controller
```
#!bash
roslaunch hw_controller test_single_controller.launch
```
start trajectory publisher
```
#!bash
rosrun trajectory_advertiser trajectory_advertiser
```