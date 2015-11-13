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
rosrun hw_interface singleJoint &
```
The controller_manager now runs in main(), you can query his services with:
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
If you want to use our custom controller, you need to set up the ros parameter server (this can later be simplified in a .yaml file):
```
#!bash
rosparam set test_controller/type hw_controller/singleJointController
rosparam set test_controller/joint A
```
Loading this controller via the respective ros service call:
```
#!bash
rosservice call /controller_manager/load_controller test_controller
```
Unfortunately so far this causes an error:
```
#!bash
[ERROR] [1447431875.426379240, 4192.754000000]: This controller requires a hardware interface of type 'hardware_interface::EffortJointInterface'. Make sure this is registered in the hardware_interface::RobotHW class.
[ERROR] [1447431875.426459258, 4192.754000000]: Initializing controller 'test_controller' failed
ok: False
```