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
Unfortunately the controller_manager seems to crash here