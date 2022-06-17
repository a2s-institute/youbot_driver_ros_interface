# Warning
This code has not been tested on the robot yet. In particular, the changes to the action server to follow joint trajectories has had several changes which need to be carefully tested.

youbot_driver_ros_interface
===========================

Interface classes for ROS to the youBot driver.


### Build
```
colcon build --symlink-install
```

### Setcap for raw socket access
```
sudo setcap cap_net_raw+ep ~/foxy_ws/build/youbot_driver_ros_interace/youbot_driver_ros_interface
```
Starting from Ubuntu 18.04, there are issues running ROS nodes with root capabilities. Therefore you might get errors that certain libraries are not found. In that case, add the following lines to `/etc/ld.so.conf.d/ros.conf`
```
/opt/ros/foxy/lib
/opt/ros/foxy/lib/x86_64-linux-gnu
```

and run `sudo ldconfig`.

If you still get an error saying that `librmw_fastrtps_cpp` is not found, then launch the node after switching to the root account with `sudo su` (you will also have to re-source the `setup.bash` files).

Some related issues:
 * https://github.com/ros2/rcpputils/issues/40
 * https://github.com/orocos/rtt_ros2_integration/issues/36 
 * https://github.com/ros2/rcpputils/pull/122

Once you launch the driver as root, you may not be able to see the node or topics (using `ros2 node list` and `ros2 topic list`) from your user account, but can still see them as the root user. This is probably because of an issue with the FastDDS (see https://github.com/eProsima/Fast-DDS/issues/1750). You can either switch to another middleware (such as CycloneDDS), or follow the instructions in the issue to create an xml file to force FastDDS to use UDP for communication. You should now be able to see the node and topics from any user account.

### Launch
```
ros2 launch youbot_driver_ros_interface youbot_driver.launch.py
```

### Examples

#### Position testing
By running the command below, the user will be able to provide angle input to each joints in the youbot manipulator. **Please provide angles within safety limit during testing (Eg: -0.5 to 0.5 rads)**
```
ros2 run youbot_driver_ros_interface youbot_arm_test
```
#### Velocity testing
By running the command below, the user will be able to provide velocity input to each joints in the youbot manipulator. **Please provide velocities within safety limit during testing (Eg: -0.2 to 0.2 rads)**
```
ros2 run youbot_driver_ros_interface youbot_arm_vel_test
```

