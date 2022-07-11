# ROS2 STorM32 Gimbal Driver


This is a ROS2 driver to communicate with STorM32 gimbal controllers. It is based on the original ROS1 based repository by McGill Robotics which can be found [here](https://github.com/mcgill-robotics/ros-storm32-gimbal). It was directly ported to ROS2, applying minor (convenience) changes and making it Python3 compatible.

**You MUST configure your gimbal with the Windows app before using this
package.**

*This package has been tested on ROS2 Foxy Fitzroy on Ubuntu 20.04. with Python3.8, but should generally work with any ROS2 distribution and modern Linux systems (including single board computers like the RaspberryPi or NVIDIA Jetson).*

## Setting up

Clone this repository into your colcon workspace's *src* folder:

```bash
git clone https://github.com/defrag-bambino/ros2-storm32-gimbal
```

## Dependencies

Before proceeding, make sure to install all dependencies by running:

```bash
rosdep update
rosdep install --from-paths storm32_gimbal_* --ignore-src -y
```
and
```bash
pip3 install pyserial transforms3d numpy
```

## Compiling

You **must** compile this package before being able to run it. You can do so
by running (don't forget ro source your ros2 installation beforehand):

```bash
colcon build
```

from the root of your `colcon` workspace.


## Running
Source the overlay workspace:
```bash
source install/setup.bash
```

To run, simply launch the `storm32_controller_node` node as such:

```bash
ros2 run storm32_gimbal_controller storm32_gimbal_controller 
```

You can change the port and TF frame ID by passing them as parameters

```bash
ros2 run storm32_gimbal_controller storm32_gimbal_controller --ros-args -p serial_port:=/dev/ttyACM0 -p frame_id:=example_frame
```


## Interfacing

### The `gimbal_ref` frame

All orientations used by this package are relative to a global reference frame
called `gimbal_ref`. This reference frame is attached to the gimbal, and has a
z axis always pointing away from the center of the earth (i.e. it does not
pitch or roll, but can yaw).

### Topics

This package publishes to two topics:

- `~camera_orientation`: The IMU1 readings as a `GimbalOrientation` message
  (i.e. the orientation of the gimballed link relative to the global
  `gimbal_ref` frame). Since the link is stabilized, this should always be
  approximately the target orientation of the gimbal.
- `~controller_orientation`: The IMU2 readings as a `GimbalOrientation` message
  (i.e. the orientation of the STorM32 board relative to the global
  `gimbal_ref` frame). This should represent the orientation of the gimbal and
  is not stabilized.

A `GimbalOrientation` message displays the orientation both as a quaternion and euler angles (*yaw*, *pitch*, *roll* - in degrees).

You can also set a new target orientation relative to the `gimbal_ref` frame by
publishing a `TargetGimbalOrientation` message to the `~target_orientation` topic.
It's `orientation` field is of type `GimbalOrientation` and thus has a *quaternion* and also *yaw*, *pitch*, *roll* properties which are all expected to be relative to the `gimbal_ref` frame. Wether the gimbal driver will use the quaternion or euler angles is determined by the `use_quaternion` boolean.
The `unlimited` field defines whether the controller should attempt to
limit its rotation to hard set limits on the STorM32 controller.


## Contributing

Contributions are welcome. Simply open an issue or pull request on the matter,
and it will be accepted as long as it does not complicate the code base too
much.

## License

See [LICENSE](LICENSE).
