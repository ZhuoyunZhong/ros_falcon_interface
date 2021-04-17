# ROS Falcon Interface

This package provides a ROS interface for Novint Falcon haptic controller. This repository is developed based on the falcon library [libnifalcon](https://github.com/libnifalcon/libnifalcon).

<img src="https://images-na.ssl-images-amazon.com/images/I/41cDl3q9mSL._AC_.jpg" width="300">

## Dependencies

This repository has been developed and tested in Ubuntu 18.04 and ROS Melodic only. Other version may work but it is not guaranteed.

To use this package, you need to build and install [libnifalcon](https://github.com/libnifalcon/libnifalcon).

```
git clone git@github.com:libnifalcon/libnifalcon.git
cd libnifalcon
mkdir build && cd build
cmake ..
make -j10
sudo make install
```

After installing [libnifalcon](https://github.com/libnifalcon/libnifalcon), you could download and make this repository.

```
git clone git@github.com:ZhuoyunZhong/ros_falcon_interface.git
cd ..
catkin_make
```

## Running

- Connect the Novint Falcon device with the computer

- Run this interface by

  ```
  rosrun ros_falcon_interface ros_falcon_interface
  ```

  You may need to calibrate your device, please follow the instruction. The robot should be able to home itself at (0.0, 0.0, 0.125), which is considered to be the center point of its workspace.

## Interface Details

The following ROS topics and methods are provided to interact with the Novint Falcon. **Please be careful when setting up force feedback or position goal. Falcon may have strong reaction due to its controller design.**

1. The position of the device is sent to topic "falcon_pos".

   ```
   rostopic echo /falcon_pos
   ```

   Data type is `geometry_msgs::Point`.

2. The button messages of the device are sent to topic "falcon_button".

   ```
   rostopic echo /falcon_button
   ```

   Data type is `std_msgs::Int8`. When the button is pressed and released, one topic message would be sent out. **1**, **2**, **3** and **4** represent **center**, **forward**, **left** and **right** button.

   ---

3. The LED light color could be changed by publishing `std_msgs/String` data to "set_falcon_led". Example:

   ```
   rostopic pub /set_falcon_led std_msgs/String "data: 'r'"
   ```

   Three colors, 'red' / 'r', 'green' / 'r', 'blue' / 'b' can be set up. When the input is not these three, the combined RGB color would be displayed, which is also the default color.

4. There are four different haptic mode for haptic device. They can be changed by publishing `std_msgs/Int8` data to "set_falcon_led". Example:

   ```
   rostopic pub /set_falcon_haptic_mode std_msgs/Int8 "data: 0"
   ```

   0, the default mode, represents self-centering mode. In this mode, falcon will always home itself in the center point. 1 is position keeping mode. This mode is activated when one tries to set up a goal point and let falcon to follow. 2 is constant force mode, which is activated when one sends a constant force feedback. All the other input will lead to the final mode, no haptic force mode.

   Position keeping and constant force modes are explained in details below.

5. To have falcon provide a haptic force. Send  `geometry_msgs/Vector3` data to "set_falcon_force".  Example:

   ```
   rostopic pub /set_falcon_force geometry_msgs/Vector3 "x: 1.5
   y: 1.5
   z: 1.0"
   ```

   The haptic mode will be set to constant force mode.

6. To set a goal location for falcon to go. Send  `geometry_msgs::Point` data to "set_falcon_point".  Example:

   ```
   rostopic pub /set_falcon_point geometry_msgs/Point "x: -0.02
   y: 0.02
   z: 0.15"
   ```

   A PID controller is implemented and tuned in order to let falcon to reach the desired goal. The haptic mode will be set to position keeping mode. The approximate valid range for 3 axes are -0.06<x<0.06, -0.06<y<0.06 and 0.075<z<0.175.

   

For more potential usages of the interface and driver for Novint Falcon, please also check the [https://github.com/libnifalcon/libnifalcon/tree/master/examples](https://github.com/libnifalcon/libnifalcon/tree/master/examples) provided by [libnifalcon](https://github.com/libnifalcon/libnifalcon).