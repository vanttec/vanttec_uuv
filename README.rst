<p align="center">
  <img src=https://github.com/vanttec/vanttec_uuv/blob/master/docs/LogoNegro_Azul.png width="400" height="240" align="center"/>
</p>

# VantTec UUV Main Repository

This is the main working repository for the UUV (Unmanned Underwater Vehicle) VantTec Platforms. Each directory represents a ROS Package:

- **arduino_br**: ROS package that uses rosserial_python and rosserial_arduino to interface with the T-100 and the T-200 thrusters.
- **vectornav_ros**: ROS package that allows the USV to interface with Vectornav's IMUs.
- **zed_ros_wrapper**: ROS package for the Stereolabs ZED Camera.

*TODO: Dependencies*

**How to start working?**

Enter the following commands into your **Ubuntu 16.04** terminal:

```Shell
cd
git clone http://github.com/vanttec/vanttec_uuv.git
cd vanttec_uuv
./init_worskpace.sh
```

# GateDetector3D

## How to run
___
1. Download and install [Point Cloud Library](https://pointclouds.org/) (this project used the compile-from-source method, but a prebuilt release should work).
2. Navigate to the workspace directory.
3. Do:
```sh
catkin_make
```
3. Clone [the simulation repo](https://github.com/vanttec/vanttec_uv_sim/tree/feature/testmissions) (that specific branch) and try to fire it up following the instructions referring to the UUV.
4. Run:
```sh
 source devel/setup.sh
```
```sh
 rosrun uuv_perception gate_detector_node
```
___

 ## To do:
- Make yaml files to load vehicle specific parameters to the parameter server. To be determined if it is really useful.
