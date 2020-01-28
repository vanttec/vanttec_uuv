<p align="center">
  <img src="https://github.com/vanttec/vanttec_usv/blob/feature/release_candidate_v1_0/docs/LogoNegro_Azul.png" width="400" height="240" align="center"/>

</p>

# VantTec UUV Main Repository

This is the main working repository for the USV (Unmanned Underwater Vehicle) VantTec Platforms. Each directory represents a ROS Package:

- **arduino_br**: ROS package that uses rosserial_python and rosserial_arduino to interface with the T-100 and the T-200 thrusters.
- **vectornav_ros**: ROS package that allows the USV to interface with Vectornav's IMUs.
- **zed_ros_wrapper**: ROS package for the Stereolabs ZED Camera.

*TODO: Dependencies*

**How to start working?**

Enter the following commands into your **Ubuntu 16.04** terminal:

```Shell
cd
git clone http://github.com/vanttec/vanttec_uuv.git
cd vanttec_usv
./init_worskpace.sh
```

