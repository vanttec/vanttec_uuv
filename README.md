<p align="center">
  <img src=https://github.com/vanttec/vanttec_uuv/blob/master/docs/LogoNegro_Azul.png width="400" height="240" align="center"/>
</p>

# VantTec's Unmanned Underwater Vehicle (UUV) Repository.

Working. This is the main UUV's repository, which is based entirely on a docker container and ROS Noetic.

**How to start working?**

```Shell
cd
git clone --recurse-submodules https://github.com/vanttec/vanttec_uuv.git
chmod +x create_container_gpu.bash 
./create_container_gpu.bash
docker build -t uuv .
docker exec -it uuv /bin/bash
```


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

