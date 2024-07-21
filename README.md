<p align="right">
  <img src="docs/uuv.jpeg" width="200" height="200" align="center"/>
  <img src="docs/vanttec.png" width="320" height="180" align="left"/>
</p>

# VantTec's UUV Repository

This is the UUV's main repository running with ROS Noetic and docker containers. Please consider the host you would be working on before starting: 1) jetson tx2, 2) OS without gpu, i.e., iOS, ubuntu20.04 or windows, 3) ubuntu20.04 with gpu.

**Running the workspace for the first time**

```Shell
cd
git clone --recurse-submodules https://github.com/vanttec/vanttec_uuv.git
```

Inside the /vanttec_uuv/dockerfiles/ directory you would find the next three options: 
* ubuntu2004
* ubuntu2004_gpu
* ubuntu2004_jetsontx2

Each option contains a the Dockerfile (image) and their respective create_container.bash, so please select the one that suits you the most and continue:

```Shell
cd ~/vanttec_uuv/dockerfiles/{selected_option}
docker build -t uuv .
./create_container.bash
docker exec -it uuv /bin/bash
```

For the jetson tx2, the ZED SDK must be installed manually (other options don't require this step):

```Shell
cd /ws/vanttec_uuv/src/dockerfiles/ubuntu2004_jetsontx2
# Manual installation of the ZED SDK
./ZED_SDK_Tegra_L4T32.7_v4.1.3.zstd.run
```

Afterward:

```Shell
cd /ws/vanttec_uuv/
catkin_make
source ~/.bashrc
```

**Are you working with the ZED on your laptop without gpu?**

In this case, you would use the zed-open-capture project for manipulating the ZED camera and you must follow the next steps:

```Shell
cd /
cd zed-open-capture
cd udev 
bash install_udev_rule.sh
cd ..
mkdir build
cd build
cmake ..
make -j$(nproc)
make install
ldconfig
```

**Did the GUI was working and now isn't? Please use:**

```Shell
docker stop uuv
docker rm uuv
cd ~/vanttec_uuv/dockerfiles/{selected_option}./create_container.bash
docker exec -it uuv /bin/bash
```
