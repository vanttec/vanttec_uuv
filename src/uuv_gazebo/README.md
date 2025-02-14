<p align="right">
  <img src="../docs/uuv.jpeg" width="200" height="200" align="center"/>
  <img src="../docs/vanttec.png" width="320" height="180" align="left"/>
</p>

# UUV's simulator

This package is based on the uuv_simulator project. Given that the project was archived, some modifications were made to make it compatible with the current Vanttec's UUV simulation.

**Important information**

Plugins:
- realsense_gazebo_plugin: 
  - It contains the packages required for the realsense camera.
- uuv_sensor_plugins:
  - uuv_sensor_plugins: contains gazebo plugins for various simulated sensors.
  - uuv_sensor_ros_plugins: ROS wrappers for each of the above.
- uuv_gazebo_plugins: 
  - uuv_gazebo_plugins: contains physical and thruster packages for the simulation.
  - uuv_gazebo_ros_plugions: ROS wrappers for each of the above.
- uuv_sensor_plugins:
  - uuv_sensor_plugins: Contains gazebo plugins for various simulated sensors.
  - uuv_sensor_ros_plugins: ROS wrappers for each of the above.

### To run the UUV simulator, on two different terminals use:

```roslaunch uuv_gazebo lake.launch```

```roslaunch uuv_description vtec_u3.launch```