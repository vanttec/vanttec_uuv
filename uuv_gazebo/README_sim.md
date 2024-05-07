# IN PROGRESS... VantTec UUV simulation

This is the UUV simulation repository, using as a base the uv_sim repository from Vanttec. It runs on Gazebo and requires the package uuv_simulator to be installed.

### To run the UUV simulator, in a terminal write the following commands:

```
roslaunch uv_worlds lake.launch
roslaunch vehicle_descriptions vtec_u3.launch 
rosrun vehicle_descriptions gazebo_interface
roslaunch vanttec_uuv uuv_simulation.launch 

run uuv simulation file from vanttec_uuv package
```

### To run fixed simulation, in a terminal write the following commands:

```
roslaunch vehicle_descriptions uuv_sim.launch
```


# Scenario tasks

- **Tasks2020** 
<p align="center">
  <img src="https://github.com/vanttec/vanttec_uv_sim/blob/feature/testmissions/sim_challenges/scene.jpeg" width="600" height="440" align="center"/>

</p>



To run the UV simulator for Aiming System, in a terminal write the following command:

`roslaunch shooter_description shooter_gazebo.launch`

To run the node for shooting the torpedoes, in a terminal write the following command:

`rosrun shooter_description shoot.py`

# uuv_sensor_plugins: Simulated Sensors for UUVs

## Contents:

 - ```uuv_sensor_plugins```: Contains gazebo plugins for various simulated sensors.
 - ```uuv_sensor_ros_plugins```: ROS wrappers for each of the above.
