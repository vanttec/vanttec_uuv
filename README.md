# UUV Sim Documentation
<p align="center">
  <img src=https://github.com/vanttec/vanttec_uuv/blob/master/docs/LogoNegro_Azul.png width="400" height="240" align="center"/>
</p>

Autores:
 
- Jorge Askur Vázquez Fernández
- José Miguel Flores González


## Requerimientos iniciales ###

 - [**Particion de Linux**](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&cad=rja&uact=8&ved=2ahUKEwjllKHt4PX9AhVMIUQIHRQNCUkQFnoECC0QAQ&url=https%3A%2F%2Fwww.xataka.com%2Fbasics%2Fcomo-instalar-linux-a-windows-10-ordenador&usg=AOvVaw2GEiRuUGLx-Iwj4YZMo119)
 - [**Docker**](https://docs.docker.com/engine/install/ubuntu/)
    
## Recomendaciones Docker ###

 - Descargar las extensiones Dev Containers y Docker en VS Code
    
## Instalación nvidia-docker2 ###

 - Seguir los pasos descritos en la siguiente pagina: [nvidia-docker installation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

##  Nvidia Drivers ##
 - Abrir en Ubuntu Apps Software & Updates
 - Entrar en Additional Drivers
 - Seleccionar lo siguiente:
<p align="center">
  <img src=https://github.com/vanttec/vanttec_uuv/blob/master/docs/Drivers.png align="center"/>
</p>
- Descargar el CUDA Toolkit: 
[CUDA Toolkit](https://developer.nvidia.com/cuda-downloads?target_os=Linux)

### Si no tienes tarjeta gráfica Nvidia correr en lugar del uuv.build ###

 - Escribe los siguientes comandos:

```
chmod +x runIntelGpu.bash
sudo make uuv.intelcreate
```
 - Comentar el run Pangolin dentro del Dockerfile
    
## Posible error de drivers ###

<p align="center">
  <img src=https://github.com/vanttec/vanttec_uuv/blob/master/docs/ErrorIMG.jpeg align="center"/>
</p>

 - Desactivar secure boot en la BIOS

## Descarga del contenedor del UUV ###
 - Crear una cuenta de [Docker](https://hub.docker.com/signup)
 - Ir al siguiente repositorio ![UV_dev_docker](https://github.com/vanttec/UV_dev_docker)
 - Seguir los pasos descritos en "Steps in order to start working"

## Herramientas de compilación ###

```
sudo apt-get install python3-catkin-tools
```
- El comando para compilar es
```
catkin build
```


## Repositorios a descargar ###
    
 - Estos repositorios tendran que ser clonados dentro del /ws/src


 - darknet_ros_zed (Paquete que nos permite usar YOLO)
```
git clone --recursive https://github.com/vanttec/darknet_ros.git
```
 - octomap_mapping (Paquete para mapeo)
```
git clone https://github.com/vanttec/octomap_mapping
```
 - vanttec_uuv
```
git clone https://github.com/vanttec/vanttec_uuv 
cd vanttec_uuv
git checkout retos
cd ../
```
 - vanttec_uuv_sim
Descargar del drive de VantTec
 - vehicle_user_control
```
git clone https://github.com/vanttec/vehicle_user_control
```

 - Para los siguientes vamos a salir de la carpeta src
```
cd ../
```
 - Seguiran los pasos de instalacion de la seccion Build with opencv_contrib de esta pagina: [OpenCV](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html)
 - Cuando termine borra las carpeta de build y devel
 - Para poder compilar de nuevo primero hay que sourcear la terminal
```
catkin build
source /ws/devel/setup.bash
```

## Configuraciones iniciales ###

Si estas usando Docker, en root edita el archivo .bashrc y agrega las siguientes lineas al final

```
source /ws/devel/setup.bash
export SVGA_VGPU10=0
```

## Librerias necesarias ###
```
sudo apt-get install ros-melodic-octomap ros-melodic-octomap-mapping  ros-melodic-octomap-msgs ros-melodic-octomap-ros ros-melodic-octomap-rviz-plugins ros-melodic-octomap-server

sudo apt-get install ros-melodic-octomap-ros

sudo apt install ros-melodic-tf2-geometry-msgs

sudo apt-get install libsdl1.2-dev

sudo apt-get install libsdl1.2debian

sudo apt-get install ros-melodic-gazebo-plugins

sudo apt-get install ros-melodic-gazebo-ros-pkgs

sudo apt-get install ros-melodic-robot-state-publisher

sudo apt-get install ros-melodic-xacro

```

## Librerias faltantes o que requieren versiones específicas ###

```
sudo pip3 install --upgrade pip

sudo apt-get install python3-rospkg

sudo apt install python3-pandas

sudo apt install python-pip

sudo apt-get install python3-sklearn

pip3 install numpy==1.18

pip3 install scipy==1.1.0

pip3 install scikit-learn==0.21.3

pip3 install joblib==1.0.0

pip3 install --upgrade opencv-python
pip3 install opencv-contrib-python 
pip install opencv-contrib-python
pip install opencv-python
pip install imutils

```

## Known Issues ###

Error: El submarino se carga en la superficie en lugar de bajo el agua.
**Solución:** Comentar la línea 4 del rviz.launch ubicado en /ws/src/vanttec_uuv/launch

Error: darknet_ros imprime “waiting for image” y no carga YOLO.
**Solución:** Dentro de /ws/src/darknet_ros/darknet_ros/config/ros.yaml en la línea 4 sustituir “/camera/rgb/image_raw” por “/invert_image” y dentro de  /ws/src/darknet_ros/darknet_ros/launch/RoboSub2021.launch en la línea 26 cambiar el default por “false”.

Error: Error response from daemon: failed to create shim task: OCI runtime create failed: runc create failed: unable to start container process: error during container init: error running hook #0: error running hook: exit status 1, stdout: , stderr: Auto-detected mode as 'legacy'

**Solucion:**
Correr los siguientes comandos, al final debería salir un recuadro con la información de tu tarjeta gráfica.

```
sudo apt install libnvidia-cfg1-515
sudo nvidia-persistenced --user USER #reemplaza USER con tu nombre de usuario de Ubuntu
sudo nvidia-smi
```

# Instrucciones para correr el simulador

## Si no estas usando Docker, en cada terminal ejecuta el siguiente comando al inicio: ###
```
source devel/setup.bash
```

Cada comando se tiene que ejecutar en una terminal nueva:

```
roscore

rosrun rviz rviz

roslaunch uv_worlds lake.launch

roslaunch vehicle_descriptions vtec_u3.launch 

rosrun vehicle_descriptions gazebo_interface

roslaunch vanttec_uuv uuv_simulation.launch 


roslaunch uv_worlds task_obstacles.launch 

roslaunch octomap_server octomap_tracking_server.launch 

rosrun octomap_server uuv_octomap.py
 
rosrun octomap_server oclust_aserver.py
```
### Tambien puedes correr el simulador, corriendo el siguiente comando
```
roslaunch vanttec_uuv sim.launch
```
### Control manual dentro de la simulacion ###

    -"e" para activar el control manual.
    -"w,a,s,d" para moverse en el simulador

### Para correr YOLO sigue los siguientes pasos
- Entrar a la carpeta de drive [SUB File](https://drive.google.com/drive/folders/1n8lwIeKVj_f0X7uYxjpccCf6gE2nJSBr?usp=sharing)
- Descargar el archivo yolo_zed.py y colocarlo en /ws/src/vanttec_uuv/scripts
- Hacerlo ejecutable
- Descargar la carpeta yolo-config y colocala en /ws/src/vanttec_uuv/scripts
```
rosrun vanttec_uuv yolo_zed.py
```

## Para crear coordenadas: ###

En Rviz, seleccionar publish point y dar click donde se desea colocar la coordenada
![Simulador Rviz]()
Para ver en donde esta, en una terminal dentro del container correr:
```
rostopic echo /clicked_point
```    

