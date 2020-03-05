# dymaxiel_rx28_rx24_bogobot
Creación de los programas "Standard Pose", "PingMotores" y "Motores" del Bogobot v2

## Descripción general
Programas para la ejecución de una posición en 'T'(**std_pose**), **ping** y comprobación de la energización(**motores**) de todos los motores del robot Bogobot v2 con los servomotores Dynamixel RX-28 y RX-24F el cuál tiene como función colocar los servomotores en una posición tipo HOME y usarlo en programas de secuencias y/o caminado.

## Pre-requisitos
El usuario deberá tener correctamente instalado ROS, GIT, así como la carpeta de `catkin_ws` inicializada así como los paquetes de Dynamixel para ROS, previamente instalado en el repositorio: [example_dynamixel](https://github.com/aaceves/example_dynamixel).

## Proceso de utilización

Al conectar el USB a la computadora con Ubuntu, es necesario verificar que el dispositivo aparece como algún puerto USB, p. ej. `/dev/ttyUSB0`, así como es de suma importancia habilitar los permisos de lectura de dicho dispositivo para ello se necesita escribir en una linea de comandos:
```
sudo chmod a+rw /dev/ttyUSB0 
```
Para instalar el paquete de este repositorio llamado: `motores`, se deben ejecutar en una terminal las siguientes instrucciones:

```
cd ~/catkin_ws/src
git clone https://github.com/JuanCarlos-MA/dymaxiel_rx28_rx24_bogobot.git
cd ~/catkin_ws
catkin build
source devel/setup.bash
```
Ya terminado el proceso, deberá de abrir dos Terminales nuevas y ejecutar los siguientes comandos:

* Terminal 1
```
roscore
```
* Terminal 2

  - En caso de utilizar el *pingMotors.cpp*
    ```
    rosrun motores pingMotors
    ```

  - En caso de utilizar el *motors.cpp*
    ```
    rosrun motores motors
    ```

  - En caso de utilizar el *std_pose.cpp*
    ```
    rosrun motores std_pose
    ```
## Autor

**Juan Carlos Martínez Aguilar** *Estancia de Investigación en Robots Humanoides*
