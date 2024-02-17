

<p align="center">
Pick-and-Place-with-ROS2
  <h2 align="center">TFG Pick and place UR3e using ROS2 and computer vision</h2>

  <p align="center">
  Mario Sanchez Garcia UPM
  </p>
</p>
<br>

La motivación de este proyecto consiste en implementar diferentes aplicaciones de Pick and Place con el robot colaborativo UR3e y visión artificial. El objetivo es realizarlo con la ayuda de ROS2 en Python. 
La principal idea es reconocer varios objetos rectangulares de diferentes colores con ayuda de la cámara, convertir sus posiciones y que el robot sea capaz de realizar aplicaciones con estas en función de la petición del usuario en una interfaz gráfica. Además, se va a visualizar en tiempo real el comportamiento tanto de la cámara como del robot con Rviz.

## MATERIALES NECESARIOS

En cuánto a software, se necesitará tener instalada correctamente Python, ROS2, Opencv, DepthAI y Tkinter.
En cuánto a hardware, en este proyecto se ha utilizado: 
- Robot colaborativo UR3e.
- Cámara 3D con modelo OAK-D Lite AF.
- Herramienta Gripper HRC-03
- Estructura con soporte para la cámara, de forma que quede vertical al espacio de trabajo.
- Cables de conexión: 3 tipo Ethernet y 1 tipo USB-C.

## DESCRIPCION DE PAQUETES

`my_func_nodes`  Nodos creados propios para el correcto y completo funcionamiento del proyecto.

      ├── my_func_nodes 
      ├── camera.py: Nodo encargado de generar las imágenes que visualiza la cámara para su posterior comunicación con el nodo de detección
      ├── camera_pub_pos.py: Nodo encargado de analizar dichas imágenes y obetener la posición de los objetos para comunicarse con el control deel robot. interfaz_menu.py: Nodo encargado de                                crear y lanzar la interfaz gráfica de usuario la cual se comunica con los demás nodos, integrando así el proyecto. 
      ├── interfaz_menu.py: Nodo encargado de crear y lanzar la interfaz gráfica de usuario la cual se comunica con los demás nodos
      └── control_robot_master.py: Nodo que controla el movimiento del robot mediante Moveit, así como su herramienta. Además, gestiona todas las comunicaciones internas y estado actual del                                     robot.

      ├── resources
      ├── euler_to_quat.py: Calculadora que transforma vector de rotación a cuaternio para ajustar la orientación del gripper.
      └── Imágenes para la interfaz gráfica.

`my_moveit2_py` Recursos en relación a planificación de trayectoriads con Moveit.

      ├── my_moveit2_py
      ├── Moveit2_resources.py: Librería basada en Moveit con funciones para planificar y ejecutar trayectorias del robot.
      └── ur3e_model.py: Modelo configurado del robot UR3e para el correcto funcionamiento de la librería.

`my_robot_bringup_ms` Paquete de lanzamiento.

      ├── launch
      ├── control_robot.py: Archivo de lanzamiento principal del proyecto.
      └── launch_descripition_resources: Información complementaria.

      ├── config: Parámetros característicos para el funcionamiento correcto de los controladores.
      ├── argsforlaunching.yaml
      └── param_bringup.yaml

## DIAGRAMA DE NODOS

<img src="https://github.com/mariooot13/Pick-and-Place-with-ROS2/blob/tutorial/DIAGNODOS.png">

## GETTING STARTED

0) Este manual de usuario ha sido creado para aquellos usuarios que tienen una distribución de Ubuntu. En caso contrario, serán necesarias acciones adicionales. 

1) ROS2 Foxy y depedencias
En primer lugar, se necesita instalar ROS2 y todas sus dependencias de forma coorecta. Se ha usado la distribución Foxy.

- Podrás encontrar la documentación de ROS2 Foxy para su instalación en el siguiente enlace: https://docs.ros.org/en/foxy/Installation.html

A continuación, se explicará como se llevar a cabo la instalación por terminal de todas las librerías mencionadas en requisitos.

- Python3 & Tkinter: 
sudo apt install python3-colcon-common-extensions

- OpenCV y dependencias de ROS2: 
Se recomienda clonar su repositorio: git clone https://github.com/opencv/opencv.git y, posteriormente configurarlo según la documentación de instalación.
Se puede consultar su documentación en https://opencv.org/

- DepthAI: Se debe clonar el repositorio de DepthAI para ROS2 y construirlo de forma apropiada.
git clone https://github.com/luxonis/depthai_ros.git

2) UR ROS2 Drivers

El siguiente paso es clonar los controladores de UR y configurar sus paquetes. Se puede realizar de la siguiente manera:

a) export COLCON_WS=~/workspace/ros_ur_driver
   mkdir -p $COLCON_WS/src
   
b) cd $COLCON_WS
  git clone -b foxy https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver
  rosdep install --ignore-src --from-paths src -y -r
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
  source install/setup.bash 
  
- De forma adicional, puedes seguir las instrucciones del apartado Getting started de UR en el siguiente enlace: 
  https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/foxy 
  
3) IMPORTANTE. Una vez, se tienen clonados los drivers de UR, es el momento de clonar este repositorio en la rama Tutorial.

a) Dirigite hacia /workspace/ros_ur_driver/src

b) Opción 1: download a zip and paste it inside src directory.

   Opción 2: git clone https://github.com/mariooot13/Pick-and-Place-with-ROS2/tree/tutorial
  
  ### RECOMENDACIONES

- Para llevar a cabo de forma correcta toda la visión artificial, es necesario calibrar previamente su sistema de referencia en el nodo de detección. Esto es estrictamente necesario dado que los sistemas de referencia del robot y de la cámara deben alinearse.

- Cualquier duda adicional que pueda surgir, está resuelta en la memoria de este trabajo de fin de grado adjunta al repositorio.

## CONSIDERACIONES PREVIAS

Existen una serie de recomendaciones que se debe saber para el trabajo correcto del proyecto:

- Para llevar a cabo de forma correcta toda la visión artificial, es necesario calibrar previamente su sistema de referencia en el nodo de detección. Esto es estrictamente necesario dado que los sistemas de referencia del robot y de la cámara deben alinearse.

- Cualquier duda adicional que pueda surgir, está resuelta en la memoria de este trabajo de fin de grado adjunta.

- El link del repositorio con todos los recursos disponibles es: \url{https://github.com/mariooot13/Pick-and-Place-with-ROS2/tree/tutorial}

Además, se debe conocer como llevar a cabo el primer paso antes de lanzar nada, que es la correcta conexión de los dispositivos:

- El primer paso antes de lanzar nada, es llevar a cabo la correcta conexión de los dispositivos.

a) Conexión del robot: 
Dentro del archivo de lanzamiento se ha asignado una dirección IP la cual deberá ser la que tu selecciones en la tablet del robot.
Deberás conectar el cable Ethernet a tu ordenador, y visualizar que dirección IP tiene el ordenador.
En la tablet, necesitarás crear un programa con control externo en el asignes la dirección IP del ordenador.

b) Instalación de la herramienta:
La herramienta debe estar instalada en la tablet del robot.

c) Cámara:
Conecta la cámara medianete un cable con conexión USB y asegurate de su correcto posicionamiento.

## Archivo de lanzamiento del proyecto

Es importante conocer todos los componentes ejecutados en este archivo único de lanzamiento, para conocer la posibilidad de ejecutar cada uno por separado. Se ejecutan los siguientes componentes, los cuales se muestran mediante el nombre de su ejecutable:

- ur_control.launch.py. Archivo que lanza los controladores de UR, encargado de la conexión con el robot y, a su vez, lanza nodos importantes para su funcionamiento.

- ur_moveit.launch.py. Archivo que lanza el nodo move_group, para la planificación de trayectorias. Además, lanza el visualizador Rviz y otros nodos complementarios a nivel de robótica. 
    
- control_robot_node_exec. Ejecutable del nodo de control principal del robot, control_robot_master. 

- camera_exec. Ejecutable del nodo que genera las imágenes de la cámara que lanza su visualización y publica en tiempo real con formato ros.

- camera_detection. Ejecutable del nodo encargado del análisis de las imágenes publicadas por la cámara y de la detección, reconocimiento y obtención de posición de los objetos. Además, incluye todo tipo de comunicaciones en el diagrama principal.
    
- interfaz_exec. Ejecutable del nodo responsable de la creación de la interfaz y de sus comunicaciones con el resto de nodos según la interacción externa.   
    
### subsubsection{Lanzamiento individual}

A pesar de haber creado un archivo de lanzamiento que facilite el funcionamiento del proyecto en un único comando, existe la posibilidad de ejecutar cada nodo por separado de forma individual gracias a sus ejecutables:

ros2 run my_func_nodes "nombre del ejecutable"

  
## USO

0) Abre una terminal en tu ordenador.

1) Necesitarás escribir el siguiente comando por pantalla.

      source install/setup.bash 
      
2) Lanza el archivo de lanzamiento en dicha terminal.

      ros2 launch my_robot_bringup_ms control_robot.py
      
3) Dale al PLAY en el programa del robot en la tablet.

4) Interacciona con la interfaz gráfica de usuario.
