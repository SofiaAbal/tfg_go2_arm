# TFG Go2 + d1_550


## Instalación

En la carpeta de nuestro workspace, debemos debemos tener los paquetes necesarios bajo el directorio /src. Para compilar el proyecto:
```
colcon build
source install/setup.bash
```

## Ejecución

Para lanzar el brazo con el visualizador:
```
ros2 launch d1_550_config d1_550_demo.launch.py
```

Para lanzar el script que nos permite recoger un objeto y moverlo:
```
# En un terminal nuevo
ros2 launch d1_550_config pick_and_place.launch.py
```
