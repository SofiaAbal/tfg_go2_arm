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
# En un terminal nuevo previo source
ros2 launch d1_550_config pick_and_place.launch.py
```

Para lanzar el servicio que nos permite añadir las configuraciones del objeto a manipular:
```
# En un terminal nuevo (previo source)
ros2 service call /pick_and_place_object d1_550_config/srv/PickAndPlaceObject "{pick_x: 0.1, pick_y: 0.2, pick_z: 0.0, place_x: 0.5, place_y: 0.2, place_z: 0.1, shape: box, dimension_x: 0.02, dimension_y: 0.02, dimension_z: 0.1}"
```