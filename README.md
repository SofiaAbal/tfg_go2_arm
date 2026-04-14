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
# En un terminal nuevo (previo source)
ros2 launch d1_550_config main_demo.launch.py
```

### Tipos de primitivas

Podemos hacer que nuestro brazo manipule diferentes tipos de objetos. Los tipos y dimensiones de los mismos serán:
- **BOX:**
    - dimension_x: tamaño eje x
    - dimension_y: tamaño eje y
    - dimension_z: tamaño eje z
- **CYLINDER:**
    - dimension_x = altura
    - dimension_y = radio 
- **CONE:**
    - dimension_x = altura
    - dimension_y = radio
- **SPHERE:**
    - dimension_y = radio

> No es necesario informar los componentes que no vayamos a utilizar.

Para lanzar el servicio que nos permite añadir las configuraciones del objeto a manipular:
```
## En un terminal nuevo (previo source)

# Ejemplo BOX
ros2 service call /pick_object d1_550_config/srv/PickObject "{pick_x: 0.3, pick_y: 0.4, pick_z: -0.05, shape: box, dimension_x: 0.02, dimension_y: 0.02, dimension_z: 0.1}"

# Ejemplo CYLINDER
ros2 service call /pick_object d1_550_config/srv/PickObject "{pick_x: 0.3, pick_y: 0.4, pick_z: -0.05, shape: cylinder, dimension_x: 0.02, dimension_y: 0.02}"

# Ejemplo SPHERE
ros2 service call /pick_object d1_550_config/srv/PickObject "{pick_x: 0.3, pick_y: 0.4, pick_z: -0.05, shape: sphere, dimension_x: 0.02}"

# Ejemplo CONE
# TODO, da problemas

```


### División de la tarea

Actualmente tenemos soporte de dos servicios independientes para la tarea pick and place:
- Servicio PickObject
```
ros2 service call /pick_object d1_550_config/srv/PickObject "{pick_x: 0.3, pick_y: 0.4, pick_z: -0.05, shape: boX, dimension_x: 0.02, dimension_y: 0.02, dimension_z: 0.1, rot_x: 3, rot_y: -1, rot_z: 0.2}"
```

- Servicio PlaceObject
```
ros2 service call /place_object d1_550_config/srv/PlaceObject "{place_x: 0.3, place_y: -0.3, place_z: -0.05}"
```