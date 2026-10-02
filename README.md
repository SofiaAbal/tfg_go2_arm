# TFG Go2 + d1_550

## Instalación

En la carpeta de nuestro workspace, debemos debemos tener los paquetes necesarios bajo el directorio /src. Para compilar el proyecto:
```
source /opt/ros/humble/setup.bash 
colcon build
source install/setup.bash
```

## Ejecución

Para lanzar el brazo con el visualizador:
```
source /opt/ros/humble/setup.bash 
source install/setup.bash
ros2 launch d1_550_config d1_550_demo.launch.py
```

Para lanzar el script que nos permite recoger un objeto y moverlo:
```
source /opt/ros/humble/setup.bash
source install/setup.bash
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

```
source /opt/ros/humble/setup.bash
source install/setup.bash
```

- Servicio PickObject (side pick)
```
ros2 service call /pick_object d1_550_config/srv/PickObject "{pick_x: 0.3, pick_y: 0.4, pick_z: -0.05, shape: boX, dimension_x: 0.02, dimension_y: 0.02, dimension_z: 0.1, rot_x: 3, rot_y: -1, rot_z: 0.2, pick_grasp: side}"
```

- Servicio PickObject (side pick con mesa)
```
ros2 service call /pick_object d1_550_config/srv/PickObject "{pick_x: -0.2, pick_y: 0.5, pick_z: 0.2, shape: boX, dimension_x: 0.02, dimension_y: 0.02, dimension_z: 0.1, pick_grasp: side}"
```

- Servicio PlaceObject (side place)
```
ros2 service call /place_object d1_550_config/srv/PlaceObject "{place_x: 0.3, place_y: -0.3, place_z: -0.05, place_grasp: side}"
```

- Servicio PlaceObject (side place con mesa)
```
ros2 service call /place_object d1_550_config/srv/PlaceObject "{place_x: 0.3, place_y: 0.3, place_z: -0.05, place_grasp: side}"
```

- Servicio PickObject (top pick)
```
ros2 service call /pick_object d1_550_config/srv/PickObject "{pick_x: 0.0, pick_y: 0.3, pick_z: -0.05, shape: boX, dimension_x: 0.02, dimension_y: 0.02, dimension_z: 0.1, pick_grasp: top}"
```

- Servicio PlaceObject (top place)
```
ros2 service call /place_object d1_550_config/srv/PlaceObject "{place_x: 0.0, place_y: -0.3, place_z: -0.05, place_grasp: top}"
```

- Servicio PlaceObject (top place)
```
ros2 service call /place_object d1_550_config/srv/PlaceObject "{place_x: 0.0, place_y: 0.3, place_z: -0.1, place_grasp: top}"
```

- Servicio push button (side grasp). Empuja en horizontal, desde la base del brazo hacia el botón; por los límites de J5 solo alcanza botones a partir de z ≈ 0.2
```
ros2 service call /push_object d1_550_config/srv/PushObject "{push_x: 0.4, push_y: 0.0, push_z: 0.3, push_grasp: side}"
```

- Servicio push button (top grasp)
```
ros2 service call /push_object d1_550_config/srv/PushObject "{push_x: 0.3, push_y: 0.0, push_z: 0, push_grasp: top}"
```


## Robot real

Para poder conectar con el brazo, de momento es necesario conectarnos a través del Go2.
([repo con control básico que nos funciona](https://github.com/lakminagamage/unitree_d1_teleoperation/tree/master)).

Para conectarnos con el perro del laboratorio:
```
ssh -X unitree@192.168.123.18
```

Para depurar el nuevo driver custom que hemos montado:
```
ros2 launch d1_550_config d1_550_demo.launch.py 2>&1 | grep -F "[D1Hardware]:"
```


## Tests simulación
### Nuevo parámetro 'scene'
```
ros2 launch d1_550_config main_demo.launch.py scene_setup:=dog
ros2 launch d1_550_config main_demo.launch.py scene_setup:=table
```

Añadido nuevo parámetro de arranque para establecer el lugar sobre el que apoyamos el brazo.


### Comandos válidos

- mesa: side grasp enfrente
```
ros2 service call /pick_object d1_550_config/srv/PickObject "{pick_x: 0.4, pick_y: 0, pick_z: 0.05, shape: cylinder, dimension_x: 0.09, dimension_y: 0.025, pick_grasp: side}"
```

```
ros2 service call /place_object d1_550_config/srv/PlaceObject "{place_x: 0.4, place_y: 0, place_z: 0.1, place_grasp: side}"
```

- mesa: side grasp lateral
```
ros2 service call /pick_object d1_550_config/srv/PickObject "{pick_x: 0.0, pick_y: 0.3, pick_z: 0.05, shape: cylinder, dimension_x: 0.09, dimension_y: 0.025, pick_grasp: side}"
```

```
ros2 service call /place_object d1_550_config/srv/PlaceObject "{place_x: 0.0, place_y: -0.4, place_z: 0.05, place_grasp: side}"
```

- mesa: top graps enfrente
```
ros2 service call /pick_object d1_550_config/srv/PickObject "{pick_x: 0.1, pick_y: 0, pick_z: 0.05, shape: cylinder, dimension_x: 0.09, dimension_y: 0.025, pick_grasp: top}"
```

```
ros2 service call /place_object d1_550_config/srv/PlaceObject "{place_x: 0.2, place_y: 0, place_z: 0.05, place_grasp: top}"
```

- mesa: top grasps lateral

```
ros2 service call /pick_object d1_550_config/srv/PickObject "{pick_x: 0.1, pick_y: 0.2, pick_z: 0.05, shape: cylinder, dimension_x: 0.09, dimension_y: 0.025, pick_grasp: top}"
```

```
ros2 service call /place_object d1_550_config/srv/PlaceObject "{place_x: 0.2, place_y: -0.2, place_z: 0.05, place_grasp: top}"
```

- perro
```
ros2 service call /pick_object d1_550_config/srv/PickObject "{pick_x: 0, pick_y: 0.3, pick_z: 0.1, shape: cylinder, dimension_x: 0.08, dimension_y: 0.01, pick_grasp: side}"
```



- mesa: pick and place
```
ros2 service call /pick_and_place_object d1_550_config/srv/PickAndPlaceObject "{pick_x: 0.4, pick_y: 0, pick_z: 0.1, shape: cylinder, dimension_x: 0.09, dimension_y: 0.025, grasp: side}"
```



- mesa: push service
```
ros2 service call /push_object d1_550_config/srv/PushObject "{push_x: 0.3, push_y: 0, push_z: 0.2, push_grasp: side}"
```

```
ros2 service call /push_object d1_550_config/srv/PushObject "{push_x: 0.3, push_y: 0, push_z: 0.1, push_grasp: top}"
```


## Errores conocidos

### Errores pendientes de corregir

- **`pick_and_place_object` con coordenadas de place a 0.** En `main.cpp` un 0 en `place_x`, `place_y` o `place_z` significa "no informado" y se sustituye por la coordenada del pick. Ej.: pick en (0.3, 0, …) y place en (0, 0.3, …) deja el objeto en (0.3, 0.3, …) sin dar error. Mientras no se corrija, usar 0.001 en vez de 0.
- **`main_demo.launch.py` sin `scene_setup` falla** ("launch configuration 'scene_setup' does not exist"): el argumento no está declarado. Lanzar siempre con `scene_setup:=table` o `scene_setup:=dog`.
- **Un valor de `scene_setup` no válido** (p. ej. `Dog`) arranca el nodo sin obstáculos y sin avisar.
- **`sweep_pick_reachability.py` no funciona**: llama al servicio `plan_pick_object`, que ya no existe (se eliminaron el servicio y `planPickTask`). Hay que recuperarlos del historial (el último commit que los tiene es `afe27b0`) o quitar el script.
- **La etapa "allow collision object-support" no hace nada**: en pick and place permite la colisión entre el objeto y `table`, pero en las escenas que usamos el apoyo es `ground_table` (o `ground`/`dog`). `table` solo existe en `setupExtraObstacles`, que está comentado.
- **Comentarios desactualizados en `moveit_controllers_d1_node.yaml`**: citan `SETTLE_TIMEOUT`, `STOPPED_TOLERANCE` y `PAUSE_AFTER_TRAJECTORY`, que ya no existen en `d1_controller_custom`.
- **El log `Command deg` de `d1_controller_custom` muestra mal la pinza**: convierte con el rango de 65 mm de la documentación en vez de los 50 mm calibrados (solo afecta al log, el comando enviado usa 50).

### Limitaciones conocidas / trabajo futuro

Driver (`d1_controller_custom`):
- **Pausas fijas en vez de comprobar que el brazo llega.** Tras cada trayectoria espera 1 s (brazo) o 2,5 s (pinza) y responde éxito. Si el brazo real no ha llegado, MoveIt puede rechazar la trayectoria siguiente (`allowed_start_tolerance` = 0.07 rad). Mejora: esperar a que los joints medidos estén dentro de una tolerancia, con timeout, y sin exigirlo a la pinza (al agarrar un objeto nunca llega a su objetivo).
- **Cancelación.** Cancelar no para el brazo (sigue hacia el último punto enviado) y, según el momento en que llegue el cancel, la acción puede devolver éxito.
- **Joints por posición, no por nombre.** Una trayectoria de 2 joints se trata como pinza y el resto como brazo en orden Joint1..Joint6. Funciona porque MoveIt manda los joints en el orden del modelo.
- **Ejecución en un hilo desacoplado (`detach`).** Apagar el nodo con el brazo en movimiento puede dar un cierre sucio, y dos goals casi simultáneos podrían aceptarse a la vez (en la práctica MoveIt los manda de uno en uno). Heredado del driver original.
- **El feedback de la acción** no sigue el orden ni las unidades de los joints del goal. MoveIt no lo usa.
- **Sin secuencia de apagado.** Al cerrar el nodo los motores del brazo y de la pinza siguen activos y el brazo se queda en su última posición. Es intencionado: desactivarlos sin más haría que el brazo cayera por su peso. Mejora: al apagar, llevar el brazo a la pose de reposo y después desactivar los motores.

Tareas MTC:
- **Estado del objeto tras un fallo.** `has_object_` solo cambia cuando un pick o un place terminan bien, y `pick_and_place_object` no lo toca. Si una ejecución falla a mitad con el objeto en la pinza, el nodo cree que no lo tiene. Mejora: consultar en la planning scene si el objeto está adherido al brazo.
- **Alcance del push lateral.** Empuja en horizontal desde la base del brazo hacia el botón y, por el límite de J5 (±1.57 rad), no llega a botones bajos. La altura mínima depende de la distancia horizontal al botón: a 0.3 m hace falta z ≳ 0.2; a 0.55 m llega hasta z ≈ 0.05. Mejora: si la pose perpendicular no es alcanzable, probar con la dirección de empuje algo inclinada (±15°/30°) usando un contenedor `Fallbacks`.
- **El push tiene una sola solución.** Para cada botón el brazo solo tiene una IK válida dentro de sus límites, así que si esa falla no hay alternativa (subir `setMaxIKSolutions` no cambia nada).
- **Profundidad de pulsación** (`push_depth`, 1 cm por defecto). El brazo real va en posición: empujar 1 cm contra algo rígido fuerza los motores. Hay que ajustarla al recorrido del botón real (ver [Parámetros ajustables](#parámetros-ajustables)).


## Parámetros ajustables

Estos valores son parámetros del nodo `pick_and_place_node`. Se leen cada vez que se crea una tarea, así que un cambio se aplica en la siguiente llamada a un servicio, sin compilar ni reiniciar.

| Parámetro | Por defecto | Qué hace | Tareas |
|---|---|---|---|
| `strength` | 0.003 | m que cada dedo se cierra de más sobre el objeto para apretarlo | pick, pick and place |
| `grasp_offset` | 0.05 | m extra entre `Empty_Link6` y el centro del objeto en el agarre top | pick, place, pick and place (solo top) |
| `push_standoff` | 0.02 | m entre la punta de la pinza y el botón antes de empujar | push |
| `push_depth` | 0.01 | m que se hunde la punta dentro del botón | push |

Con el nodo en marcha, desde un terminal con el mismo entorno (`source env.sh`; si no, `ros2 param` responde "Node not found" porque está en otro `ROS_DOMAIN_ID`):

- Ver el valor actual:
```
ros2 param get /pick_and_place_node push_depth
```

- Cambiar uno:
```
ros2 param set /pick_and_place_node push_depth 0.005
```

- Cambiar varios a la vez: editar `d1_550_config/config/mtc_node_params.yaml` y cargarlo. Se lee directamente de `src`, no hace falta `colcon build`:
```
ros2 param load /pick_and_place_node ~/ws_tfg/src/tfg_go2_arm/d1_550_config/config/mtc_node_params.yaml
```

Notas:
- Siempre con punto decimal (`0.0`, no `0`). Con un entero falla: `Wrong parameter type, parameter {push_depth} is of type {double}, setting it to {integer} is not allowed`.
- Los cambios duran hasta que se reinicia el nodo. Al arrancar coge los valores de `d1_550_config/include/utils/constants.h` (`STRENGTH`, `GRASP_OFFSET`, `PUSH_STANDOFF`, `PUSH_DEPTH`). Para cambiar el valor de arranque hay que editar ahí y compilar.