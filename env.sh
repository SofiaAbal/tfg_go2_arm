# Entorno del TFG. Cargar en cada terminal nuevo con:
#   source ~/ws_tfg/src/tfg_go2_arm/env.sh
# (con 'source', no ejecutándolo: los export tienen que quedarse en la shell)

# No mezclar instalaciones: si el terminal ya tiene otra cargada (p. ej. /opt/ros/rolling, que
# además no define ROS_DISTRO), abrir uno limpio
_tfg_other="$(printf '%s' "$AMENT_PREFIX_PATH" | tr ':' '\n' | grep '^/opt/ros/' | grep -v '^/opt/ros/humble$' | head -n1)"
if [ -n "$_tfg_other" ]; then
    echo "env.sh: este terminal ya tiene cargado $_tfg_other, abre uno nuevo" >&2
    unset _tfg_other
    return 1
fi
unset _tfg_other

_tfg_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

source /opt/ros/humble/setup.bash
if [ -f "$_tfg_ws/install/setup.bash" ]; then
    source "$_tfg_ws/install/setup.bash"
else
    echo "env.sh: no existe $_tfg_ws/install, compila con colcon build y vuelve a hacer source" >&2
fi
unset _tfg_ws

# ROS siempre sobre FastDDS, igual que fuerza custom_driver.launch.py. CycloneDDS queda solo
# para el SDK de Unitree (d1_controller_custom / bridge), que lo configura en su propio launch.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# El SDK de Unitree habla con el brazo en el dominio DDS 0 (explícito en el código, no lee esta
# variable). Con ROS en otro dominio el tráfico de ROS y el del brazo no se ven entre sí, y
# tampoco los ROS de otras personas en la misma red. Tiene que ser el mismo en todos los terminales.
export ROS_DOMAIN_ID=42

# La interfaz de red del brazo la pone el launch solo a los nodos del SDK (additional_env).
# Exportada aquí haría que cualquier nodo con CycloneDDS se atase al adaptador USB-Ethernet.
##unset CYCLONEDDS_URI

# Opcional, solo simulación: el tráfico de ROS no sale del ordenador
# export ROS_LOCALHOST_ONLY=1

##echo "env.sh: ROS $ROS_DISTRO | $RMW_IMPLEMENTATION | ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
