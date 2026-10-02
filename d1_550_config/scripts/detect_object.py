#!/usr/bin/env python3
import math
import shlex
import subprocess

# De momento el objeto es siempre una botella: cilindro de pie (rot = 0).
# En el servidor: dimension_x = altura, dimension_y = radio (setObjectData)
SHAPE = "cylinder"
SERVICES = ("pick", "place")
GRASPS = ("side", "top")

# Superficies de apoyo de cada escena (setupTable / setupDogDown en pick_and_place.cpp),
# en world (= base_link): (nombre, x_min, x_max, y_min, y_max, z_superior)
SCENES = {
    "table": [("suelo", -0.75, 0.75, -0.75, 0.75, -0.004)],
    "dog": [("suelo", -1.0, 1.0, -1.0, 1.0, -0.195),
            ("perro", -0.35, 0.35, -0.175, 0.175, 0.0)],
}
SURFACE_TOLERANCE = 0.005  # m que la base puede quedar por encima/debajo de la superficie

# Pinza del modelo (URDF y mallas de Empty_Link6 y Empty_Link_L/R)
GRIPPER_MAX_OPENING = 0.066  # m entre dedos con la pinza abierta
# Con top el TCP (0.10 + GRASP_OFFSET desde Empty_Link6) va al centro del objeto y la palma
# está a ~0.052 m de Empty_Link6: la mitad superior del objeto no puede superar esta distancia
TOP_MAX_HALF_HEIGHT = 0.15 - 0.052


def spherical_to_cartesian(distance, azimuth, elevation):
    az = math.radians(azimuth)
    el = math.radians(elevation)
    x = distance * math.cos(el) * math.cos(az)
    y = distance * math.cos(el) * math.sin(az)
    z = distance * math.sin(el)
    return x, y, z


def ask(prompt):
    """input() que devuelve None si el usuario quiere salir (q, Ctrl+C, Ctrl+D)."""
    try:
        line = input(prompt).strip()
    except (EOFError, KeyboardInterrupt):
        print()
        return None
    return None if line.lower() in ("q", "quit", "exit") else line


def ask_choice(label, options):
    """Pide una opción de 'options' hasta que sea válida. None si el usuario sale."""
    while True:
        answer = ask(f"{label} ({'/'.join(options)}) [q para salir]: ")
        if answer is None:
            return None
        if answer.lower() in options:
            return answer.lower()
        print(f"  '{answer}' no válido, elige entre: {', '.join(options)}")


def read_numbers(prompt, n):
    """Pide n números separados por espacios hasta que sean válidos. None si el usuario sale."""
    while True:
        line = ask(prompt)
        if line is None:
            return None
        parts = line.split()
        if len(parts) != n:
            print(f"  Se esperaban {n} valores, recibidos {len(parts)}")
            continue
        try:
            return tuple(float(p) for p in parts)
        except ValueError:
            print(f"  Los {n} valores deben ser numéricos (decimales con punto)")


def read_values(label):
    while True:
        values = read_numbers(f"[{label}] azimut(deg) elevacion(deg) distancia(cm) [q para salir]: ", 3)
        if values is None or values[2] >= 0:
            return values
        print("  La distancia no puede ser negativa")


def read_cylinder():
    """Pide altura y diámetro de la botella en cm. Devuelve (altura, radio) en m, None si sale."""
    while True:
        values = read_numbers("[botella] altura(cm) diametro(cm) [q para salir]: ", 2)
        if values is None:
            return None
        height_cm, diameter_cm = values
        if height_cm <= 0 or diameter_cm <= 0:
            print("  Las medidas deben ser positivas")
            continue
        if diameter_cm / 100.0 >= GRIPPER_MAX_OPENING:
            print(f"  No cabe en la pinza abierta (máx. {GRIPPER_MAX_OPENING * 100:.1f} cm de diámetro)")
            continue
        return height_cm / 100.0, diameter_cm / 200.0


def read_point(label):
    """Lee un punto en esféricas y lo devuelve en cartesianas. None si el usuario sale."""
    values = read_values(label)
    if values is None:
        return None
    azimuth, elevation, distance_cm = values
    # El terminal recibe cm; los servicios esperan metros
    x, y, z = spherical_to_cartesian(distance_cm / 100.0, azimuth, elevation)
    print(f"  base medida -> x: {x:.4f}, y: {y:.4f}, z: {z:.4f}")
    return x, y, z


def cylinder_center(base, height):
    """Centro de un cilindro de pie a partir del punto medido en su base (sobre su eje)."""
    x, y, z = base
    return x, y, z + height / 2.0


def support_surface(scene, x, y, radius):
    """Superficie más alta de la escena bajo la huella de la botella: (nombre, z) o None."""
    below = [(name, z_top) for name, x_min, x_max, y_min, y_max, z_top in SCENES[scene]
             if x + radius > x_min and x - radius < x_max
             and y + radius > y_min and y - radius < y_max]
    return max(below, key=lambda s: s[1], default=None)


def check_support(scene, base, radius):
    """Avisa si la base de la botella no queda apoyada sobre la superficie de la escena."""
    surface = support_surface(scene, base[0], base[1], radius)
    if surface is None:
        print("  aviso: no hay ninguna superficie de la escena bajo la botella")
        return
    name, surface_z = surface
    gap = base[2] - surface_z
    if abs(gap) <= SURFACE_TOLERANCE:
        return
    state = f"flota {gap * 100:.1f} cm sobre" if gap > 0 else f"se mete {-gap * 100:.1f} cm en"
    print(f"  aviso: la botella {state} el {name} (z = {surface_z} m)")
    distance = math.dist((0.0, 0.0, 0.0), base)
    if distance >= abs(surface_z):
        elevation = math.degrees(math.asin(surface_z / distance))
        print(f"         para apoyarla con el mismo azimut y distancia: elevacion = {elevation:.2f}")


def check_top_grasp(height):
    """Avisa si con top la parte alta de la botella choca con la palma de la pinza."""
    if height / 2.0 > TOP_MAX_HALF_HEIGHT:
        print(f"  aviso: con top, botellas de más de {TOP_MAX_HALF_HEIGHT * 200:.1f} cm chocan con "
              "la palma y no habrá IK; usa side")


def fmt_point(prefix, point):
    x, y, z = point
    return f"{prefix}_x: {x:.4f}, {prefix}_y: {y:.4f}, {prefix}_z: {z:.4f}"


def build_command(service, point, grasp, cylinder=None):
    """Devuelve el 'ros2 service call' para el servicio, punto y grasp dados.
    'point' es el centro del objeto; para pick, 'cylinder' es (altura, radio)."""
    if service == "pick":
        height, radius = cylinder
        return ("ros2 service call /pick_object d1_550_config/srv/PickObject "
                f"\"{{{fmt_point('pick', point)}, shape: {SHAPE}, "
                f"dimension_x: {height:.4f}, dimension_y: {radius:.4f}, pick_grasp: {grasp}}}\"")

    if service == "place":
        return ("ros2 service call /place_object d1_550_config/srv/PlaceObject "
                f"\"{{{fmt_point('place', point)}, place_grasp: {grasp}}}\"")

    raise ValueError(f"Servicio no soportado: '{service}' (usa 'pick' o 'place')")


def call_service(command):
    """Ejecuta el 'ros2 service call' en este terminal y espera la respuesta del servidor."""
    try:
        subprocess.run(shlex.split(command))
    except FileNotFoundError:
        print("  No se encuentra 'ros2': haz source de ROS 2 y del workspace (install/setup.bash)")
    except KeyboardInterrupt:
        print("\n  Llamada interrumpida")


def main():
    # Tiene que coincidir con scene_setup del launch
    scene = ask_choice("escena del launch", SCENES)
    if scene is None:
        return
    last_pick = None  # (cylinder, grasp) del último pick: el place suelta esa misma botella

    while True:
        service = ask_choice("servicio", SERVICES)
        if service is None:
            return
        base = read_point(service)
        if base is None:
            return
        grasp = ask_choice("grasp", GRASPS)
        if grasp is None:
            return

        if service == "place" and last_pick is not None:
            cylinder, pick_grasp = last_pick
            print(f"  botella del último pick: altura {cylinder[0] * 100:.1f} cm, "
                  f"diámetro {cylinder[1] * 200:.1f} cm")
            if grasp != pick_grasp:
                print(f"  aviso: se cogió con {pick_grasp}; con {grasp} el servidor la dejaría tumbada")
        else:
            cylinder = read_cylinder()
            if cylinder is None:
                return
        if service == "pick":
            last_pick = (cylinder, grasp)

        height, radius = cylinder
        check_support(scene, base, radius)
        if grasp == "top":
            check_top_grasp(height)

        center = cylinder_center(base, height)
        print(f"  centro botella -> {fmt_point(service, center)}")
        command = build_command(service, center, grasp, cylinder)
        print(f"  {command}")
        run = ask_choice("¿llamar al servicio?", ("s", "n"))
        if run is None:
            return
        if run == "s":
            call_service(command)
        print()


if __name__ == '__main__':
    main()
