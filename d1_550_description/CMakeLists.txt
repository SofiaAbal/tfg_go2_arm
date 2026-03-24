#cmake_minimum_required(VERSION 2.8.3)

#project(d1_550_description)

#find_package(catkin REQUIRED)

#catkin_package()

#find_package(roslaunch)

#foreach(dir config launch meshes urdf)
#	install(DIRECTORY ${dir}/
#		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
#endforeach(dir)

cmake_minimum_required(VERSION 3.5)

project(d1_550_description)

# Busca ament_cmake, obligatorio en ROS2
find_package(ament_cmake REQUIRED)

# Instala los directorios que contiene tu URDF, launch, meshes, config, etc.
install(DIRECTORY config launch meshes urdf textures
  DESTINATION share/${PROJECT_NAME}
)

# Registra el paquete en ROS2
ament_package()