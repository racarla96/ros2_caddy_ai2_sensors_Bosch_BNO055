cmake_minimum_required(VERSION 3.8)
project(ros2_caddy_ai2_sensors_bosch_bno055)

# Asegúrate de que los compiladores soporten C++23 (o la versión que prefieras)
add_compile_options(-std=c++23)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(ros2_control REQUIRED)
find_package(sensor_interfaces REQUIRED)

include_directories(
  hardware/include
)

# Añadir el ejecutable del hardware interface
add_library(${PROJECT_NAME} SHARED
  hardware/src/bosch_bno055_sensor_hardware_interface.cpp
)

# Definir dependencias para la biblioteca
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  ros2_control
  sensor_interfaces  # Dependencia del paquete de mensajes AHRS
)

# Instala la biblioteca y el archivo de configuración
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# Instalar archivos de configuración y lanzamiento
install(DIRECTORY bringup/config/
    DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY bringup/launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

# Para ROS2 Control plugins (opcional, si utilizas ros2_control)
install(FILES bosch_bno055_sensor_hardware_interface_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake rclcpp hardware_interface ros2_control sensor_interfaces)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
