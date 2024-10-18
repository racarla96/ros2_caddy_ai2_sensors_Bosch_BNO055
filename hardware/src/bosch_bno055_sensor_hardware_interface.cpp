#include "ros2_caddy_ai2_sensors_bosch_bno055/bosch_bno055_sensor_hardware_interface.hpp"

namespace bosch_bno055_sensor_hardware_interface
{

hardware_interface::CallbackReturn BoschBNO055SensorHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SensorInterface::on_init(info) != 
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Inicialización del sensor (simulada o real)
    ahrs_msg_.header.frame_id = "imu_link";
    ahrs_msg_.sensor_clock = 0;  // Inicializa el clock del sensor a 0
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BoschBNO055SensorHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Aquí defines las interfaces de estado que exporta el hardware, en base a los datos del mensaje AHRS
    state_interfaces.emplace_back(hardware_interface::StateInterface("ahrs", "orientation_x", &ahrs_msg_.orientation.x));
    state_interfaces.emplace_back(hardware_interface::StateInterface("ahrs", "orientation_y", &ahrs_msg_.orientation.y));
    state_interfaces.emplace_back(hardware_interface::StateInterface("ahrs", "orientation_z", &ahrs_msg_.orientation.z));
    state_interfaces.emplace_back(hardware_interface::StateInterface("ahrs", "orientation_w", &ahrs_msg_.orientation.w));
    
    // Exportar más interfaces, como velocidad angular, aceleración, etc.
    state_interfaces.emplace_back(hardware_interface::StateInterface("ahrs", "angular_velocity_x", &ahrs_msg_.angular_velocity.x));
    state_interfaces.emplace_back(hardware_interface::StateInterface("ahrs", "linear_acceleration_x", &ahrs_msg_.linear_acceleration.x));
    
    // Continúa para las demás interfaces...

    return state_interfaces;
}

hardware_interface::return_type BoschBNO055SensorHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // Simulación de lectura de datos del sensor BNO055
    ahrs_msg_.header.stamp = rclcpp::Clock().now();  // Actualiza la marca de tiempo
    ahrs_msg_.sensor_clock += 1;  // Incrementa el clock del sensor

    // Simulación de datos (esto debería reemplazarse con datos reales del sensor BNO055)
    ahrs_msg_.orientation.x += 0.01;
    ahrs_msg_.orientation.y += 0.01;
    ahrs_msg_.orientation.z += 0.01;
    ahrs_msg_.orientation.w = 1.0;

    // Actualiza más datos del sensor como velocidad angular, aceleración, campo magnético...
    ahrs_msg_.angular_velocity.x = 0.1;
    ahrs_msg_.linear_acceleration.x = 9.81;  // Gravedad simulada

    return hardware_interface::return_type::OK;
}

}  // namespace bosch_bno055_sensor_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  bosch_bno055_sensor_hardware_interface::BoschBNO055SensorHardwareInterface,
  hardware_interface::SensorInterface)