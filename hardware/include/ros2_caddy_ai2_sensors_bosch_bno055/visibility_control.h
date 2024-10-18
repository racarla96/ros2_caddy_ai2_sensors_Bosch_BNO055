#ifndef BOSCH_BNO055_SENSOR_HARDWARE_INTERFACE_HPP_
#define BOSCH_BNO055_SENSOR_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "visibility_control.h"  // Visibility control header (opcional)
#include "sensor_interfaces/msg/ahrs.hpp"  // Importamos el mensaje AHRS

namespace bosch_bno055_sensor_hardware
{

class BoschBNO055SensorHardwareInterface : public hardware_interface::SensorInterface
{
public:
    // Constructor sin parámetros
    BoschBNO055SensorHardwareInterface() = default;

    // Métodos del ciclo de vida del hardware
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) override;
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override;
    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & /*previous_state*/) override;

    // Métodos de SensorInterface
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    hardware_interface::return_type read() override;

private:
    // Estructura de mensaje AHRS para almacenar todos los valores del sensor
    sensor_interfaces::msg::AHRS ahrs_msg_;
};

}  // namespace bosch_bno055_sensor_hardware

#endif  // BOSCH_BNO055_SENSOR_HARDWARE_INTERFACE_HPP_
