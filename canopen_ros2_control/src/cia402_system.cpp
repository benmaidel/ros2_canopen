// Copyright (c) 2022, StoglRobotics
// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2022-08-01
 *
 */
//----------------------------------------------------------------------

#include "canopen_ros2_control/cia402_system.hpp"
#include "canopen_ros2_control/canopen_system.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <memory>

namespace
{
auto const kLogger = rclcpp::get_logger("Cia402System");
}

namespace canopen_ros2_control
{

Cia402System::Cia402System() : CanopenSystem() {}

hardware_interface::CallbackReturn Cia402System::on_init(
  const hardware_interface::HardwareInfo & info)
{
  auto ret_val = CanopenSystem::on_init(info);
  if (ret_val != hardware_interface::CallbackReturn::SUCCESS)
  {
    return ret_val;
  }

  return CallbackReturn::SUCCESS;
}

void Cia402System::initDeviceContainer()
{
  std::string tmp_master_bin = (info_.hardware_parameters["master_bin"] == "\"\"")
                                 ? ""
                                 : info_.hardware_parameters["master_bin"];

  device_container_->init(
    info_.hardware_parameters["can_interface_name"], info_.hardware_parameters["master_config"],
    info_.hardware_parameters["bus_config"], tmp_master_bin);
  auto drivers = device_container_->get_registered_drivers();
  RCLCPP_INFO(kLogger, "Number of registered drivers: '%lu'", device_container_->count_drivers());
  for (const auto & [node_id, driver] : drivers)
  {
    const auto driver_type = device_container_->get_driver_type(node_id);
    RCLCPP_INFO(kLogger, "Driver type: '%s'", driver_type.c_str());

    auto nmt_state_cb = [this](canopen::NmtState nmt_state, uint8_t id) {
      canopen_data_[id].nmt_state.set_state(nmt_state);
    };
    auto rpdo_cb_cia402 = [this](ros2_canopen::COData data, uint8_t id) {
      canopen_data_[id].rpdo_data.set_data(data);
    };
    auto rpdo_cb_proxy = [this](ros2_canopen::COData data, uint8_t id) {
      canopen_data_[id].set_rpdo_data(data);
    };
    auto emcy_cb = [this](ros2_canopen::COEmcy data, uint8_t id) {
      canopen_data_[id].emcy_data.set_emcy(data);
    };

    if (driver_type == "ros2_canopen::Cia402Driver")
    {
      auto can_driver = std::static_pointer_cast<ros2_canopen::Cia402Driver>(driver);
      can_driver->register_nmt_state_cb(nmt_state_cb);
      can_driver->register_rpdo_cb(rpdo_cb_cia402);
    }
    else if (driver_type == "ros2_canopen::ProxyDriver")
    {
      auto can_driver = std::static_pointer_cast<ros2_canopen::ProxyDriver>(driver);
      can_driver->register_nmt_state_cb(nmt_state_cb);
      can_driver->register_rpdo_cb(rpdo_cb_proxy);
      can_driver->register_emcy_cb(emcy_cb);
    }
    else {
      RCLCPP_ERROR(kLogger, "Unknown driver type '%s' for NodeID 0x%X", driver_type.c_str(), node_id);
      continue;
    }

    RCLCPP_INFO(
      kLogger, "\nRegistered driver:\n    name: '%s'\n    node_id: '0x%X'",
      driver->get_node_base_interface()->get_name(), node_id);
  }

  RCLCPP_INFO(device_container_->get_logger(), "Initialisation successful.");
}

hardware_interface::CallbackReturn Cia402System::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  device_container_ = std::make_shared<ros2_canopen::DeviceContainer>(executor_);
  executor_->add_node(device_container_);

  // threads
  spin_thread_ = std::make_unique<std::thread>(&Cia402System::spin, this);
  init_thread_ = std::make_unique<std::thread>(&Cia402System::initDeviceContainer, this);

  // actually wait for init phase to end
  if (init_thread_->joinable())
  {
    init_thread_->join();

    // TODO(livanov93): see how to handle configure once LifecycleCia402Driver is introduced
    /*
    auto drivers = device_container_->get_registered_drivers();
    for (auto it = drivers.begin(); it != drivers.end(); it++) {
        auto d = std::static_pointer_cast<ros2_canopen::LifecycleCia402Driver>(it->second);
        d->configure();
    }
    */
  }
  else
  {
    RCLCPP_ERROR(kLogger, "Could not join init thread!");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Cia402System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // underlying base class export first
  state_interfaces = CanopenSystem::export_state_interfaces();

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    if (info_.joints[i].parameters.find("node_id") == info_.joints[i].parameters.end())
    {
      // skip adding motor canopen interfaces
      continue;
    }
    const uint8_t node_id = static_cast<uint8_t>(std::stoi(info_.joints[i].parameters["node_id"]));

    // actual position
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &motor_data_[node_id].actual_position));
    // actual speed
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &motor_data_[node_id].actual_speed));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Cia402System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // underlying base class export first
  command_interfaces = CanopenSystem::export_command_interfaces();

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    if (info_.joints[i].parameters.find("node_id") == info_.joints[i].parameters.end())
    {
      // skip adding canopen interfaces
      continue;
    }

    const uint8_t node_id = static_cast<uint8_t>(std::stoi(info_.joints[i].parameters["node_id"]));

    // target
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &motor_data_[node_id].target.position_value));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &motor_data_[node_id].target.velocity_value));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
      &motor_data_[node_id].target.torque_value));
    // init
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "init_cmd", &motor_data_[node_id].init.ons_cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "init_fbk", &motor_data_[node_id].init.resp));

    // halt
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "halt_cmd", &motor_data_[node_id].halt.ons_cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "halt_fbk", &motor_data_[node_id].halt.resp));

    // recover
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "recover_cmd", &motor_data_[node_id].recover.ons_cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "recover_fbk", &motor_data_[node_id].recover.resp));

    // set position mode
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "position_mode_cmd", &motor_data_[node_id].position_mode.ons_cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "position_mode_fbk", &motor_data_[node_id].position_mode.resp));

    // set velocity mode
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "velocity_mode_cmd", &motor_data_[node_id].velocity_mode.ons_cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "velocity_mode_fbk", &motor_data_[node_id].velocity_mode.resp));

    // set cyclic velocity mode
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "cyclic_velocity_mode_cmd",
      &motor_data_[node_id].cyclic_velocity_mode.ons_cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "cyclic_velocity_mode_fbk",
      &motor_data_[node_id].cyclic_velocity_mode.resp));
    // set cyclic position mode
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "cyclic_position_mode_cmd",
      &motor_data_[node_id].cyclic_position_mode.ons_cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "cyclic_position_mode_fbk",
      &motor_data_[node_id].cyclic_position_mode.resp));
    // set interpolated position mode
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "interpolated_position_mode_cmd",
      &motor_data_[node_id].interpolated_position_mode.ons_cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "interpolated_position_mode_fbk",
      &motor_data_[node_id].interpolated_position_mode.resp));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn Cia402System::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  return CanopenSystem::on_activate(previous_state);
}

hardware_interface::CallbackReturn Cia402System::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  return CanopenSystem::on_deactivate(previous_state);
}

hardware_interface::return_type Cia402System::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // TODO(anyone): read robot states

  auto ret_val = CanopenSystem::read(time, period);

  auto drivers = device_container_->get_registered_drivers();

  for (const auto & [node_id, canopen_data] : canopen_data_)
  {
    const auto driver_type = device_container_->get_driver_type(node_id);
    if (driver_type.compare("ros2_canopen::Cia402Driver") == 0)
    {
      auto motion_controller_driver =
      std::static_pointer_cast<ros2_canopen::Cia402Driver>(drivers[node_id]);
      // get position
      motor_data_[node_id].actual_position = motion_controller_driver->get_position();
      // get speed
      motor_data_[node_id].actual_speed = motion_controller_driver->get_speed();
    }
  }

  return ret_val;
}

hardware_interface::return_type Cia402System::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto drivers = device_container_->get_registered_drivers();

  for (auto it = canopen_data_.begin(); it != canopen_data_.end(); ++it)
  {
    const auto driver_type = device_container_->get_driver_type(it->first);
    if (driver_type.compare("ros2_canopen::Cia402Driver") == 0)
    {

      // TODO(livanov93): check casting
      auto motion_controller_driver =
        std::static_pointer_cast<ros2_canopen::Cia402Driver>(drivers[it->first]);
      // do same as in proxy system first - handle nmt, tpdo, rpdo
      // reset node nmt
      if (it->second.nmt_state.reset_command())
      {
        motion_controller_driver->reset_node_nmt_command();
      }

      // start nmt
      if (it->second.nmt_state.start_command())
      {
        motion_controller_driver->start_node_nmt_command();
      }

      // tpdo data one shot mechanism
      if (it->second.tpdo_data.write_command())
      {
        it->second.tpdo_data.prepare_data();
        motion_controller_driver->tpdo_transmit(it->second.tpdo_data.original_data);
      }

      // initialisation
      handleInit(it->first, motion_controller_driver);

      // halt
      handleHalt(it->first, motion_controller_driver);

      // recover
      handleRecover(it->first, motion_controller_driver);

      // mode switching
      switchModes(it->first, motion_controller_driver);

      const uint16_t & mode = motion_controller_driver->get_mode();

      switch (mode)
      {
        case MotorBase::No_Mode:
          break;
        case MotorBase::Profiled_Position:
        case MotorBase::Cyclic_Synchronous_Position:
        case MotorBase::Interpolated_Position:
          motion_controller_driver->set_target(motor_data_[it->first].target.position_value);
          break;
        case MotorBase::Profiled_Velocity:
        case MotorBase::Cyclic_Synchronous_Velocity:
          motion_controller_driver->set_target(motor_data_[it->first].target.velocity_value);
          break;
        case MotorBase::Profiled_Torque:
          motion_controller_driver->set_target(motor_data_[it->first].target.torque_value);
          break;
        default:
          RCLCPP_INFO(kLogger, "Mode %u not supported", mode);
      }
    }
    else
    {
      if (drivers.find(it->first) == drivers.end())
      {
        // this is expeced for NodeID 0x00 - why do we have it at all?
        RCLCPP_DEBUG(kLogger, "Driver for NodeID 0x%X not found. Skipping...", it->first);
        continue;
      }
      auto proxy_driver = std::static_pointer_cast<ros2_canopen::ProxyDriver>(drivers[it->first]);

      // reset node nmt
      if (it->second.nmt_state.reset_command())
      {
        it->second.nmt_state.reset_fbk = static_cast<double>(proxy_driver->reset_node_nmt_command());
      }

      // start nmt
      if (it->second.nmt_state.start_command())
      {
        it->second.nmt_state.start_fbk = static_cast<double>(proxy_driver->start_node_nmt_command());
      }

      // canopen_ros2_control::WORos2ControlCoData data;
      // data.index = 0x2110;
      // data.subindex = 0x00;
      // data.data = int16_t(123);
      // data.prepare_data();
      // RCLCPP_INFO(kLogger, "This is a debug message in HW-write().....");
      // RCLCPP_INFO(kLogger, "NodeID: 0x%X; Index: 0x%X; Subindex: 0x%X; Data: %u",
      //   it->first,
      //   data.original_data.index_,
      //   data.original_data.subindex_,
      //   data.original_data.data_);
      // RCLCPP_INFO(kLogger, "--- END of the debug message in HW-write()");
      // proxy_driver->tpdo_transmit(data.original_data);

      // tpdo data one shot mechanism
      if (it->second.tpdo_data.write_command())
      {
        it->second.tpdo_data.prepare_data();
        try
        {
          proxy_driver->tpdo_transmit(it->second.tpdo_data.original_data);
        }
        catch(const std::exception& e)
        {
          std::cerr << e.what() << '\n';
        }
      }
    }
  }

  return hardware_interface::return_type::OK;
}

void Cia402System::switchModes(uint id, const std::shared_ptr<ros2_canopen::Cia402Driver> & driver)
{
  if (motor_data_[id].position_mode.is_commanded())
  {
    motor_data_[id].position_mode.set_response(
      driver->set_operation_mode(MotorBase::Profiled_Position));
  }

  if (motor_data_[id].cyclic_position_mode.is_commanded())
  {
    motor_data_[id].cyclic_position_mode.set_response(
      driver->set_operation_mode(MotorBase::Cyclic_Synchronous_Position));
  }

  if (motor_data_[id].velocity_mode.is_commanded())
  {
    motor_data_[id].velocity_mode.set_response(
      driver->set_operation_mode(MotorBase::Profiled_Velocity));
  }

  if (motor_data_[id].cyclic_velocity_mode.is_commanded())
  {
    motor_data_[id].cyclic_velocity_mode.set_response(
      driver->set_operation_mode(MotorBase::Cyclic_Synchronous_Velocity));
  }

  if (motor_data_[id].torque_mode.is_commanded())
  {
    motor_data_[id].torque_mode.set_response(
      driver->set_operation_mode(MotorBase::Profiled_Torque));
  }

  if (motor_data_[id].interpolated_position_mode.is_commanded())
  {
    motor_data_[id].interpolated_position_mode.set_response(
      driver->set_operation_mode(MotorBase::Interpolated_Position));
  }
}

void Cia402System::handleInit(uint id, const std::shared_ptr<ros2_canopen::Cia402Driver> & driver)
{
  if (motor_data_[id].init.is_commanded())
  {
    motor_data_[id].init.set_response(driver->init_motor());
  }
}

void Cia402System::handleRecover(
  uint id, const std::shared_ptr<ros2_canopen::Cia402Driver> & driver)
{
  if (motor_data_[id].recover.is_commanded())
  {
    motor_data_[id].recover.set_response(driver->recover_motor());
  }
}

void Cia402System::handleHalt(uint id, const std::shared_ptr<ros2_canopen::Cia402Driver> & driver)
{
  if (motor_data_[id].halt.is_commanded())
  {
    motor_data_[id].halt.set_response(driver->halt_motor());
  }
}

}  // namespace canopen_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(canopen_ros2_control::Cia402System, hardware_interface::SystemInterface)
