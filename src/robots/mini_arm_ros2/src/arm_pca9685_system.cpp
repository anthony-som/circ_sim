#include "mini_arm_ros2/arm_pca9685_system.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace mini_arm_ros2
{

hardware_interface::CallbackReturn ArmPCA9685System::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Optional params from URDF <hardware> <param name="...">
  if (info_.hardware_parameters.count("command_topic")) {
    command_topic_ = info_.hardware_parameters.at("command_topic");
  }
  if (info_.hardware_parameters.count("center_deg")) {
    center_deg_ = std::stod(info_.hardware_parameters.at("center_deg"));
  }
  if (info_.hardware_parameters.count("deg_min")) {
    deg_min_ = std::stod(info_.hardware_parameters.at("deg_min"));
  }
  if (info_.hardware_parameters.count("deg_max")) {
    deg_max_ = std::stod(info_.hardware_parameters.at("deg_max"));
  }

  const size_t n = info_.joints.size();
  hw_states_pos_.assign(n, 0.0);
  hw_commands_pos_.assign(n, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArmPCA9685System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_pos_[i])
    );
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmPCA9685System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_pos_[i])
    );
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn ArmPCA9685System::on_configure(const rclcpp_lifecycle::State &)
{
  if (!rclcpp::ok()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  node_ = std::make_shared<rclcpp::Node>("arm_pca9685_hw");
  pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(command_topic_, 10);

  exec_.add_node(node_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmPCA9685System::on_activate(const rclcpp_lifecycle::State &)
{
  active_ = true;

  // On activate, publish a "center" pose (all zeros -> 90 degrees mapping)
  std_msgs::msg::Float32MultiArray out;
  out.data.resize(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); i++) {
    float deg = static_cast<float>(center_deg_);
    deg = clamp_f(deg, static_cast<float>(deg_min_), static_cast<float>(deg_max_));
    out.data[i] = deg;
    hw_states_pos_[i] = hw_commands_pos_[i];
  }

  pub_->publish(out);
  exec_.spin_some();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmPCA9685System::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArmPCA9685System::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Open loop: assume robot achieved commanded position
  for (size_t i = 0; i < hw_states_pos_.size(); i++) {
    hw_states_pos_[i] = hw_commands_pos_[i];
  }
  exec_.spin_some();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmPCA9685System::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_ || !pub_) {
    return hardware_interface::return_type::OK;
  }

  std_msgs::msg::Float32MultiArray out;
  out.data.resize(hw_commands_pos_.size());

  // Mapping: rad -> degrees with 0 rad = 90 deg
  for (size_t i = 0; i < hw_commands_pos_.size(); i++) {
    double rad = hw_commands_pos_[i];
    // Flip direction only for joint index 1 (shoulder_joint)
    if (i == 0 || i== 1) {
      rad = -rad;
    }
    double deg = center_deg_ + rad * rad_to_deg_scale_;
    float deg_f = clamp_f(static_cast<float>(deg),
                          static_cast<float>(deg_min_),
                          static_cast<float>(deg_max_));
    out.data[i] = deg_f;
  }

  pub_->publish(out);
  exec_.spin_some();
  return hardware_interface::return_type::OK;
}

}  // namespace mini_arm_ros2

PLUGINLIB_EXPORT_CLASS(mini_arm_ros2::ArmPCA9685System, hardware_interface::SystemInterface)
