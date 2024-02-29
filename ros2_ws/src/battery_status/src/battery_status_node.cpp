#include "battery_status/battery_status_node.hpp"

BatteryStatusNode::BatteryStatusNode() : Node("battery_status_node")
{
  init_parameters();
  init_interfaces();

  timer_ = this->create_wall_timer(
    loop_dt_, std::bind(&BatteryStatusNode::timer_callback, this));
}

BatteryStatusNode::~BatteryStatusNode()
{
}

void BatteryStatusNode::init_parameters()
{
}

void BatteryStatusNode::init_interfaces()
{
  publisher_battery_state_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
}

void BatteryStatusNode::timer_callback()
{
  read_battery_status();

  auto msg = sensor_msgs::msg::BatteryState();
  msg.header.stamp = this->now();
  msg.header.frame_id = "world";

  msg.voltage = this->voltage_;
  msg.current = this->current_;
  msg.charge = this->charge_;
  msg.capacity = this->capacity_;
  msg.design_capacity = this->design_capacity_;
  msg.percentage = this->percentage_;
  msg.location = this->location_;

  publisher_battery_state_->publish(msg);
}

void BatteryStatusNode::read_battery_status()
{
  read_voltage();
  read_current();
  read_charge();
  read_capacity();
  read_design_capacity();
  read_percentage();
}

void BatteryStatusNode::read_voltage()
{
  std::ifstream file(VOLTAGE_PATH);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Error: Unable to open file %s", VOLTAGE_PATH.c_str());
  }

  file >> this->voltage_;
  file.close();

  // La tension est souvent donnée en microvolts, donc la convertir en volts
  this->voltage_ /= 1000000.0;
}

void BatteryStatusNode::read_current()
{
  std::ifstream file(CURRENT_PATH);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Error: Unable to open file %s", CURRENT_PATH.c_str());
  }

  file >> this->current_;
  file.close();

  // Le courant est souvent donné en microampères, donc le convertir en ampères
  this->current_ /= -1000000.0;
}

void BatteryStatusNode::read_charge()
{
  std::ifstream file(CHARGE_PATH);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Error: Unable to open file %s", CHARGE_PATH.c_str());
  }

  file >> this->charge_;
  file.close();

  // La charge est souvent donnée en microampères-heures, donc la convertir en ampères-heures
  this->charge_ /= 1000000.0;
}

void BatteryStatusNode::read_capacity()
{
  std::ifstream file(CAPACITY_PATH);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Error: Unable to open file %s", CAPACITY_PATH.c_str());
  }

  file >> this->capacity_;
  file.close();

  // La capacité est souvent donnée en microampères-heures, donc la convertir en ampères-heures
  this->capacity_ /= 1000000.0;
}

void BatteryStatusNode::read_design_capacity()
{
  std::ifstream file(DESIGN_CAPACITY_PATH);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Error: Unable to open file %s", DESIGN_CAPACITY_PATH.c_str());
  }

  file >> this->design_capacity_;
  file.close();

  // La capacité est souvent donnée en microampères-heures, donc la convertir en ampères-heures
  this->design_capacity_ /= 1000000.0;
}

void BatteryStatusNode::read_percentage()
{
  std::ifstream file(PERCENTAGE_PATH);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Error: Unable to open file %s", PERCENTAGE_PATH.c_str());
  }

  file >> this->percentage_;
  file.close();

  // La capacité est souvent donnée en pourcentage, donc la convertir en fraction
  this->percentage_ /= 100.0;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryStatusNode>());
  rclcpp::shutdown();
  return 0;
}
