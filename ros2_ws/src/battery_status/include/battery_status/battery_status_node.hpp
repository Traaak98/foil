#ifndef BATTERY_STATUS_NODE_H
#define BATTERY_STATUS_NODE_H

#include <memory>
#include <fstream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

using namespace std::chrono_literals;
using namespace std;

string VOLTAGE_PATH = "/sys/class/power_supply/BAT1/voltage_now";
string CURRENT_PATH = "/sys/class/power_supply/BAT1/current_now";
string CHARGE_PATH = "/sys/class/power_supply/BAT1/charge_now";
string CAPACITY_PATH = "/sys/class/power_supply/BAT1/charge_full";
string DESIGN_CAPACITY_PATH = "/sys/class/power_supply/BAT1/charge_full_design";
string PERCENTAGE_PATH = "/sys/class/power_supply/BAT1/capacity";

class BatteryStatusNode : public rclcpp::Node {
public:
    BatteryStatusNode();

    ~BatteryStatusNode();

private:
    double voltage_ = 0.0;
    double current_ = 0.0;
    double charge_ = 0.0;
    double capacity_ = 0.0;
    double design_capacity_ = 0.0;
    double percentage_ = 0.0;

    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 10000ms; // loop dt

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher_battery_state_;

    void init_parameters();
    void init_interfaces();
    void timer_callback();
    void read_battery_status();
    void read_voltage();
    void read_current();
    void read_charge();
    void read_capacity();
    void read_design_capacity();
    void read_percentage();
    void battery_state_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
};

#endif // BATTERY_STATUS_NODE_H
