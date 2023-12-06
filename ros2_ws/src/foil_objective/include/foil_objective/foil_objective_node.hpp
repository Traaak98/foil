#ifndef BUILD_FOIL_OBJECTIVE_NODE_H
#define BUILD_FOIL_OBJECTIVE_NODE_H

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float32.hpp"

#include "foil_objective_msg/msg/foil_objective.hpp"

using namespace std::chrono_literals;
using namespace std;

class FoilObjectiveNode : public rclcpp::Node {
public:
    FoilObjectiveNode();

    ~FoilObjectiveNode();
private:
    double x_objective_ = 0.0;
    double y_objective_ = 0.0;
    double z_objective_ = 0.0;

    double roll_objective_ = 0.0;
    double pitch_objective_ = 0.0;

    double speed_objective_ = 0.0;

    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 100ms; // loop dt

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_foil_objective_position_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_foil_objective_speed_;
    rclcpp::Publisher<foil_objective_msg::msg::FoilObjective>::SharedPtr publisher_foil_objective_;

    void init_parameters();
    void init_interfaces();
    void timer_callback();
    void foil_objective_position_callback(const geometry_msgs::msg::Point::SharedPtr msg);
    void foil_objective_speed_callback(const std_msgs::msg::Float32::SharedPtr msg);

};

#endif //BUILD_FOIL_OBJECTIVE_NODE_H
