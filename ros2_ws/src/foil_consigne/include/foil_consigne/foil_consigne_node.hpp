#ifndef BUILD_FOIL_CONSIGNE_NODE_H
#define BUILD_FOIL_CONSIGNE_NODE_H

#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "custom_msg/msg/foil_state.hpp"
#include "custom_msg/msg/foil_objective.hpp"
#include "custom_msg/msg/foil_consigne.hpp"
#include "custom_msg/msg/param_consigne.hpp"

using namespace std::chrono_literals;
using namespace std;

const double SPEED_OFFSET = 0.8;
const double PI = 3.14159265358979323846;
const double Z_OFFSET = 0.34;

class FoilConsigneNode : public rclcpp::Node {
public:
    FoilConsigneNode();

    ~FoilConsigneNode();

private:

    double x_ = 0.0;
    double y_ = 0.0;
    double z_ = 0.0;
    double roll_ = 0.0;
    double pitch_ = 0.0;
    double yaw_ = 0.0;
    double speed_x_ = 0.0;
    double speed_y_ = 0.0;
    double speed_z_ = 0.0;
    double speed_ = 0.0;
    double height_est_ = 0.0;

    double x_objective_ = 0.0;
    double y_objective_ = 0.0;
    double z_objective_ = 0.0;
    double roll_objective_ = 0.0;
    double pitch_objective_ = 0.0;
    double yaw_objective_ = 0.0;
    bool objective_ = false;

    double kpitch_ = 0.0;
    double kspeed_ = 0.0;
    double kroll_ = 0.0;

    double foil_regulation = 0.0;
    double speed_regulation = 0.0;
    double roll_regulation = 0.0;


    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 100ms; // loop dt

    rclcpp::Subscription<custom_msg::msg::FoilState>::SharedPtr subscription_foil_state_;
    rclcpp::Subscription<custom_msg::msg::FoilObjective>::SharedPtr subscription_foil_objective_;
    rclcpp::Publisher<custom_msg::msg::FoilConsigne>::SharedPtr publisher_foil_consigne_;
    rclcpp::Publisher<custom_msg::msg::ParamConsigne>::SharedPtr publisher_parametres_consigne_;

    void init_parameters();
    void init_interfaces();
    void foil_state_callback(const custom_msg::msg::FoilState::SharedPtr msg);
    void foil_objective_callback(const custom_msg::msg::FoilObjective::SharedPtr msg);
    void timer_callback();
    void read_parameters();
    void regulation_pitch();
    void regulation_speed();
    void regulation_roll();
};

#endif //BUILD_FOIL_CONSIGNE_NODE_V2_H