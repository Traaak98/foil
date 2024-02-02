#ifndef BUILD_FOIL_CONSIGNE_NODE_H
#define BUILD_FOIL_CONSIGNE_NODE_H

#include <memory>
#include <Eigen/Core>
#include "rclcpp/rclcpp.hpp"

#include "foil_state_msg/msg/foil_state.hpp"
#include "foil_objective_msg/msg/foil_objective.hpp"
#include "foil_consigne_msg/msg/foil_consigne.hpp"

using namespace std::chrono_literals;
using namespace std;
using namespace Eigen;

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

    double x_objective_ = 0.0;
    double y_objective_ = 0.0;
    double z_objective_ = 0.0;

    double roll_objective_ = 0.0;
    double pitch_objective_ = 0.0;

    double speed_objective_ = 0.0;

    Matrix <double, 3, 3> Model_;
    Matrix <double, 3, 3> Model_inv_;

    double l = 1.0; // TODO: set this parameter
    double d = 1.0; // TODO: set this parameter

    Model << 1.0 1.0 1.0,
             d -d 0.0,
             0.0 0.0 l;

    Model_inv_ = Model.inverse();

    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 100ms; // loop dt
    
    // TODO: Add Publisher for consigne message
    rclcpp::Subscription<foil_state_msg::msg::FoilState>::SharedPtr subscription_foil_state_;
    rclcpp::Subscription<foil_objective_msg::msg::FoilObjective>::SharedPtr subscription_foil_objective_;
    rclcpp::Publisher<foil_consigne_msg::msg::FoilConsigne>::SharedPtr publisher_foil_consigne_;

    void init_parameters();
    void init_interfaces();
    void timer_callback();
    void foil_state_callback(const foil_state_msg::msg::FoilState::SharedPtr msg);
    void foil_objective_callback(const foil_objective_msg::msg::FoilObjective::SharedPtr msg);
};

#endif //BUILD_FOIL_CONSIGNE_NODE_H