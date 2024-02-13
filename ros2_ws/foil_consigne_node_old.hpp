#ifndef BUILD_FOIL_CONSIGNE_NODE_H
#define BUILD_FOIL_CONSIGNE_NODE_H

#include <memory>
#include <Eigen/Core>
#include <Eigen/LU>
#include <math.h>
#include "rclcpp/rclcpp.hpp"


#include "geometry_msgs/msg/point.hpp"
#include "custom_msg/msg/foil_state.hpp"
#include "custom_msg/msg/foil_objective.hpp"
#include "custom_msg/msg/foil_consigne.hpp"
#include "custom_msg/msg/param_consigne.hpp"


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

    double speed_ = 0.0;

    double x_objective_ = 0.0;
    double y_objective_ = 0.0;
    double z_objective_ = 0.0;

    double roll_objective_ = 0.0;
    double pitch_objective_ = 0.0;
    double yaw_objective_ = 0.0;

    double speed_objective_ = 0.0;

    double height_est_ = 0.0;

    double l = 2.2;
    double d = 0.52;

    double kz_ = 0.0;
    double kroll_ = 0.0;
    double kpitch_ = 0.0;
    double kspeed_ = 0.0;

    double kz_proportional_ = 0.0;
    double kroll_proportional_ = 0.0;
    double kpitch_proportional_ = 0.0;
    double kyaw_proportional_ = 0.0;

    Matrix <double, 3, 3> Model_ {
        {1.0, 1.0, 1.0},
        {d, -d, 0.0},
        {0.0, 0.0, l}
    };
    Matrix <double, 3, 3> Model_inv_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 100ms; // loop dt
    
    // TODO: Add Publisher for consigne message
    rclcpp::Subscription<custom_msg::msg::FoilState>::SharedPtr subscription_foil_state_;
    rclcpp::Subscription<custom_msg::msg::FoilObjective>::SharedPtr subscription_foil_objective_;
    rclcpp::Publisher<custom_msg::msg::FoilConsigne>::SharedPtr publisher_foil_consigne_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_forces_actionneurs_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_forces_angles_;
    rclcpp::Publisher<custom_msg::msg::ParamConsigne>::SharedPtr publisher_parametres_consigne_;


    void init_parameters();
    void init_interfaces();
    void timer_callback();
    void foil_state_callback(const custom_msg::msg::FoilState::SharedPtr msg);
    void foil_objective_callback(const custom_msg::msg::FoilObjective::SharedPtr msg);
};

#endif //BUILD_FOIL_CONSIGNE_NODE_H