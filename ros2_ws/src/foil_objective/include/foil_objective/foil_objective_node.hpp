#ifndef BUILD_FOIL_OBJECTIVE_NODE_H
#define BUILD_FOIL_OBJECTIVE_NODE_H

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float32.hpp"
#include <proj.h>       // PROJ library
#include <memory>
#include <cmath>

#include "custom_msg/msg/foil_objective.hpp"
#include "custom_msg/msg/foil_state.hpp"

using namespace std::chrono_literals;
using namespace std;

class FoilObjectiveNode : public rclcpp::Node {
public:
    FoilObjectiveNode();

    ~FoilObjectiveNode();
private:

    // Définir un point gps à transformer 
    double lat_ = 48.19949;
    double lon_ = -3.01573;

    PJ_CONTEXT *C;
    PJ *P;
    PJ *norm;
    PJ_COORD a, b;

    double x_ = 0.0;
    double y_ = 0.0;
    double z_ = 0.0;

    double roll_ = 0.0;
    double pitch_ = 0.0;
    double yaw_ = 0.0;

    double speed_x_ = 0.0;
    double speed_y_ = 0.0;
    double speed_z_ = 0.0;
    
    // p1 p2 les points de la trajectoire à suivre
    double *p1_ = {7146300., -534400.};
    double *p2_ = {7146550., -533600.0};

    double x_objective_ = 0.0;
    double y_objective_ = 0.0;
    double z_objective_ = 1.; // Pour simuler max de la vitesse

    double roll_objective_ = 0.0;
    double pitch_objective_ = 0.0;
    double yaw_objective_ = 0.0;

    bool objective_ = false;

    double R_ = 10.0; //* Rayon de la zone d'arrivée

    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 100ms; // loop dt

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_foil_objective_position_;
    rclcpp::Subscription<custom_msg::msg::FoilState>::SharedPtr subscription_foil_state_;
    rclcpp::Publisher<custom_msg::msg::FoilObjective>::SharedPtr publisher_foil_objective_;

    void init_parameters();
    void init_interfaces();
    void timer_callback();
    void foil_objective_position_callback(const geometry_msgs::msg::Point::SharedPtr msg);
    void foil_state_callback(const custom_msg::msg::FoilState::SharedPtr msg);
    void end_objective();
    void find_theta_objective();
};

#endif //BUILD_FOIL_OBJECTIVE_NODE_H
