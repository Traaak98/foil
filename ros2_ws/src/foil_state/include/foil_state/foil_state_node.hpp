#ifndef BUILD_FOIL_STATE_NODE_H
#define BUILD_FOIL_STATE_NODE_H

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "sbg_driver/msg/sbg_ekf_euler.hpp"
#include "sbg_driver/msg/sbg_gps_vel.hpp"
#include "sbg_driver/msg/sbg_gps_hdt.hpp"
#include "sbg_driver/msg/sbg_gps_pos.hpp"
#include "foil_state_msg/msg/foil_state.hpp"
#include "foil_height_sensor_message/msg/foil_height.hpp"

using namespace std::chrono_literals;
using namespace std;

class FoilStateNode : public rclcpp::Node {
public:
    FoilStateNode();

    ~FoilStateNode();

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

    double height_left_ = 0.0;
    double height_right_ = 0.0;
    double height_rear_ = 0.0;
    double height_potar_ = 0.0;

    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds loop_dt_ = 100ms; // loop dt

    rclcpp::Subscription<sbg_driver::msg::SbgEkfEuler>::SharedPtr subscription_sbg_ekf_euler_;
    rclcpp::Subscription<sbg_driver::msg::SbgGpsVel>::SharedPtr subscription_sbg_gps_vel_;
    rclcpp::Subscription<sbg_driver::msg::SbgGpsHdt>::SharedPtr subscription_sbg_gps_hdt_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_utm_pose;
    rclcpp::Subscription<foil_height_sensor_message::msg::FoilHeight>::SharedPtr subscription_foil_height_;
    rclcpp::Publisher<foil_state_msg::msg::FoilState>::SharedPtr publisher_foil_state_;

    void init_parameters();
    void init_interfaces();
    void timer_callback();
    void sbg_ekf_euler_callback(const sbg_driver::msg::SbgEkfEuler::SharedPtr msg);
    void sbg_gps_vel_callback(const sbg_driver::msg::SbgGpsVel::SharedPtr msg);
    void sbg_gps_hdt_callback(const sbg_driver::msg::SbgGpsHdt::SharedPtr msg);
    void utm_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void foil_height_callback(const foil_height_sensor_message::msg::FoilHeight::SharedPtr msg);
};

#endif //BUILD_FOIL_STATE_NODE_H