#ifndef BUILD_FOIL_STATE_NODE_H
#define BUILD_FOIL_STATE_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sbg_driver/msg/sbg_ekf_euler.hpp"
#include "sbg_driver/msg/sbg_gps_vel.hpp"
#include "sbg_driver/msg/sbg_gps_hdt.hpp"

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

        rclcpp::TimerBase::SharedPtr timer_;
        std::chrono::milliseconds loop_dt_ = 100ms; // loop dt

        rclcpp::Subscription<utm_proj::msg::PoseStamped>::SharedPtr subscription_utm_pose_;
        rclcpp::Subscription<sbg_driver::msg::SbgEkfEuler>::SharedPtr subscription_sbg_ekf_euler_;
        rclcpp::Subscription<sbg_driver::msg::SbgGpsVel>::SharedPtr subscription_sbg_gps_vel_;
        rclccp::Subscription<sbg_driver::msg::SbgGpsHdt>::SharedPtr subscription_sbg_gps_hdt_;

        void init_parameters();
        void init_interfaces();
}

#endif //BUILD_FOIL_STATE_NODE_H