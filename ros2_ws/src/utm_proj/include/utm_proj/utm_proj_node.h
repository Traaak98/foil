#ifndef BUILD_UTM_PROJ_NODE_H
#define BUILD_UTM_PROJ_NODE_H

#include <proj.h>       // PROJ library
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sbg_driver/msg/sbg_gps_pos.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;
using namespace std;

class UtmProjNode : public rclcpp::Node {
public:
  UtmProjNode();

  ~UtmProjNode();

private:
  /// Variables
  double lat_ = 0.0;
  double lon_ = 0.0;

  PJ_CONTEXT *C;
  PJ *P;
  PJ *norm;
  PJ_COORD a, b;

  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::milliseconds loop_dt_ = 100ms; // loop dt

  /// Topics / Services
  rclcpp::Subscription<sbg_driver::msg::SbgGpsPos>::SharedPtr subscription_gps_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose_;

  /**
   *  Init and get parameters of the Node
   */
  void init_parameters();

  /**
   * Init topics to this node (publishers & subscribers)
   */
  void init_interfaces();

  /**
   * Callback of the timer
   */
  void timer_callback();

  /**
   * Callback of the subscription
   */
  void gps_callback(const sbg_driver::msg::SbgGpsPos::SharedPtr msg);
};

#endif //BUILD_UTM_PROJ_NODE_H