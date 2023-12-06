#include "foil_state/foil_state_node.hpp"

FoilStateNode::FoilStateNode() : Node("foil_state_node")
{
  init_parameters();
  init_interfaces();

  timer_ = this->create_wall_timer(
          loop_dt_, std::bind(&FoilStateNode::timer_callback, this));
}

FoilStateNode::~FoilStateNode()
{
}

void FoilStateNode::init_parameters()
{
}

void FoilStateNode::init_interfaces()
{
  subscription_utm_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("utm_pose", 10, std::bind(&FoilStateNode::utm_pose_callback, this, std::placeholders::_1));
  subscription_sbg_ekf_euler_ = this->create_subscription<sbg_driver::msg::SbgEkfEuler>("ekf_euler", 10, std::bind(&FoilStateNode::sbg_ekf_euler_callback, this, std::placeholders::_1));
  subscription_sbg_gps_vel_ = this->create_subscription<sbg_driver::msg::SbgGpsVel>("gps_vel", 10, std::bind(&FoilStateNode::sbg_gps_vel_callback, this, std::placeholders::_1));
  subscription_sbg_gps_hdt_ = this->create_subscription<sbg_driver::msg::SbgGpsHdt>("gps_hdt", 10, std::bind(&FoilStateNode::sbg_gps_hdt_callback, this, std::placeholders::_1));
  publisher_foil_state_ = this->create_publisher<foil_state_msg::msg::FoilState>("foil_state", 10);
}

void FoilStateNode::timer_callback()
{
  auto msg = foil_state_msg::msg::FoilState();
  msg.header.stamp = this->now();
  msg.header.frame_id = "world";

  msg.pose.pose.position.x = this->x_;
  msg.pose.pose.position.y = this->y_;
  msg.pose.pose.position.z = this->z_;

  msg.pose.pose.orientation.x = this->roll_;
  msg.pose.pose.orientation.y = this->pitch_;
  msg.pose.pose.orientation.z = this->yaw_;

  msg.speed.x = this->speed_x_;
  msg.speed.y = this->speed_y_;
  msg.speed.z = this->speed_z_;

  publisher_foil_state_->publish(msg);
}

void FoilStateNode::utm_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  this->x_ = msg->pose.position.x;
  this->y_ = msg->pose.position.y;
}

void FoilStateNode::sbg_ekf_euler_callback(const sbg_driver::msg::SbgEkfEuler::SharedPtr msg)
{
  this->roll_ = msg->angle.x;
  this->pitch_ = msg->angle.y;
}

void FoilStateNode::sbg_gps_vel_callback(const sbg_driver::msg::SbgGpsVel::SharedPtr msg)
{
  this->speed_x_ = msg->velocity.x;
  this->speed_y_ = msg->velocity.y;
  this->speed_z_ = msg->velocity.z;
}

void FoilStateNode::sbg_gps_hdt_callback(const sbg_driver::msg::SbgGpsHdt::SharedPtr msg)
{
  this->yaw_ = msg->true_heading;
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FoilStateNode>());
  rclcpp::shutdown();

  printf("hello world foil_state package\n");
  return 0;
}
