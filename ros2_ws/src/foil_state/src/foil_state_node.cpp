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
  subscription_sbg_ekf_euler_ = this->create_subscription<sbg_driver::msg::SbgEkfEuler>("/sbg/ekf_euler", 10, std::bind(&FoilStateNode::sbg_ekf_euler_callback, this, std::placeholders::_1));
  subscription_sbg_gps_vel_ = this->create_subscription<sbg_driver::msg::SbgGpsVel>("/sbg/gps_vel", 10, std::bind(&FoilStateNode::sbg_gps_vel_callback, this, std::placeholders::_1));
  subscription_sbg_gps_hdt_ = this->create_subscription<sbg_driver::msg::SbgGpsHdt>("/sbg/gps_hdt", 10, std::bind(&FoilStateNode::sbg_gps_hdt_callback, this, std::placeholders::_1));
  subscription_utm_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>("utm_pose", 10, std::bind(&FoilStateNode::utm_pose_callback, this, std::placeholders::_1));
  subscription_foil_height_ = this->create_subscription<foil_height_sensor_message::msg::FoilHeight>("esp_data", 10, std::bind(&FoilStateNode::foil_height_callback, this, std::placeholders::_1));
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

  msg.height_est = this->height_est_;

  RCLCPP_INFO(this->get_logger(),
               "Publishing: \n pos_x = %f pos_y = %f pos_z = %f \n angle_x = %f angle_y = %f angle_z = %f \n speed_x = %f speed_y = %f speed_z = %f \n h_est = %f",
               msg.pose.pose.position.x,
               msg.pose.pose.position.y,
               msg.pose.pose.position.z,
               msg.pose.pose.orientation.x,
               msg.pose.pose.orientation.y,
               msg.pose.pose.orientation.z,
               msg.speed.x,
               msg.speed.y,
               msg.speed.z,
               msg.height_est);

  publisher_foil_state_->publish(msg);
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

void FoilStateNode::utm_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  this->x_ = msg->pose.position.x;
  this->y_ = msg->pose.position.y;
}

void FoilStateNode::foil_height_callback(const foil_height_sensor_message::msg::FoilHeight::SharedPtr msg)
{
  this->height_est_ = msg->height_est;
}

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  rclcpp::init(argc, argv);
  printf("hello world foil_state package\n");
  rclcpp::spin(std::make_shared<FoilStateNode>());
  rclcpp::shutdown();

  printf("hello world foil_state package\n");
  return 0;
}
