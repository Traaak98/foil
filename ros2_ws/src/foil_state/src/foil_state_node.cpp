#include <cstdio>
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
  subscription_utm_pose_ = this->create_subscription<utm_proj::msg::PoseStamped>("utm_pose", 10, std::bind(&FoilStateNode::utm_pose_callback, this, std::placeholders::_1));
  subscription_sbg_ekf_euler_ = this->create_subscription<sbg_driver::msg::SbgEkfEuler>("ekf_euler", 10, std::bind(&FoilStateNode::sbg_ekf_euler_callback, this, std::placeholders::_1));
  subscription_sbg_gps_vel_ = this->create_subscription<sbg_driver::msg::SbgGpsVel>("gps_vel", 10, std::bind(&FoilStateNode::sbg_gps_vel_callback, this, std::placeholders::_1));
  subscription_sbg_gps_hdt_ = this->create_subscription<sbg_driver::msg::SbgGpsHdt>("gps_hdt", 10, std::bind(&FoilStateNode::sbg_gps_hdt_callback, this, std::placeholders::_1));
}

void FoilStateNode::timer_callback()
{
}

void FoilStateNode::utm_pose_callback(const utm_proj::msg::PoseStamped::SharedPtr msg)
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


