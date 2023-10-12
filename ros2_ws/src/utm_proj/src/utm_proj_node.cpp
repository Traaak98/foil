#include "utm_proj/utm_proj_node.h"

UtmProjNode::UtmProjNode() : Node("utm_proj_node")
{
  init_parameters();
  init_interfaces();

  this->C = proj_context_create();

  this->P = proj_create_crs_to_crs(C, "EPSG:4326", "+proj=utm +zone=30 +datum=WGS84", NULL);

  this->norm = proj_normalize_for_visualization(C, P);
  proj_destroy(this->P);
  this->P = this->norm;

  timer_ = this->create_wall_timer(
          loop_dt_, std::bind(&UtmProjNode::timer_callback, this));
}

UtmProjNode::~UtmProjNode()
{
}

void UtmProjNode::init_parameters()
{
}

void UtmProjNode::init_interfaces()
{
  subscription_gps_ = this->create_subscription<sbg_driver::msg::SbgGpsPos>("fix", 10, std::bind(&UtmProjNode::gps_callback, this, std::placeholders::_1));
  publisher_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("utm_pose", 10);
}

void UtmProjNode::timer_callback()
{
  a = proj_coord(this->lat_, this->lon_, 0, 0);
  b = proj_trans(this->P, PJ_FWD, a);

  RCLCPP_INFO(this->get_logger(),"easting: %.3f, northing: %.3f", b.enu.e, b.enu.n);

  geometry_msgs::msg::PoseStamped msg;

  msg.header.stamp = this->now();
  msg.header.frame_id = "utm";
  msg.pose.position.x = b.enu.e;
  msg.pose.position.y = b.enu.n;

  publisher_pose_->publish(msg);
}

void UtmProjNode::gps_callback(const sbg_driver::msg::SbgGpsPos::SharedPtr msg)
{
  this->lat_ = msg->latitude;
  this->lon_ = msg->longitude;
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UtmProjNode>());
  rclcpp::shutdown();

  printf("hello world utm_proj package\n");
  return 0;
}
