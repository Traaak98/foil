#include "foil_objective/foil_objective_node.hpp"

FoilObjectiveNode::FoilObjectiveNode() : Node("foil_objective_node") {
    init_parameters();
    init_interfaces();

    timer_ = this->create_wall_timer(
        loop_dt_, std::bind(&FoilObjectiveNode::timer_callback, this));
}

FoilObjectiveNode::~FoilObjectiveNode()
{
}

void FoilObjectiveNode::init_parameters()
{
}

void FoilObjectiveNode::init_interfaces()
{
    subscription_foil_objective_position_ = this->create_subscription<geometry_msgs::msg::Point>("foil_objective_position", 10, std::bind(&FoilObjectiveNode::foil_objective_position_callback, this, std::placeholders::_1));
    subscription_foil_objective_speed_ = this->create_subscription<std_msgs::msg::Float32>("foil_objective_speed", 10, std::bind(&FoilObjectiveNode::foil_objective_speed_callback, this, std::placeholders::_1));
    publisher_foil_objective_ = this->create_publisher<foil_objective_msg::msg::FoilObjective>("foil_objective", 10);
}

void FoilObjectiveNode::timer_callback()
{
    auto msg = foil_objective_msg::msg::FoilObjective();
    msg.pose.header.stamp = this->now();
    msg.pose.header.frame_id = "world";

    msg.pose.pose.position.x = x_objective_;
    msg.pose.pose.position.y = y_objective_;
    msg.pose.pose.position.z = z_objective_;

    msg.pose.pose.orientation.x = roll_objective_;
    msg.pose.pose.orientation.y = pitch_objective_;

    msg.speed = speed_objective_;

    publisher_foil_objective_->publish(msg);
}

void FoilObjectiveNode::foil_objective_position_callback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    x_objective_ = msg->x;
    y_objective_ = msg->y;
    z_objective_ = msg->z;
}

void FoilObjectiveNode::foil_objective_speed_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    speed_objective_ = msg->data;
}

int main(int argc, char * argv[])
{
    (void) argc;
    (void) argv;

    rclcpp::init(argc, argv);
    printf("hello world foil_objective package\n");
    rclcpp::spin(std::make_shared<FoilObjectiveNode>());
    rclcpp::shutdown();

    printf("Foil objective node stopped\n");
    return 0;
}