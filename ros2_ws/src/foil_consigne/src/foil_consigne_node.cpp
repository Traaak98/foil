#include "foil_consigne/foil_consigne_node.hpp"

FoilConsigneNode::FoilConsigneNode() : Node("foil_consigne_node")
{
    init_parameters();
    init_interfaces();

    timer_ = this->create_wall_timer(
        loop_dt_, std::bind(&FoilConsigneNode::timer_callback, this));
}

FoilConsigneNode::~FoilConsigneNode()
{
}

void FoilConsigneNode::init_parameters()
{
}

void FoilConsigneNode::init_interfaces()
{
    subscription_foil_state_ = this->create_subscription<foil_state_msg::msg::FoilState>("foil_state", 10, std::bind(&FoilConsigneNode::foil_state_callback, this, std::placeholders::_1));
    subscription_foil_objective_ = this->create_subscription<foil_objective_msg::msg::FoilObjective>("foil_objective", 10, std::bind(&FoilConsigneNode::foil_objective_callback, this, std::placeholders::_1));
    // publisher_foil_consigne_ = this->create_publisher<uart_msg::msg::Uart>("foil_consigne", 10);
}

void FoilConsigneNode::timer_callback()
{
    // TODO : auto msg = uart_msg::msg::Uart();
    // TODO : Find relation between speed and thruster command

    float speed = 0.0;
    
    float alpha1_left_aileron = 0.0;
    float alpha2_right_aileron = 0.0;
    float beta_foil = 0.0;
    float theta_rudder = atan2(y_objective_ - y_, x_objective_ - x_);

    // TODO : Add to message

    // publisher_foil_consigne_->publish(msg);

}

void FoilConsigneNode::foil_state_callback(const foil_state_msg::msg::FoilState::SharedPtr msg)
{
    this->x_ = msg->pose.pose.position.x;
    this->y_ = msg->pose.pose.position.y;
    this->z_ = msg->pose.pose.position.z;

    this->roll_ = msg->pose.pose.orientation.x;
    this->pitch_ = msg->pose.pose.orientation.y;
    this->yaw_ = msg->pose.pose.orientation.z;

    this->speed_x_ = msg->speed.x;
    this->speed_y_ = msg->speed.y;
    this->speed_z_ = msg->speed.z;
}

void FoilConsigneNode::foil_objective_callback(const foil_objective_msg::msg::FoilObjective::SharedPtr msg)
{
    this->x_objective_ = msg->pose.pose.position.x;
    this->y_objective_ = msg->pose.pose.position.y;
    this->z_objective_ = msg->pose.pose.position.z;

    this->roll_objective_ = msg->pose.pose.orientation.x;
    this->pitch_objective_ = msg->pose.pose.orientation.y;

    this->speed_objective_ = msg->speed;
}

int main(int argc, char * argv[])
{
    (void) argc;
    (void) argv;

    rclcpp::init(argc, argv);
    printf("hello world foil_consigne package\n");
    rclcpp::spin(std::make_shared<FoilConsigneNode>());
    rclcpp::shutdown();

    printf("Foil objective node stopped\n");
    return 0;
}