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
    this->declare_parameter<double>("kpitch_", 5.3); //* (180/PI)/10 at the beginning of the project
    this->declare_parameter<double>("kspeed_", 0.2*PI); //TODO: SET THE RIGHT VALUE
    this->declare_parameter<double>("kroll_", (180/PI)/20); //* (180/PI)/20 at the beginning of the project
}

void FoilConsigneNode::init_interfaces()
{
    subscription_foil_state_ = this->create_subscription<custom_msg::msg::FoilState>("foil_state", 10, std::bind(&FoilConsigneNode::foil_state_callback, this, std::placeholders::_1));
    subscription_foil_objective_ = this->create_subscription<custom_msg::msg::FoilObjective>("foil_objective", 10, std::bind(&FoilConsigneNode::foil_objective_callback, this, std::placeholders::_1));
    publisher_foil_consigne_ = this->create_publisher<custom_msg::msg::FoilConsigne>("foil_consigne", 10);
    publisher_parametres_consigne_ = this->create_publisher<custom_msg::msg::ParamConsigne>("parametres_consigne", 10);
}

void FoilConsigneNode::foil_state_callback(const custom_msg::msg::FoilState::SharedPtr msg)
{
    this->x_ = msg->pose.pose.position.x;
    this->y_ = msg->pose.pose.position.y;
    this->z_ = msg->pose.pose.position.z;

    this->roll_ = msg->pose.pose.orientation.x;
    this->pitch_ = msg->pose.pose.orientation.y;
    this->yaw_ = msg->pose.pose.orientation.z;

    this->speed_x_ = msg->vector_speed.x;
    this->speed_y_ = msg->vector_speed.y;
    this->speed_z_ = msg->vector_speed.z;

    this->speed_ = msg->speed;
    this->height_est_ = msg->height_est;
}

void FoilConsigneNode::foil_objective_callback(const custom_msg::msg::FoilObjective::SharedPtr msg)
{
    this->x_objective_ = msg->pose.pose.position.x;
    this->y_objective_ = msg->pose.pose.position.y;
    this->z_objective_ = msg->pose.pose.position.z;

    this->roll_objective_ = msg->pose.pose.orientation.x;
    this->pitch_objective_ = msg->pose.pose.orientation.y;
    this->yaw_objective_ = msg->pose.pose.orientation.z;

    this->speed_objective_ = msg->speed;
}

void FoilConsigneNode::timer_callback() 
{
    auto msg_consigne = custom_msg::msg::FoilConsigne();

    read_parameters();

    regulation_pitch();

    regulation_speed();

    regulation_roll();

    // Send data to the publisher
    msg_consigne.servo_foil = foil_regulation;
    msg_consigne.thruster = speed_regulation;
    msg_consigne.servo_aileron_left = roll_regulation;
    msg_consigne.servo_aileron_right = roll_regulation;

    publisher_foil_consigne_->publish(msg_consigne);
}

void FoilConsigneNode::read_parameters()
{
    auto msg_parameters = custom_msg::msg::ParamConsigne();

    kpitch_ = this->get_parameter("kpitch_").as_double();
    kspeed_ = this->get_parameter("kspeed_").as_double();
    kroll_ = this->get_parameter("kroll_").as_double();

    msg_parameters.kpitch = kpitch_;
    msg_parameters.kspeed = kspeed_;
    msg_parameters.kroll = kroll_;

    publisher_parametres_consigne_->publish(msg_parameters);
}

void FoilConsigneNode::regulation_pitch() 
{
    foil_regulation = kpitch_ * (pitch_objective_ - pitch_);
    if (foil_regulation > 1.){
        foil_regulation = 1.;
    } else if (foil_regulation < -1.){
        foil_regulation = -1.;
    }
    foil_regulation = 100*foil_regulation;
}

void FoilConsigneNode::regulation_speed()
{
    speed_regulation = tanh(kspeed_ * (z_objective_-z_));
    if (speed_regulation >= 0.) {
        speed_regulation = SPEED_OFFSET + (1-0.8)*speed_regulation; //* 0.8 -> plage d'accélération entre 80 et 100%
    } else {
        speed_regulation = SPEED_OFFSET + (1-0.9)*speed_regulation; //* 0.9 -> plage de freinage entre 80 et 70%
    }
    speed_regulation = 100*speed_regulation;
}

void FoilConsigneNode::regulation_roll()
{
    roll_regulation = kroll_ * (roll_objective_ - roll_);
    if (roll_regulation > 1.){
        roll_regulation = 1.;
    } else if (roll_regulation < -1.){
        roll_regulation = -1.;
    }
    roll_regulation = 100*roll_regulation;
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