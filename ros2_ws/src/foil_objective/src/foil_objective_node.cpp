#include "foil_objective/foil_objective_node.hpp"

FoilObjectiveNode::FoilObjectiveNode() : Node("foil_objective_node") {
    init_parameters();
    init_interfaces();

    this->C = proj_context_create();
    this->P = proj_create_crs_to_crs(C, "EPSG:4326", "+proj=utm +zone=30 +datum=WGS84", NULL);

    this->norm = proj_normalize_for_visualization(C, P);
    proj_destroy(this->P);
    this->P = this->norm;

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
    subscription_foil_state_ = this->create_subscription<custom_msg::msg::FoilState>("foil_state", 10, std::bind(&FoilObjectiveNode::foil_state_callback, this, std::placeholders::_1));
    publisher_foil_objective_ = this->create_publisher<custom_msg::msg::FoilObjective>("foil_objective", 10);
}

void FoilObjectiveNode::timer_callback()
{
    auto msg = custom_msg::msg::FoilObjective();

    a = proj_coord(this->lat_, this->lon_, 0, 0);
    b = proj_trans(this->P, PJ_FWD, a);


    msg.pose.header.stamp = this->now();
    msg.pose.header.frame_id = "world";

    msg.pose.pose.position.x = x_objective_;
    msg.pose.pose.position.y = y_objective_;
    msg.pose.pose.position.z = z_objective_;

    find_theta_objective(p1_, p2_);
    msg.pose.pose.orientation.x = roll_objective_;
    msg.pose.pose.orientation.y = pitch_objective_;
    msg.pose.pose.orientation.z = yaw_objective_;


    end_objective(p1_, p2_);
    msg.objective = objective_;

    publisher_foil_objective_->publish(msg);
}

void FoilObjectiveNode::foil_objective_position_callback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    x_objective_ = msg->x;
    y_objective_ = msg->y;
    z_objective_ = msg->z;
}

void FoilObjectiveNode::foil_state_callback(const custom_msg::msg::FoilState::SharedPtr msg)
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
}

void FoilObjectiveNode::end_objective(double *p1, double *p2){
    double p3[2];
    p3[0] = p2[0] - (p2[1] - p1[1]);
    p3[1] = p2[1] + (p2[0] - p1[0]);

    double dist = (p3[0] - p2[0]) * (p2[1] - y_) - (p2[0] - x_) * (p3[1] - p2[1]);
    if (dist > 0){
        objective_ = true;
    } else {
        objective_ = false;
    }
}

void FoilObjectiveNode::find_theta_objective(double *p1, double *p2)
{   

    
    double dist = (p2[0] - p1[0]) * (y_ - p1[1]) - (x_ - p1[0]) * (p2[1] - p1[1]);
    double heading = atan2((p2[1] - p1[1]), (p2[0] - p1[0]));

    //Warning : si le comportement est pas dans le bon sens, d'abord penser à tester avec un +tanh
    yaw_objective_ = heading - tanh(0.0002*dist)*M_PI/4;
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