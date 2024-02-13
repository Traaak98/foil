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
    Model_inv_ = Model_.inverse();
    this->declare_parameter<double>("kz_", 0.5);
    this->declare_parameter<double>("kroll_", 1.0);
    this->declare_parameter<double>("kpitch_", 5.3); // (180/3.1415)/10
    this->declare_parameter<double>("kz_proportional_", 0.5);
    this->declare_parameter<double>("kroll_proportional_", 0.5);
    this->declare_parameter<double>("kpitch_proportional_", 0.5);
    this->declare_parameter<double>("kyaw_proportional_", 0.5);
    //TODO : kspeed_ 
}

void FoilConsigneNode::init_interfaces()
{
    subscription_foil_state_ = this->create_subscription<custom_msg::msg::FoilState>("foil_state", 10, std::bind(&FoilConsigneNode::foil_state_callback, this, std::placeholders::_1));
    subscription_foil_objective_ = this->create_subscription<custom_msg::msg::FoilObjective>("foil_objective", 10, std::bind(&FoilConsigneNode::foil_objective_callback, this, std::placeholders::_1));
    publisher_foil_consigne_ = this->create_publisher<custom_msg::msg::FoilConsigne>("foil_consigne", 10);
    publisher_forces_actionneurs_ = this->create_publisher<geometry_msgs::msg::Point>("forces_actionneurs", 10);
    publisher_forces_angles_ = this->create_publisher<geometry_msgs::msg::Point>("forces_angles", 10);
    publisher_parametres_consigne_ = this->create_publisher<custom_msg::msg::ParamConsigne>("parametres_consigne", 10);

}

void FoilConsigneNode::timer_callback()
{
    auto msg = custom_msg::msg::FoilConsigne();
    auto msg_angles = geometry_msgs::msg::Point();
    auto msg_actionneurs = geometry_msgs::msg::Point();
    auto msg_parametres = custom_msg::msg::ParamConsigne();

    // double kz_ = 0.5; // TODO: set this parameter
    // double kroll_ = 0.5; // TODO: set this parameter
    // double kpitch_ = 0.5; // TODO: set this parameter

    kz_ = this->get_parameter("kz_").as_double();
    kroll_ = this->get_parameter("kroll_").as_double();
    kpitch_ = this->get_parameter("kpitch_").as_double();

    msg_parametres.kz = kz_;
    msg_parametres.kroll = kroll_;
    msg_parametres.kpitch = kpitch_;

    double g = 9.81;

    // double z_desired = z_objective_*tanh(kz_*speed_);
    // z_desired = z_; // TODO: TEST PARAMETER. TO BE REMOVED
    // double z_diff = z_desired - z_;

    // double yaw_desired = atan2(y_objective_ - y_, x_objective_ - x_);
    // yaw_desired = 0.0; // TODO: TEST PARAMETER. TO BE REMOVED
    // double yaw_diff = yaw_desired - yaw_;

    // double roll_desired = atan(kroll_*yaw_diff*speed_/g);
    // roll_desired = 0.0; // TODO: TEST PARAMETER. TO BE REMOVED
    // double roll_diff = roll_desired - roll_;
    // double pitch_desired = pitch_objective_*tanh(kpitch_*z_diff);
    // pitch_desired = 0.0; // TODO: TEST PARAMETER. TO BE REMOVED
    // double pitch_diff = pitch_desired - pitch_;

    // double kz_proportional = 0.5; // TODO: set this parameter
    // double kroll_proportional = 0.5; // TODO: set this parameter
    // double kpitch_proportional = 0.5; // TODO: set this parameter

    kz_proportional_ = this->get_parameter("kz_proportional_").as_double();
    kroll_proportional_ = this->get_parameter("kroll_proportional_").as_double();
    kpitch_proportional_ = this->get_parameter("kpitch_proportional_").as_double();

    msg_parametres.kz_proportional = kz_proportional_;
    msg_parametres.kroll_proportional = kroll_proportional_;
    msg_parametres.kpitch_proportional = kpitch_proportional_;


    double force_u_desired = kz_proportional_*z_diff;
    double force_roll_desired = kroll_proportional_*roll_diff;
    double force_pitch_desired = kpitch_proportional_*pitch_diff;

    RCLCPP_INFO(this->get_logger(),
    "Force u desirée: %f, Force roll désirée: %f, Force pitch désirée: %f",
    force_u_desired,
    force_roll_desired,
    force_pitch_desired);

    msg_angles.x = force_u_desired;
    msg_angles.y = force_roll_desired;
    msg_angles.z = force_pitch_desired;

    double force_aileron_left = Model_inv_(0, 0)*force_u_desired + Model_inv_(0, 1)*force_roll_desired + Model_inv_(0, 2)*force_pitch_desired;
    double force_aileron_right = Model_inv_(1, 0)*force_u_desired + Model_inv_(1, 1)*force_roll_desired + Model_inv_(1, 2)*force_pitch_desired;
    double force_foil = Model_inv_(2, 0)*force_u_desired + Model_inv_(2, 1)*force_roll_desired + Model_inv_(2, 2)*force_pitch_desired;

    RCLCPP_INFO(this->get_logger(),
    "Force aileron left: %f, Force aileron right: %f, Force foil: %f",
    force_aileron_left,
    force_aileron_right,
    force_foil);

    msg_actionneurs.x = force_aileron_left;
    msg_actionneurs.y = force_aileron_right;
    msg_actionneurs.z = force_foil;

    // On intuite (on a aucune idée de ce que l'on fait mais tracasse, on a qu'un lidar a 4000 balles et une sbg a 2000)

    double alpha1_left_aileron = force_aileron_left;
    double alpha2_right_aileron = force_aileron_right;
    double beta_foil = force_foil;
    double theta_gouvernail = 0.0;


    // ########################################################################################### //
    // ########################################################################################### //
    //                                                                                             //
    //                           REGULATION PROPORTIONEL EN CAP                                    //
    //                                                                                             //
    // ########################################################################################### //
    // ########################################################################################### //

    // REGULATION CAP :
    //    if (yaw_objective_ < 0){
    //        yaw_objective_+=2*M_PI;
    //    }
    //    else{yaw_objective_+= M_PI;}
    // double yaw_error = sawtooth(yaw_objective_ - yaw_);
    
    double yaw_error = yaw_objective_ - yaw_;
    kyaw_proportional_ = this->get_parameter("kyaw_proportional_").as_double();
    if (yaw_error >= 0){
        theta_gouvernail += kyaw_proportional_*yaw_error;
    }
    else{
        theta_gouvernail -= kyaw_proportional_*yaw_error;
    }

    // ########################################################################################### //
    // ########################################################################################### //
    //                                                                                             //
    //                    REGULATION PROPORTIONEL DU FOIL ARRIERE                                  //
    //                                                                                             //
    // ########################################################################################### //
    // ########################################################################################### //

    pitch_desired = 0.0;
    pitch_diff = pitch_desired - pitch_;
    double beta_foil_regul = kpitch_ * pitch_diff;
    if (beta_foil_regul > 1.){
        beta_foil_regul = 1.;
    } else if (beta_foil_regul < -1.){
        beta_foil_regul = -1.;
    }

    RCLCPP_INFO(this->get_logger(), "Pitch diff: %f", pitch_diff);

    // ########################################################################################### //
    // ########################################################################################### //
    //                                                                                             //
    //                         REGULATION PROPORTIONEL EN ROLL                                     //
    //                                                                                             //
    // ########################################################################################### //
    // ########################################################################################### //

    // double roll_desired = 0.0;
    // double roll_diff = roll_desired - roll_;
    // double roll_foil_regul = kroll_ * roll_diff;
    // if (roll_foil_regul > 1.){
    //     roll_foil_regul = 1.;
    // } else if (roll_foil_regul < -1.){
    //     roll_foil_regul = -1.;
    // }

    // alpha1_left_aileron = roll_foil_regul;
    // alpha2_right_aileron = roll_foil_regul;

    // RCLCPP_INFO(this->get_logger(), "Roll diff: %f", roll_diff);

    // ########################################################################################### //
    // ########################################################################################### //
    //                                                                                             //
    //                         REGULATION PROPORTIONEL EN HAUTEUR                                  //
    //                                                                                             //
    // ########################################################################################### //
    // ########################################################################################### //

    double z_desired = 0.8; // hauteur désirée en mètres
    z_ = height_est_
    z_diff = z_desired - z_;
    
    // définir kz_ violent  
    double z_foil_regul = z_desired*tanh(kz_*z_diff);
    if (z_foil_regul > 1.){
        z_foil_regul = 1.;
    } else if (z_foil_regul < 0.){
        z_foil_regul = 0.;
    }
    RCLCPP_INFO(this->get_logger(), "Z diff: %f", z_diff);

    theta_gouvernail = 0.0; //TODO: Remove this parameter

    // Renvoyer un pourcentage d'angle entre -100 et 100 à la liaison série
    double beta_foil_extrema = 0.6; // TODO: set this parameter$
    double theta_gouvernail_extrema = 0.6; // TODO: set this parameter
    double alpha_aileron_extrema = 0.3; // TODO: set this parameter
    double speed_extrema = 1.0; // TODO: set this parameter

    // TODO: Regarder les angles max et min pour chacun des capteurs.

    // Passage en pourcentage
    beta_foil = beta_foil/(2*beta_foil_extrema);
    theta_gouvernail = theta_gouvernail/(2*theta_gouvernail_extrema);
    alpha1_left_aileron = alpha1_left_aileron/(2*alpha_aileron_extrema);
    alpha2_right_aileron = alpha2_right_aileron/(2*alpha_aileron_extrema);
    speed_ = speed_foil_regul/(speed_extrema);

    // TODO: SATURATION DES COMMANDES. NIQUEZ VOUS, ON FLINGUE PAS LES SERVOS CETTE FOIS.

    // Envoyer les données à la liaison série (UART)
    msg.servo_foil = 100*beta_foil_regul;
    msg.servo_gouvernail = 100*theta_gouvernail;
    msg.servo_aileron_left = 100*alpha1_left_aileron;
    msg.servo_aileron_right = -100*alpha2_right_aileron;
    msg.thruster = 100*speed_;

    RCLCPP_INFO(
    this->get_logger(), 
    "Foil consigne: servo_foil: %f, servo_gouvernail: %f, servo_aileron_left: %f, servo_aileron_right: %f, thruster: %f \n---------------------------------------------------------------\n",
    msg.servo_foil, 
    msg.servo_gouvernail, 
    msg.servo_aileron_left, 
    msg.servo_aileron_right, 
    msg.thruster);

    publisher_foil_consigne_->publish(msg);
    publisher_forces_angles_->publish(msg_angles);
    publisher_forces_actionneurs_->publish(msg_actionneurs);
    publisher_parametres_consigne_->publish(msg_parametres);
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