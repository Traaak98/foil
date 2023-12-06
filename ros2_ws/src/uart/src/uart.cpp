#include <rclcpp/rclcpp.hpp>
#include </msg/cmd.msg>
#include <iostream>
#include <string>
#include <unistd.h>
#include <math.h>


// Installer la librairie serial
// sudo apt-get install libserial-dev

#include <SerialStream.h>
#include <serial/serial.h>  // Inclure la bibliothèque serial

class Uart : public rclcpp::Node
{
public:
    Uart() : Node("uart")
    {
        // Souscrire au topic donnant les consignes d'angle des servomoteurs
        servo_angles_subscriber_ = this->create_subscription<votre_package::msg::ServoAngles>(
            "/planning/servo_angles", 10, std::bind(&Uart::servo_angles_callback, this, std::placeholders::_1));

        // Configuration de la communication série
        serial_port_.Open("/dev/ttyUSB0");  // Attention à remplacer par le port connecter au microcontrôleur ATmega32U4
        serial_port_.SetBaudRate(SerialStreamBuf::BAUD_9600);
        serial_port_.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
        serial_port_.SetNumOfStopBits(1);
        serial_port_.SetParity(SerialStreamBuf::PARITY_NONE);

        if (!serial_port_.good())
        {
            RCLCPP_ERROR(get_logger(), "Erreur lors de l'ouverture du port série");
        }
    }

private:
    void servo_angles_callback(const uart::msg::ServoAngles::SharedPtr msg)
    {
        // Lecture des consignes d'angle des servomoteurs et de la commande du thruster
        RCLCPP_INFO(get_logger(), "Angles des servomoteurs - servoFoil: %f, servoGouvernail: %f, servoAileronLeft: %f, servoAileronRight: %f, Thruster: %f",
                    msg->servoFoil, msg->servoGouvernail, msg->servoAileronLeft, msg->servoAileronRight, msg->thruster);

        // Mise à jour des angles des servomoteurs et de la commande du thruster
        angle_servoFoil = msg->servoFoil;
        angle_servoGouvernail = msg->servoGouvernail;
        angle_servoAileronLeft = msg->servoAileronLeft;
        angle_servoAileronRight = msg->servoAileronRight;
        thruster = msg->thruster;

        // Envoie des données via la liaison UART
        send_uart_data();
    }

    void send_uart_data()
    {
        // Construction de la chaîne de données à envoyer
        std::string uart_data = "{" + std::to_string(angle_servoFoil) + "," +
                                std::to_string(angle_servoGouvernail) + "," +
                                std::to_string(angle_servoAileronLeft) + "," +
                                std::to_string(angle_servoAileronRight) + "," +
                                std::to_string(thruster) + "}";


        // Convertion de la chaîne en tableau de caractères
        const char *data = uart_data.c_str();

        // Envoie des données via la liaison UART
        serial_port_ << data;
        serial_port_.flush();
    }

    rclcpp::Subscription<votre_package::msg::ServoAngles>::SharedPtr servo_angles_subscriber_;
    LibSerial::SerialStream serial_port_;

    double angle_servoFoil = 0.0;
    double angle_servoGouvernail = 0.0;
    double angle_servoAileronLeft = 0.0;
    double angle_servoAileronRight = 0.0;
    double thruster = 0.0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto uart = std::make_shared<Uart>();
    rclcpp::spin(uart);
    rclcpp::shutdown();
    return 0;
}
