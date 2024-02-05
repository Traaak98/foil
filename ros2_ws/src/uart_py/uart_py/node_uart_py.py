import struct
import time
import os 

import rclpy
import serial
from foil_cmd_servo_msg.msg import FoilCmd
from foil_consigne_msg.msg import FoilConsigne
from rclpy.node import Node


# Définir la structure pour stocker les données envoyées
class CommandData:
    def __init__(
        self,
        command_servo_foil,
        command_servo_gouvernail,
        command_servo_aileron_left,
        command_servo_aileron_right,
        command_thruster,
    ):
        self.command_servo_foil = command_servo_foil
        self.command_servo_gouvernail = command_servo_gouvernail
        self.command_servo_aileron_left = command_servo_aileron_left
        self.command_servo_aileron_right = command_servo_aileron_right
        self.command_thruster = command_thruster


# Définir la structure pour stocker les données reçues
class ReceivedData:
    def __init__(
        self,
        servo_foil,
        servo_gouvernail,
        servo_aileron_left,
        servo_aileron_right,
        thruster,
        mode,
        emergency_stop,
    ):
        self.servo_foil = servo_foil
        self.servo_gouvernail = servo_gouvernail
        self.servo_aileron_left = servo_aileron_left
        self.servo_aileron_right = servo_aileron_right
        self.thruster = thruster
        self.mode = mode
        self.emergency_stop = emergency_stop


class Uart(Node):

    def __init__(self):
        super().__init__("uart")
        self.init_interface()
        self.commands_to_send = CommandData(0.0, 0.0, 0.0, 0.0, 0.0)
        self.commands_received = ReceivedData(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def init_interface(self):
        # Souscrire au topic donnant les consignes d'angle des servomoteurs
        self.servo_angles_subscription = self.create_subscription(
            FoilConsigne, "/foil_consigne", self.servo_angles_callback, 10
        )

        # Créer un publisher pour publier les données du capteur
        self.sensor_data_publisher = self.create_publisher(
            FoilCmd, "/controler_data", 10
        )

        self.get_logger().info("Subscribeur et publisher initialisé.")
        self.get_logger().info("Ouverture de la liaison série.")

        while not os.path.exists("/dev/ttyArduino"):
            self.get_logger().info("Le périphérique /dev/ttyArduino n'est pas connecté ou n'est pas reconnue.")
            time.sleep(0.5)

        # Configuration de la communication série
        try :
            self.serial_port = serial.Serial("/dev/ttyArduino", 115200, timeout=1)

            # Temps d'ouverture de la liaison
            time.sleep(1)
        
            # Vérifier si la liaison série est ouverte
            if self.serial_port.is_open:
                self.get_logger().info("Liaison série ouverte.")
            else:
                self.get_logger().info("Erreur lors de l'ouverture de la liaison série.")
        
        except IOError:
            self.serial_port.close()
            self.serial_port.open()
            self.get_logger().info("Port série refermé puis rouvert")

        # Créer un minuteur pour appeler la fonction time_callback toutes les 2 secondes
        self.timer = self.create_timer(0.05, self.time_callback)

    def servo_angles_callback(self, msg):
        # Lecture des consignes d'angle des servomoteurs et de la commande du thruster
        self.get_logger().info(
            f"Angles des servomoteurs - \
            servo_foil: {msg.servo_foil}, \
            servo_gouvernail: {msg.servo_gouvernail}, \
            servo_aileron_left: {msg.servo_aileron_left}, \
            servo_aileron_right: {msg.servo_aileron_right}, \
            thruster: {msg.thruster}"
        )

        self.commands_to_send.command_servo_foil = msg.servo_foil
        self.commands_to_send.command_servo_gouvernail = msg.servo_gouvernail
        self.commands_to_send.command_servo_aileron_left = msg.servo_aileron_left
        self.commands_to_send.command_servo_aileron_right = msg.servo_aileron_right
        self.commands_to_send.command_thruster = msg.thruster

    def time_callback(self):
        # Assurez-vous d'utiliser les attributs corrects de msg
        msg = FoilCmd()

        # Envoyer des données via UART
        self.send_uart_data(msg)

        # Recevoir des données via UART
        self.commands_received = self.receive_sensor_data()
        if self.commands_received is not None:
            msg.cmd_servo_foil = self.commands_received.servo_foil
            msg.cmd_servo_gouvernail = self.commands_received.servo_gouvernail
            msg.cmd_servo_aileron_left = self.commands_received.servo_aileron_left
            msg.cmd_servo_aileron_right = self.commands_received.servo_aileron_right
            msg.cmd_thruster = self.commands_received.thruster
            msg.mode = self.commands_received.mode
            msg.emmergency_stop = self.commands_received.emergency_stop

            # Publier les données du capteur
            self.sensor_data_publisher.publish(msg)
        else:
            self.get_logger().warn("Erreur lors de la lecture des données du capteur.")

    def send_uart_data(self, msg):
        self.get_logger().info(
            f"Angles consignes des servomoteurs à l'envoie - \
            servoFoil: {self.commands_to_send.command_servo_foil}, \
            servo_gouvernail: {self.commands_to_send.command_servo_gouvernail}, \
            servo_aileron_left: {self.commands_to_send.command_servo_aileron_left}, \
            servo_aileron_right: {self.commands_to_send.command_servo_aileron_right}, \
            Thruster: {self.commands_to_send.command_thruster}"
        )

        # pour tester

        # Emballer les données dans une chaîne binaire
        data_to_send = struct.pack(
            "fffff",
            self.commands_to_send.command_servo_foil,
            self.commands_to_send.command_servo_gouvernail,
            self.commands_to_send.command_servo_aileron_left,
            self.commands_to_send.command_servo_aileron_right,
            self.commands_to_send.command_thruster,
        )

        # Envoyer les données via la liaison UART
        self.serial_port.write(data_to_send)

        # Afficher les données envoyées
        self.get_logger().info(
            f"Données de commandes envoyées: {self.commands_to_send.__dict__}"
        )

    def receive_sensor_data(self):
        # Lire la taille des données (sizeof(SensorData))
        data_size = struct.calcsize("fffffff")  # Utiliser 'fffff' pour 5 flottants

        # Lire les données depuis le port série
        raw_data = self.serial_port.read(data_size)

        # Vérifier si suffisamment de données ont été lues
        if len(raw_data) == data_size:
            # Déballer les données dans la structure SensorData
            sensor_data = ReceivedData(*struct.unpack("fffffff", raw_data))
            self.get_logger().info(f"Données du capteur reçues: {sensor_data.__dict__}")

            return sensor_data
        else:
            self.get_logger().warn("Erreur de lecture des données série.")
            return None

    def destroy_node(self):
        # Fermer la liaison série lors de la fermeture du node
        self.get_logger().info("Fermeture de la liaison série...")
        if self.serial_port.is_open:
            self.serial_port.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    uart = Uart()
    rclpy.spin(uart)
    uart.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
