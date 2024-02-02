import struct
import time

import rclpy
import serial
from foil_consigne_msg.msg import FoilConsigne
from rclpy.node import Node


# Définir la structure pour stocker les données envoyées
class CommandData:
    def __init__(self, command1, command2, command3, command4, command5):
        self.command1 = command1
        self.command2 = command2
        self.command3 = command3
        self.command4 = command4
        self.command5 = command5


# Définir la structure pour stocker les données reçues
class SensorData:
    def __init__(
        self, sensor_data1, sensor_data2, sensor_data3, sensor_data4, sensor_data5
    ):
        self.sensor_data1 = sensor_data1
        self.sensor_data2 = sensor_data2
        self.sensor_data3 = sensor_data3
        self.sensor_data4 = sensor_data4
        self.sensor_data5 = sensor_data5


class Uart(Node):
    def __init__(self):
        super().__init__("uart")
        self.init_interface()

    def init_interface(self):
        # Souscrire au topic donnant les consignes d'angle des servomoteurs
        self.servo_angles_subscription = self.create_subscription(
            FoilConsigne, "/foil_consigne", self.servo_angles_callback, 10
        )

        # Créer un publisher pour publier les données du capteur
        # self.sensor_data_publisher = self.create_publisher(SensorData, '/sensors_data', 10)

        self.get_logger().info("Subscribeur et publisher initialisé.")
        self.get_logger().info("Ouverture de la liaison série.")

        # Configuration de la communication série
        self.serial_port = serial.Serial("/dev/ttyArduino", 115200, timeout=1)

        time.sleep(1)
        # Vérifier si la liaison série est ouverte
        if self.serial_port.is_open:
            self.get_logger().info("Liaison série ouverte.")
        else:
            self.get_logger().info("Erreur lors de l'ouverture de la liaison série.")

        # Créer un minuteur pour appeler la fonction time_callback toutes les 2 secondes
        self.timer = self.create_timer(2.0, self.time_callback)

    def servo_angles_callback(self, msg):
        # Lecture des consignes d'angle des servomoteurs et de la commande du thruster
        self.get_logger().info(
            "Angles des servomoteurs - servoFoil: %f, servoGouvernail: %f, servoAileronLeft: %f, servoAileronRight: %f, Thruster: %f",
            msg.servo_foil,
            msg.servo_gouvernail,
            msg.servo_aileron_left,
            msg.servo_aileron_right,
            msg.thruster,
        )

    def time_callback(self):
        # Code à exécuter à intervalles réguliers
        self.get_logger().info(
            "Fonction time_callback appelée à intervalles réguliers."
        )

        # Assurez-vous d'utiliser les attributs corrects de msg
        msg = FoilConsigne()

        # Envoyer des données via UART
        self.send_uart_data(msg)

        # Recevoir des données via UART
        received_data = self.receive_sensor_data()
        if received_data:
            self.get_logger().info(
                f"Données du capteur reçues: {received_data.__dict__}"
            )

            # Publier les données du capteur
            # self.sensor_data_publisher.publish(received_data)
        else:
            self.get_logger().warn("Erreur lors de la lecture des données du capteur.")

    def send_uart_data(self, msg):
        # command_to_send = CommandData(msg.servo_foil, msg.servo_gouvernail, msg.servo_aileron_left, msg.servo_aileron_right, msg.thruster)
        command_to_send = CommandData(-80, 80, 40, 0.0, 0.0)

        # Emballer les données dans une chaîne binaire
        data_to_send = struct.pack(
            "fffff",
            command_to_send.command1,
            command_to_send.command2,
            command_to_send.command3,
            command_to_send.command4,
            command_to_send.command5,
        )

        # Envoyer les données via la liaison UART
        self.serial_port.write(data_to_send)

        # Attendre les 5 flottants
        while self.serial_port.in_waiting < struct.calcsize("5f"):
            time.sleep(0.01)

        # Afficher les données envoyées
        self.get_logger().info(
            f"Données de commandes envoyées: {command_to_send.__dict__}"
        )

    def receive_sensor_data(self):
        # Lire la taille des données (sizeof(SensorData))
        data_size = struct.calcsize("fffff")  # Utiliser 'fffff' pour 5 flottants

        # Lire les données depuis le port série
        raw_data = self.serial_port.read(data_size)

        # Vérifier si suffisamment de données ont été lues
        if len(raw_data) == data_size:
            # Déballer les données dans la structure SensorData
            sensor_data = SensorData(*struct.unpack("fffff", raw_data))
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
