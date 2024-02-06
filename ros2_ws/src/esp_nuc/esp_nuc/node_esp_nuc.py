import struct
import time
import os

import rclpy
import serial
from foil_height_sensor_message.msg import FoilHeight
from rclpy.node import Node


# # Définir la structure pour stocker les données envoyées
# class CommandData:
#     def __init__(self, command1, command2, command3, command4, command5):
#         self.command1 = command1
#         self.command2 = command2
#         self.command3 = command3
#         self.command4 = command4
#         self.command5 = command5


# Définir la structure pour stocker les données reçues
class ReceivedNucData:
    def __init__(
        self,
        sensor_data1,
        sensor_data2,
        sensor_data3,
        sensor_data4,

    ):
        self.height_left = sensor_data1
        self.height_right = sensor_data2
        self.height_rear = sensor_data3
        self.height_potar = sensor_data4


class EspUart(Node):

    def __init__(self):
        super().__init__("esp_nuc")
        self.init_interface()
        #self.commands_to_send = CommandData(0.0, 0.0, 0.0, 0.0, 0.0)
        self.commands_received = ReceivedNucData(0.0, 0.0, 0.0, 0.0)

    def init_interface(self):    
        # Souscrire au topic pouvant contenir des données à envoyer à l'esp32 
        # self.servo_angles_subscription = self.create_subscription(
        #     FoilCmd, "/foil_consigne", self.servo_angles_callback, 10
        # )

        # Créer un publisher pour publier les données du capteur
        self.sensor_data_publisher = self.create_publisher(
            FoilHeight, "/esp_data", 10
        )

        self.get_logger().info("Publisher initialisé.")
        self.get_logger().info("Ouverture de la liaison série.")

        while not os.path.exists("/dev/ttyESP"):
            self.get_logger().info("Le périphérique /dev/ttyESP n'est pas connecté ou n'est pas reconnue.")
            time.sleep(0.5)

        # Configuration de la communication série
        try :
            self.serial_port = serial.Serial("/dev/ttyESP", 9600, timeout=1)

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

    def time_callback(self):
        # Assurez-vous d'utiliser les attributs corrects de msg
        msg = FoilHeight()

        # # Envoyer des données via UART
        # self.send_uart_data(msg)

        # Recevoir des données via UART
        self.commands_received = self.receive_sensor_data()
        if self.commands_received is not None:
            msg.height_left = self.commands_received.height_left
            msg.height_right = self.commands_received.height_right
            msg.height_rear = self.commands_received.height_rear
            msg.height_potar = self.commands_received.height_potar

            self.get_logger().info(
            f"Données publiées - \
            Height left : {msg.height_left}, \
            Height right: {msg.height_right}, \
            Height rear: {msg.height_rear}, \
            Height potar: {msg.height_potar}, \
            ")

            # Publier les données du capteur
            self.sensor_data_publisher.publish(msg)
        else:
            self.get_logger().warn("Erreur lors de la lecture des données du capteur.")

    def receive_sensor_data(self):
        # Lire la taille des données (sizeof(SensorData))
        data_size = struct.calcsize("ffff")  # Utiliser 'fffff' pour 5 flottants

        # Lire les données depuis le port série
        raw_data = self.serial_port.read(data_size)

        # Vérifier si suffisamment de données ont été lues
        if len(raw_data) == data_size:
            # Déballer les données dans la structure SensorData
            sensor_data = ReceivedNucData(*struct.unpack("ffff", raw_data))
            self.get_logger().info(f"Données du capteur reçues: {sensor_data.__dict__}")

            return sensor_data
        else:
            self.get_logger().warn("Erreur de lecture des données série.")
            return None

    # def send_uart_data(self, msg):
    #     self.get_logger().info(
    #         f"Angles consignes des servomoteurs à l'envoie - \
    #         servoFoil: {self.commands_to_send.command1}, \
    #         servoGouvernail: {self.commands_to_send.command2}, \
    #         servoAileronLeft: {self.commands_to_send.command3}, \
    #         servoAileronRight: {self.commands_to_send.command4}, \
    #         Thruster: {self.commands_to_send.command5}"
    #         )
        
    #     # pour tester 

    #     # Emballer les données dans une chaîne binaire
    #     data_to_send = struct.pack(
    #         "fffff",
    #         self.commands_to_send.command1,
    #         self.commands_to_send.command2,
    #         self.commands_to_send.command3,
    #         self.commands_to_send.command4,
    #         self.commands_to_send.command5,
    #     )

    #     # Envoyer les données via la liaison UART
    #     self.serial_port.write(data_to_send)

    #     # Afficher les données envoyées
    #     self.get_logger().info(
    #         f"Données de commandes envoyées: {self.commands_to_send.__dict__}"
    #     )

    def destroy_node(self):
        # Fermer la liaison série lors de la fermeture du node
        self.get_logger().info("Fermeture de la liaison série...")
        if self.serial_port.is_open:
            self.serial_port.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    esp_nuc = EspUart()
    rclpy.spin(esp_nuc)
    esp_nuc.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
