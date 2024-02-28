import struct
import time
import os

import rclpy
import serial
from foil_height_sensor_message.msg import FoilHeight
from rclpy.node import Node


class ReceivedNucData:
    def __init__(self, sensor_data1, sensor_data2, sensor_data3, sensor_data4):
        self.height_left = sensor_data1
        self.height_right = sensor_data2
        self.height_rear = sensor_data3
        self.height_potar = sensor_data4
        self.height_est = 0.5*(sensor_data1 + sensor_data2)


class EspUart(Node):

    def __init__(self):
        super().__init__("esp_nuc")
        self.init_interface()
        self.commands_received = ReceivedNucData(0.0, 0.0, 0.0, 0.0)


    def init_interface(self):    
        # Créer un publisher pour publier les données du capteur
        self.sensor_data_publisher = self.create_publisher(FoilHeight, "/esp_data", 10)

        self.get_logger().info("Publisher initialisé.")
        self.get_logger().info("Ouverture de la liaison série.")

        while not os.path.exists("/dev/ttyUSB0"):
            self.get_logger().info("Le périphérique /dev/ttyESP n'est pas connecté ou n'est pas reconnue.")
            time.sleep(0.5)

        # Configuration de la communication série
        try :
            self.serial_port = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)

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

        # Recevoir des données via UART
        self.commands_received = self.receive_sensor_data()
        if self.commands_received is not None:
            msg.height_left = self.commands_received.height_left
            msg.height_right = self.commands_received.height_right
            msg.height_rear = self.commands_received.height_rear
            msg.height_potar = self.commands_received.height_potar
            #/!\ Estimation à 2 capteurs - pas fini
            msg.height_est = 0.5*(msg.height_left + msg.height_right)



            self.get_logger().info(f"Données publiées - Height left : {msg.height_left:.3f}, \
                                                      \tHeight right: {msg.height_right:.3f}, \
                                                      \tHeight rear: {msg.height_rear:.3f}, \
                                                      \tHeight potar: {msg.height_potar:.3f}, \
                                                      \tHeight est : {msg.height_est:.3f}")
            self.sensor_data_publisher.publish(msg)
        
        else:
            self.get_logger().warn("Erreur lors de la lecture des données du capteur.")


    def receive_sensor_data(self):
        # Lire la taille des données (sizeof(SensorData))
        float_map = "ffff" # Utiliser 4'f' pour 4 flottants
        data_size = struct.calcsize(float_map)

        # Lire les données depuis le port série
        raw_data = self.serial_port.read(data_size)

        # Vérifier si suffisamment de données ont été lues
        if len(raw_data) == data_size:
            # Déballer les données dans la structure SensorData
            sensor_data = ReceivedNucData(*struct.unpack(float_map, raw_data))
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
    esp_nuc = EspUart()
    rclpy.spin(esp_nuc)
    esp_nuc.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
