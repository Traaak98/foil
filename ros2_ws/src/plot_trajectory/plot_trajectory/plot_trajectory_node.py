import rclpy
from custom_msg.msg import FoilState
from rclpy.node import Node
import matplotlib.pyplot as plt
from math import cos, sin

class Plot(Node):

    def __init__(self):
        super().__init__("plot")
        self.x = 0
        self.y = 0
        self.psi = 0
        self.fig, self.ax = plt.subplots()
        self.init_interface()

    def init_interface(self):
        Xlim = [7146050, 7146750]
        Ylim = [-534250, -533800]
        scalex = 3000
        scaley= 1000
        offsetx = 0
        offsety = -500
        Xlim = [Xlim[0]-scalex+offsetx, Xlim[1]+scalex+offsetx]
        Ylim = [Ylim[0]-scaley+offsety, Ylim[1]+scaley+offsety]
        pointA = [7146100, -534100]
        pointB = [7146300, -533900]
        pointC = [7146600, -533900]
        self.ax.scatter(pointA[0], pointA[1], color="red", marker="o", label="Point A", s=100)
        self.ax.scatter(pointB[0], pointB[1], color="red", marker="o", label="Point B")
        self.ax.scatter(pointC[0], pointC[1], color="red", marker="o", label="Point C")
        self.ax.set_xlim(7145550, 7147250)
        self.ax.set_ylim(Ylim[0], Ylim[1])

        # Souscrire au topic donnant les consignes d'angle des servomoteurs
        self.foil_state_subscription = self.create_subscription(
            FoilState, "/foil_state", self.plot_callback, 10
        )

        self.get_logger().info("Subscribeur et publisher initialisé.")

        # Créer un minuteur pour appeler la fonction time_callback toutes les 2 secondes
        self.timer = self.create_timer(0.05, self.time_callback)

    def plot_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.psi = msg.pose.pose.orientation.z

    def time_callback(self):
        self.ax.scatter(self.x, self.y, color="blue", marker="x")
        arrow = self.ax.arrow(self.x, self.y, 100 * cos(self.psi), 100 * sin(self.psi))
        self.get_logger().info("PLotting...")
        #plt.show()
        plt.pause(0.5)
        arrow.remove()
        self.get_logger().info("PLotting good")


    def destroy_node(self):
        # Fermer la liaison série lors de la fermeture du node
        self.get_logger().info("Fermeture de la liaison série...")
        if self.serial_port.is_open:
            self.serial_port.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    plot = Plot()
    rclpy.spin(plot)
    plot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()