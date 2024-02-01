import rclpy
from rclpy.node import Node

#from senix_interface import Senix
from std_msgs.msg import Int32

from pymodbus.client import ModbusSerialClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder


class Senix:
    def __init__(self, port="/dev/ttyAcous", baudrate=9600):   
        print(f"Connecting to {port}...")
        self.client = ModbusSerialClient(method="rtu", port=port, baudrate=baudrate)
        print(f"Connected : ID = {self.client.read_holding_registers(0x320, 1, slave=1)}")
        self.dict = {
            "raw_counts": 0x0220,
            "filter_counts": 0x0221,
            "distance": 0x0222,
            "temperature": 0x0223,
            "user_target_width": 0x0224,
            "reference_target_width": 0x0225,
            "reference_target_distance": 0x0226,
            "scale_top": 0x0227,
            "scale_bottom": 0x0228
        }
        self.__coeff = 0.085954

    def get_value(self,reg):
        result = self.client.read_holding_registers(reg, 1, slave=1)
        decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.BIG, wordorder=Endian.BIG)
        return decoder.decode_16bit_int()

    def distance(self):
        """distance in millimeters"""
        return self.get_value(self.dict["distance"])

    def temperature(self):
        """temperature in °C (scale factor 10)"""
        return self.get_value(self.dict["temperature"])

    def raw_counts(self):
        return self.get_value(self.dict["raw_counts"])/self.__coeff

    def filt_counts(self):
        return self.get_value(self.dict["filter_counts"])/self.__coeff

    def target_width(self):
        return self.get_value(self.dict["user_target_width"])/self.__coeff

    def ref_target_width(self):
        return self.get_value(self.dict["reference_target_width"])/self.__coeff

    def ref_target_distance(self):
        return self.get_value(self.dict["reference_target_distance"])/self.__coeff

    def scale_top(self):
        return self.get_value(self.dict["scale_top"])

    def scale_bottom(self):
        return self.get_value(self.dict["scale_bottom"])

class senixNode(Node):
    def __init__(self):
        super().__init__('senixNode')
        self.publisher_int = self.create_publisher(Int32, 'senix/altitude', 1)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.my_senix = Senix(port = "/dev/ttyAcous")

    def timer_callback(self):
        dist = self.my_senix.distance()
        if dist is not None :
            msg = Int32()
            msg.data = dist
            if dist !=0 :
                self.publisher_int.publish(msg)
                self.get_logger().info(f"Senix sent altitude : {msg.data}")

def main():
    rclpy.init()
    node_senix = senixNode()
    rclpy.spin(node_senix)
    node_senix.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
