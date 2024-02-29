import serial
import os
import struct
from time import time, sleep
class ReceivedNucData:
	def __init__(self, sensor_data1, sensor_data2, sensor_data3, sensor_data4):
		self.height_left = sensor_data1
		self.height_right = sensor_data2
		self.height_rear = sensor_data3
		self.height_potar = sensor_data4
		self.height_est = 0.5*(sensor_data1 + sensor_data2)

while not os.path.exists("/dev/ttyUSB0"):
	print("Le périphérique /dev/ttyESP n'est pas connecté ou n'est pas reconnue.")
	sleep(0.5)

try:
	serial_port = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)
	sleep(1)

	if serial_port.is_open:
		print("Liaison série ouverte.")
	else:
		print("Erreur lors de l'ouverture de la liaison série.")

except IOError:
	serial_port.close()
	serial_port.open()
	print("Port série refermé puis rouvert")
sensor_data = ReceivedNucData(0.0, 0.0, 0.0, 0.0)
t0 = time()
try :
	while True :
		float_map = "ffff"
		data_size = struct.calcsize(float_map)
		raw_data = serial_port.read(data_size)

		if len(raw_data) == data_size:
			# Déballer les données dans la structure SensorData
			sensor_data = ReceivedNucData(*struct.unpack(float_map, raw_data))
			print(f"Données du capteur reçues: {sensor_data.__dict__}")

			print(sensor_data)
		else:
			print("Erreur de lecture des données série.")
			print(None)
		
		while time()-t0 < 0.05 :
			pass
		t0 = time()
except KeyboardInterrupt:
	serial_port.close()
	exit()
        
