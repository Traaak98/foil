from rosbags.highlevel import AnyReader
from rosbags.typesys import get_types_from_msg, register_types

from pathlib import Path

speed = 1

############ IMPORTING CUSTOM MESSAGES ############

msg_path_list = ['/home/vivi/Documents/ENSTAB/S5/6.1-Guerledan/HydroChal-ROS2/src/interfaces/msg/Date.msg',
                 '/home/vivi/Documents/ENSTAB/S5/6.1-Guerledan/HydroChal-ROS2/src/interfaces/msg/Time.msg',
                 '/home/vivi/Documents/ENSTAB/S5/6.1-Guerledan/HydroChal-ROS2/src/interfaces/msg/GPS.msg',
                 '/home/vivi/Documents/ENSTAB/S5/6.1-Guerledan/HydroChal-ROS2/src/interfaces/msg/HEADING.msg',
                 '/home/vivi/Documents/ENSTAB/S5/6.1-Guerledan/HydroChal-ROS2/src/interfaces/msg/WIND.msg',
                 '/home/vivi/Documents/ENSTAB/S5/6.1-Guerledan/HydroChal-ROS2/src/interfaces/msg/YPR.msg',]

def guess_msgtype(path: Path) -> str:
    """Guess message type name from path."""
    name = path.relative_to(path.parents[2]).with_suffix('')
    if 'msg' not in name.parts:
        name = name.parent / 'msg' / name.name
    return str(name)

add_types = {}
for pathstr in msg_path_list :
    msgpath = Path(pathstr)
    msgdef = msgpath.read_text(encoding='utf-8')
    add_types.update(get_types_from_msg(msgdef, guess_msgtype(msgpath)))

register_types(add_types)
###################################################

################# CONFIG WEBSITE MONITORING #################

import mysql.connector

db = mysql.connector.connect(
          host="localhost",
          user="monitor",
          password="",
          database="sailboat"
        )

sql = "INSERT INTO boat (ros_timestamp_t0,ros_timestamp, lat, lon, heading, speed, true_wind_direction) VALUES (%s,%s,%s,%s,%s,%s,%s)"

#############################################################

import time

rosbag_path = "rosbag_ensta_test_gps/rosbag2_2023_11_24-09_05_06" # tour du stade
# rosbag_path = "rosbag_S1/rosbag2_2023_10_12-10_13_26" # guerledan aller retour COOL !
# rosbag_path = "rosbag_S1/rosbag2_2023_10_12-07_46_53" # guerledan test batiment pas fou 
rosbag_path = "rosbag_S1/rosbag2_2023_10_12-12_00_11" # guerledan aller retour bizarre
# rosbag_path = "rosbag_S1/rosbag2_2023_10_13-08_01_06" # guerledan ça va dans tt les sens mais COOL
# rosbag_path = "rosbag_S1/rosbag2_2023_10_13-08_15_06" # guerledan COOL mais saut mesure

# create reader instance and open for reading

latitude = 0
longitude = 0
heading = 0
sog = 0
wd = 0

old = 0
db.cursor().execute("DELETE FROM boat;")

t0 = 0 
 
with AnyReader([Path(rosbag_path)]) as reader:
    connections = [x for x in reader.connections if x.topic in ['/GPS','/HEADING','/WIND']]
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        if connection.topic == '/GPS':
            latitude_deg =  int((msg.latitude*100)/100)
            longitude_deg =  int((msg.longitude*100)/100)
            latitude = latitude_deg  + (((msg.latitude*100)) - latitude_deg*100 ) /60   
            longitude = longitude_deg  + (((msg.longitude*100)) - longitude_deg*100 ) /60  
            sog = msg.sog
            print("GPS ------")
            print(f"received :    {msg.latitude}   {msg.longitude}")
            print(f"converted :   {latitude}    {longitude}\n")
        elif connection.topic == '/HEADING':
            heading = msg.heading
            print("HEADING -------")
            print(f"received :  {heading}\n")
        elif connection.topic == '/WIND' : 
            wd = msg.wind_direction
            print("WIND -------")
            print(f"received :  {wd}\n")

        if t0 == 0:
            t0 = timestamp

        val = (t0,timestamp,latitude,longitude,heading,sog,wd)
        # db.cursor().execute("DELETE FROM boat;")
        db.cursor().execute(sql, val,multi=False)
        db.commit()

        if old!=0 : 
            t_to_wait = (timestamp-old)/10**9 # in seconds
            time.sleep(t_to_wait/speed)
            # time.sleep(0.1)
            print((timestamp-old)/10**9)
        old = timestamp
