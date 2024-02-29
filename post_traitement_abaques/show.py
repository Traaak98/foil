from rosbags.highlevel import AnyReader
from rosbags.typesys import get_types_from_msg, register_types

from pathlib import Path

speed = 1

############ IMPORTING CUSTOM MESSAGES ############

tmp_dir = "/home/vivi/Documents/ENSTAB/S5/6.1-Guerledan/HydroChal-ROS2/src/interfaces/msg"

msg_path_list = [f'{tmp_dir}/Date.msg',
                 f'{tmp_dir}/Time.msg',
                 f'{tmp_dir}/GPS.msg',
                 f'{tmp_dir}/HEADING.msg',
                 f'{tmp_dir}/WIND.msg',
                 f'{tmp_dir}/YPR.msg',]



def guess_msgtype(path):
    """Trouver le nom du message à partir du chemin."""
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


################# CONFIG WEBSITE MONITORING #################

"""
import mysql.connector

db = mysql.connector.connect(
          host="localhost",
          user="monitor",
          password="",
          database="sailboat"
        )

sql = "INSERT INTO boat (ros_timestamp_t0,ros_timestamp, lat, lon, heading, speed, true_wind_direction) VALUES (%s,%s,%s,%s,%s,%s,%s)"
"""
#############################################################

import time

rosbag_path = "rosbag_ensta_test_gps/rosbag2_2023_11_24-09_05_06"


latitude = 0
longitude = 0
heading = 0
sog = 0
wd = 0

old = 0
"""
db.cursor().execute("DELETE FROM boat;")
"""
t0 = 0 



with AnyReader([Path(rosbag_path)]) as reader:
    connections = [x for x in reader.connections if x.topic in ['/Attitude','/Vitesse','/Hauteur']]
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        
        if connection.topic == '/Attitude':
            roll = msg.roll
            pitch= msg.pitch
            yaw  = msg.yaw 
            
        elif connection.topic == '/Vitesse':
            Vx = msg.Vx
            Vy = msg.Vy
            Vz = msg.Vz
        elif connection.topic == '/Hauteur':
            Height = msg.Height

        if t0 == 0:
            t0 = timestamp

        data = (t0, timestamp, roll, pitch, yaw, Vx, Vy, Vz, Height)

        """
        # db.cursor().execute("DELETE FROM boat;")
        db.cursor().execute(sql, val,multi=False)
        db.commit()
        """


        """
        if old!=0 : 
            t_to_wait = (timestamp-old)/10**9 # in seconds
            time.sleep(t_to_wait/speed)
            # time.sleep(0.1)
            print((timestamp-old)/10**9)
        old = timestamp
        """
