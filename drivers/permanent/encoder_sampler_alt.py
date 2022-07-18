#**********************************************************************************************************
#PROJECT: LSPE-STRIP
#
#Run From: CLIENT
#Program: "encoder_sampler_alt.py"
#
#Description:
#   This program must be running at any time the telescope is not disabled.
#   It connects to the Elevation Trio-Controller and permanent the pointing of the telescope over time.
#   The pointing of the telescope as a function of time during the motion is saved in
#   "/pointing/main_sequence_DATE_TIMESTAMP.dat".
#
#**********************************************************************************************************

import controller_definitions.elevation_definitions as altdef #importing the I/O and VR allocations of Elevation Controller
from pyModbusTCP.client import ModbusClient #package to manage Modbus connections
from utils import library as lb
import time
import json

import msgpack
import redis


#from utils.redis_definitions import *

driver_params=json.load(open("../configuration/TCS_driver_parameters.json"))

alt_ip=driver_params["ip&ports"]["alt_proxy_ip"]
alt_port=driver_params["ip&ports"]["alt_proxy_port"]

software_timeout=driver_params["safety_parameters"]["software_timeout"]

redis_ip=driver_params["ip&ports"]["redis_ip"]
redis_port=driver_params["ip&ports"]["redis_port"]

channel_stream_elevation=driver_params["redis_definitions"]["channels"]["alt_encoder_stream"]

#preparing modbus connection to altimuth Controller
modbus_alt = ModbusClient(host=alt_ip, port=alt_port, debug=False)

try:
    # connect to the REDIS server
    client = redis.Redis(host=redis_ip, port=redis_port)

    #connecting to the Trio-Controller
    lb.Connect_to_Controller(modbus_alt, "Elevation")

    # saving the time written on Controller memory on a temporary variable to check for time updates
    tmp = lb.Read_32_bit_floats(modbus_alt, altdef.master_time_to_ws, 3)

    # while the Controller is running
    while True:  # lb.Read_32_bit_floats(modbus_az, azdef.system_status)[0]!=azdef.disabled:
        # read the time and positions written on controller memory
        pos_time = lb.Read_32_bit_floats(modbus_alt, altdef.master_time_to_ws, 4)

        # every time the ticks parameter is modified
        if pos_time[1] != tmp[1]:
            packet={"encoder_pos": pos_time[2],
                "time_from_MC": pos_time[0],
                "ticks_since_TOS": pos_time[1]
                }
            client.publish(channel_stream_elevation, msgpack.packb(packet))
            tmp = pos_time

    lb.Disconnect_Controller(modbus_alt, "Elevation")

except:
    lb.Disconnect_Controller(modbus_alt, "Elevation")