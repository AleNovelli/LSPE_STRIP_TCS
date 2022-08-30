#**********************************************************************************************************
#PROJECT: LSPE-STRIP
#
#Run From: CLIENT
#Program: "encoder_sampler_az.py"
#
#Description:
#   This program must be running at any time the telescope is not disabled.
#   It connects to the Azimuth Trio-Controller and permanent the pointing of the telescope over time.
#   The pointing of the telescope as a function of time during the motion is saved in
#   "/pointing/main_sequence_DATE_TIMESTAMP.dat".
#
#**********************************************************************************************************

import controller_definitions.azimuth_definitions as azdef #importing the I/O and VR allocations of Azimuth Controller
from pyModbusTCP.client import ModbusClient #package to manage Modbus connections
from utils import library as lb
import time
import json

import redis
import msgpack
#from utils.redis_definitions import *

import logging
log = logging.getLogger("encoder sampler alt")
log.setLevel(logging.INFO)
formatter = logging.Formatter('%(asctime)s :: %(levelname)s :: %(filename)s :: %(message)s')
stream_handler = logging.StreamHandler()
stream_handler.setFormatter(formatter)
log.addHandler(stream_handler)

driver_params=json.load(open("../configuration/TCS_driver_parameters.json"))

az_ip=driver_params["ip&ports"]["az_proxy_ip"]
az_port=driver_params["ip&ports"]["az_proxy_port"]

software_timeout=driver_params["safety_parameters"]["software_timeout"]

redis_ip=driver_params["ip&ports"]["redis_ip"]
redis_port=driver_params["ip&ports"]["redis_port"]

channel_stream_azimuth=driver_params["redis_definitions"]["channels"]["az_encoder_stream"]


#preparing modbus connection to Azimuth Controller
modbus_az = ModbusClient(host=az_ip, port=az_port, debug=False)

try:
    log.info("Encoder Sampler AZ connecting...")
    # connect to the REDIS server
    client = redis.Redis(host=redis_ip, port=redis_port)

    #connecting to the Trio-Controller
    lb.Connect_to_Controller(modbus_az, "Azimuth")
    
    log.info("Encoder Sampler AZ launched")
    
    # saving the time written on Controller memory on a temporary variable to check for time updates
    tmp = lb.Read_32_bit_floats(modbus_az, azdef.master_time_to_ws, 4)

    #time_tmp=0

    # while the Controller is running
    while True:#lb.Read_32_bit_floats(modbus_az, azdef.system_status)[0]!=azdef.disabled:
        # read the time and positions written on controller memory
        pos_time = lb.Read_32_bit_floats(modbus_az, azdef.master_time_to_ws, 4)

        # every time the ticks parameter is modified
        if pos_time[1]!= tmp[1]:
            packet = {"encoder_pos": pos_time[2],
                      "time_from_MC": pos_time[0],
                      "ticks_since_TOS": -pos_time[1]+pos_time[3]
                      }
            client.publish(channel_stream_azimuth, msgpack.packb(packet))


            #tmp = pos_time
            #time_ac=pos_time[0]-(pos_time[1]-pos_time[3])/1000
            #print("Time: {:f} Time passed: {:f}".format(time_ac, time_ac-time_tmp))
            #time_tmp=time_ac

    lb.Disconnect_Controller(modbus_az, "Azimuth")

except Exception as e:
	log.exception(e)
	lb.Disconnect_Controller(modbus_az, "Azimuth")
