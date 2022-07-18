#**********************************************************************************************************
#PROJECT: LSPE-STRIP
#
#Run From: CLIENT
#Program: "define_position.py"
#
#Description:
#   This program connects to the LSPE-SWIPE telescope and redefines the position of the telescope.
#   By running this program we redefine the encoder value at which we set the 0 degrees position of the two axis
#**********************************************************************************************************
import time

import controller_definitions.azimuth_definitions as azdef #importing the I/O and VR allocations of Azimuth Controller
import controller_definitions.elevation_definitions as altdef #importing the I/O and VR allocations of Elevation Controller
from pyModbusTCP.client import ModbusClient #package to manage Modbus connections
from utils import library as lb
import json

import msgpack
import redis
import sys

param_file=open("../configuration/TCS_driver_parameters.json")
driver_params=json.load(param_file)

az_ip=driver_params["ip&ports"]["az_proxy_ip"]
az_port=driver_params["ip&ports"]["az_proxy_port"]
alt_ip=driver_params["ip&ports"]["alt_proxy_ip"]
alt_port=driver_params["ip&ports"]["alt_proxy_port"]

redis_ip=driver_params["ip&ports"]["redis_ip"]
redis_port=driver_params["ip&ports"]["redis_port"]
redis_answerback="failure"

software_timeout=driver_params["safety_parameters"]["software_timeout"]
motion_timeout=driver_params["safety_parameters"]["movement_timeout"]

channel_motion_command=driver_params["redis_definitions"]["channels"]["motion_commands"]
channel_motion_answerback=driver_params["redis_definitions"]["channels"]["motion_answerbacks"]
current_motion_var=driver_params["redis_definitions"]["variables"]["current_motion"]


#print(sys.argv[0], sys.argv[1])
parameters=json.loads(sys.argv[1])

alt_pos=parameters["alt_pos"]
alt_enc=parameters["alt_enc"]

az_pos=parameters["az_pos"]
az_enc=parameters["az_enc"]

modbus_alt=None
modbus_az=None
try:
    modbus_az = ModbusClient(host=az_ip, port=az_port, debug=False) #preparing modbus connection to Azimuth Controller
    modbus_alt = ModbusClient(host=alt_ip, port=alt_port, debug=False) #preparing modbus connection to Elevation Controller

    #connecting to the two controllers
    lb.Connect_to_Controller(modbus_az, "Azimuth")
    lb.Connect_to_Controller(modbus_alt, "Elevation")

    #control if the system can be used for motion
    az_status =lb.Test_Controller_Available(modbus_az, "az")
    alt_status = lb.Test_Controller_Available(modbus_alt, "alt")


    #Writing the parameters for the new positions on the Trio-Controllers
    lb.Write_32_bit_floats(modbus_az, azdef.az_pos_def, az_pos)
    lb.Write_32_bit_floats(modbus_az, azdef.az_enc_def, az_enc)

    lb.Write_32_bit_floats(modbus_alt, altdef.alt_pos_def, alt_pos)
    lb.Write_32_bit_floats(modbus_alt, altdef.alt_enc_def, alt_enc)

    # if the system is disabled enable all axis
    lb.Enable_Motion(modbus_az, "az", az_status, software_timeout)
    lb.Enable_Motion(modbus_alt, "alt", alt_status, software_timeout)

    # giving the comand to redefine the position
    lb.Write_32_bit_floats(modbus_az, azdef.motion_command, azdef.redefine_pos)
    lb.Write_32_bit_floats(modbus_alt, altdef.motion_command, altdef.redefine_pos)

    # Waiting for the controllers to have finished the redefinition
    lb.Wait_until_coils(modbus_az, azdef.io_user_azimuth_idle, software_timeout)
    lb.Wait_until_coils(modbus_alt, altdef.io_user_elevation_idle, software_timeout)

    #signaling the two motors that also the other controller has redefined the position succesfully
    modbus_alt.write_single_coil(altdef.io_user_elevation_idle, False)
    modbus_az.write_single_coil(azdef.in_user_elevation_idle, False)

    #redefining the encoder position of the 0 degrees of the two azis in the azis parameters
    driver_params["encoder_pos_at_zero_deg"]["az"]=int(az_enc-az_pos/360*(2**32-1))
    driver_params["encoder_pos_at_zero_deg"]["alt"] = int(alt_enc - alt_pos / 360 * (2**32 - 1))

    lb.Disconnect_Controller(modbus_az, "Azimuth")
    lb.Disconnect_Controller(modbus_alt, "Elevation")

    redis_client = redis.Redis(host=redis_ip, port=redis_port)
    redis_client.set(current_motion_var, "none")
    answerback = {'type': "answerback", 'from': "redefine_pos",
                  "answer": "success"}
    redis_client.publish(channel_motion_answerback, msgpack.packb(answerback))

except:
    print("Error in redefining telescope position:", redis_answerback)
    redis_client = redis.Redis(host=redis_ip, port=redis_port)
    answerback = {'type': "answerback", 'from': "redefine_pos",
                  "answer": redis_answerback}
    redis_client.publish(channel_motion_answerback, msgpack.packb(answerback))
    redis_client.set(current_motion_var, "none")

    if modbus_az:
        lb.Disconnect_Controller(modbus_az, "Azimuth")
    if modbus_alt:
        lb.Disconnect_Controller(modbus_alt, "Elevation")