#**********************************************************************************************************
#PROJECT: LSPE-STRIP
#
#Run From: WORKSTATION
#Program: "test_encoder.py"
#
#Description:
#   This program connects to the LSPE-SWIPE telescope and tests the accuracy of the encoders.
#   The encoders continuously make a random motion and then return to a given position, the encoder value is then
#   measured.
#
#**********************************************************************************************************

import numpy as np
from datetime import datetime

import controller_definitions.azimuth_definitions as azdef #importing the I/O and VR allocations of Azimuth Controller
from pyModbusTCP.client import ModbusClient #package to manage Modbus connections
from utils import library as lb

import json

driver_params=json.load(open("../configuration/TCS_driver_parameters.json"))

software_timeout=driver_params["safety_parameters"]["software_timeout"]
motion_timeout=driver_params["safety_parameters"]["movement_timeout"]

az_ip=driver_params["ip&ports"]["az_proxy_ip"]
az_port=driver_params["ip&ports"]["az_proxy_port"]
alt_ip=driver_params["ip&ports"]["alt_proxy_ip"]
alt_port=driver_params["ip&ports"]["alt_proxy_port"]

modbus_az = ModbusClient(host=az_ip, port=az_port, debug=False) #preparing modbus connection to Azimuth Controller
#modbus_alt = ModbusClient(host=alt_ip, port=alt_port, debug=False) #preparing modbus connection to Elevation Controller

try:

    # connecting to the two controllers
    lb.Connect_to_Controller(modbus_az, "Azimuth")
    #lb.Connect_to_Controller(modbus_alt, "Elevation")

    # control if the system can be used for motion
    az_status = lb.Test_Controller_Available(modbus_az, "az")
    #alt_status = lb.Test_Controller_Available(modbus_alt, "alt")

    # if the system is disabled enable all axis
    lb.Enable_Motion(modbus_az, "az", az_status, software_timeout)
    #lb.Enable_Motion(modbus_alt, "alt", alt_status, software_timeout)

    # giving the command to start the sequence
    lb.Write_32_bit_floats(modbus_az, azdef.motion_command, azdef.encoder_test)
    #lb.Write_32_bit_floats(modbus_alt, altdef.motion_command, altdef.test_encoder)

    filename="test_Bolo/encoder/enc_test"+datetime.now().strftime("%Y_%m_%d__%H_%M_%S.npy")

    enc_values=np.array([])

    lb.Wait_until_register(modbus_az, azdef.encoder_test_ready, 0, software_timeout, period=0.1)

    while True:
        lb.Wait_until_register(modbus_az, azdef.encoder_test_ready, 1, motion_timeout, period=0.1)
        enc_value=lb.Read_32_bit_floats(modbus_az, azdef.encoder_test_value)[0]
        enc_values = np.append(enc_values, enc_value)
        np.save(filename, enc_values)
        lb.Write_32_bit_floats(modbus_az, azdef.encoder_test_ready, 0)


    lb.Disconnect_Controller(modbus_az, "Azimuth")
    #lb.Disconnect_Controller(modbus_alt, "Elevation")

except KeyboardInterrupt:
    lb.Disconnect_Controller(modbus_az, "Azimuth")
    lb.Disconnect_Controller(modbus_alt, "Elevation")
