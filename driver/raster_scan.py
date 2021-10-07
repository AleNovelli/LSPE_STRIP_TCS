#**********************************************************************************************************
#PROJECT: LSPE-STRIP
#
#Run From: CLIENT
#Program: "raster_scan.py"
#
#Description:
#   This program connects to the LSPE-SWIPE telescope and launches the raster_scan sequence.
#
#**********************************************************************************************************

import controller_definitions.azimuth_definitions as azdef #importing the I/O and VR allocations of Azimuth Controller
import controller_definitions.elevation_definitions as altdef #importing the I/O and VR allocations of Elevation Controller
from pyModbusTCP.client import ModbusClient #package to manage Modbus connections
import library as lb
import numpy as np
import time
import json

import os #da UTILIZZARE (per ora NON mi serve perchè sto lavorando su windows)
import subprocess #<---da ELIMINARE (per ora mi serve perchè sto lavorando su windows)

driver_params=json.load(open("TCS_driver_parameters.json"))

az_ip=driver_params["general"]["ip_az_controller"]
az_port=driver_params["general"]["port_az_controller"]
alt_ip=driver_params["general"]["ip_alt_controller"]
alt_port=driver_params["general"]["port_alt_controller"]

alt=driver_params["raster_scan"]["alt"]
az_min=driver_params["raster_scan"]["az_min"]
az_max=driver_params["raster_scan"]["az_max"]
duration=driver_params["raster_scan"]["duration"]
speed=driver_params["raster_scan"]["speed"]
accel_to_decel_frac=driver_params["raster_scan"]["accel_to_decel_frac"]

software_timeout=driver_params["general"]["software_timeout"]
motion_timeout=driver_params["general"]["movement_timeout"]


modbus_az = ModbusClient(host=az_ip, port=az_port, debug=False) #preparing modbus connection to Azimuth Controller
modbus_alt = ModbusClient(host=alt_ip, port=alt_port, debug=False) #preparing modbus connection to Elevation Controller

try:
    # connecting to the two controllers
    lb.Connect_to_Controller(modbus_az, "Azimuth")
    lb.Connect_to_Controller(modbus_alt, "Elevation")

    # control if the system can be used for motion
    az_status = lb.Test_Controller_Available(modbus_az, "az")
    alt_status = lb.Test_Controller_Available(modbus_alt, "alt")

    #Write the parameters of the raster scan on the Trio-Controllers
    lb.Write_32_bit_floats(modbus_az, azdef.raster_scan_alt, alt)
    lb.Write_32_bit_floats(modbus_az, azdef.raster_scan_az_min, az_min)
    lb.Write_32_bit_floats(modbus_az, azdef.raster_scan_az_max, az_max)
    lb.Write_32_bit_floats(modbus_az, azdef.raster_scan_duration, duration)
    lb.Write_32_bit_floats(modbus_az, azdef.raster_scan_speed, speed)
    lb.Write_32_bit_floats(modbus_az, azdef.raster_scan_accel_to_decel_frac, accel_to_decel_frac)

    lb.Write_32_bit_floats(modbus_alt, altdef.raster_scan_alt, alt)
    lb.Write_32_bit_floats(modbus_alt, altdef.raster_scan_az_min, az_min)
    lb.Write_32_bit_floats(modbus_alt, altdef.raster_scan_az_max, az_max)
    lb.Write_32_bit_floats(modbus_alt, altdef.raster_scan_duration, duration)
    lb.Write_32_bit_floats(modbus_alt, altdef.raster_scan_speed, speed)
    lb.Write_32_bit_floats(modbus_alt, altdef.raster_scan_accel_to_decel_frac, accel_to_decel_frac)

    # if the system is disabled enable all axis
    lb.Enable_Motion(modbus_az, "az", az_status, software_timeout)
    lb.Enable_Motion(modbus_alt, "alt", alt_status, software_timeout)

    #Launching the encoder monitor
    # os.system("encoder_monitor.py") # da UTILIZZARE (per ora NON mi serve perchè sto lavorando su windows)
    #da ELIMINARE (per ora mi serve perchè sto lavorando su windows)
    #subprocess.call('start C:/Users/aless/OneDrive/Desktop/Borsa_INAF/LSPE_SWIPE/encoder_monitor.py',shell=True)
    #time.sleep(1.5)#waiting for encoder monitor to start

    # giving the comand to start the raster scan
    lb.Write_32_bit_floats(modbus_az, azdef.motion_command, azdef.usr_raster_scan)
    lb.Write_32_bit_floats(modbus_alt, altdef.motion_command, altdef.usr_raster_scan)

    # Waiting for the Elevation motor to be IDLE
    lb.Wait_until_coils(modbus_alt, altdef.io_user_elevation_idle, motion_timeout)
    modbus_alt.write_single_coil(altdef.io_user_elevation_idle, False)  # switchin off coil
    modbus_az.write_single_coil(azdef.in_user_elevation_idle, True)  # signaling that Elevation is IDLE to AZ controller

    lb.Disconnect_Controller(modbus_az, "Azimuth")
    lb.Disconnect_Controller(modbus_alt, "Elevation")

except KeyboardInterrupt:
    lb.Disconnect_Controller(modbus_az, "Azimuth")
    lb.Disconnect_Controller(modbus_alt, "Elevation")