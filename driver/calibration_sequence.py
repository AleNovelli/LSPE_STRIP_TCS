#**********************************************************************************************************
#PROJECT: LSPE-STRIP
#
#Run From: CLIENT
#Program: "calibration_sequence.py"
#
#Description:
#   This program connects to the LSPE-SWIPE telescope and launches the calibration sequence.
#   The presence of an operator is required to switch between different pointings of the telescope.
#
#**********************************************************************************************************

import numpy as np
from astropy.coordinates import SkyCoord, Angle, AltAz
from astropy.coordinates import get_sun
from astropy.coordinates import EarthLocation
from astropy.time import Time
from astropy import units as u
from datetime import datetime

import controller_definitions.azimuth_definitions as azdef #importing the I/O and VR allocations of Azimuth Controller
import controller_definitions.elevation_definitions as altdef #importing the I/O and VR allocations of Elevation Controller
from pyModbusTCP.client import ModbusClient #package to manage Modbus connections
import library as lb

import time
import json

import os #da UTILIZZARE (per ora NON mi serve perchè sto lavorando su windows)
import subprocess #<---da ELIMINARE (per ora mi serve perchè sto lavorando su windows)

driver_params=json.load(open("TCS_driver_parameters.json"))

az_ip=driver_params["general"]["ip_az_controller"]
az_port=driver_params["general"]["port_az_controller"]
alt_ip=driver_params["general"]["ip_alt_controller"]
alt_port=driver_params["general"]["port_alt_controller"]

telescope_lon=driver_params["general"]["telescope_site_lon"]
telescope_lat=driver_params["general"]["telescope_site_lat"]
telescope_height=driver_params["general"]["telescope_height"]

alt_address=driver_params["calibration_sequence"]["address_alt"]
az_address=driver_params["calibration_sequence"]["address_az"]

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

    #reading parameters from file
    params = lb.Read_Parameters("parameters/calibration_parameters.dat")
    params=params.astype(np.float32).tolist() #converting the parameters from numpy strings to list of floats

    for position in params:

        az, alt, duration =position

        # observing location in Tenerife
        observing_location = EarthLocation(lat=telescope_lat, lon=telescope_lon, height=telescope_height * u.m)
        Location_Time = AltAz(location=observing_location, obstime=Time(datetime.now()))  # adding current time
        # getting the position of the sun
        sun = get_sun(Time(datetime.now())).transform_to(Location_Time)  # getting the position of the sun
        sun.az.wrap_at("180d", inplace=True)  # defining Az coordinate to be kept between (-180deg and 180deg)

        # UTILIZZATO PER I TEST, DA ELIMINARE DOPO
        sun = np.array([[1, 35]])
        sun = SkyCoord(sun, unit="degree", frame='altaz')
        sun.az.wrap_at("180d", inplace=True)

        # getting the position of the telescope and the position to be reached
        az_encoder_pos = lb.Read_32_bit_floats(modbus_az, azdef.ax0_mpos)
        alt_encoder_pos = lb.Read_32_bit_floats(modbus_alt, altdef.ax0_mpos)
        # start_pos=Somefunction(az_encoder_pos, alt_encoder_pos)

        # DA MODIFICARE
        start = np.array([[10, 40]])
        start = SkyCoord(start, unit="degree", frame='altaz')
        start.az.wrap_at("180d", inplace=True)

        finish = np.array([[az, alt]])
        finish = SkyCoord(finish, unit="degree", frame='altaz')
        finish.az.wrap_at("180d", inplace=True)

        az_traj, alt_traj = lb.Traj_to_Pos(sun, start, finish, elongation, alt_max, alt_min)

        traj_lenght = len(
            az_traj)  # saving the lenght of the trajectory (they have the same len so i take len of az_traj)
        addr_az = addr_alt + traj_lenght  # storing table of azimuth right after elevations
        lb.Write_32_bit_floats(modbus_az, azdef.avoid_sun_az_table_start, addr_az)
        lb.Write_32_bit_floats(modbus_az, azdef.avoid_sun_alt_table_start, addr_alt)
        lb.Write_32_bit_floats(modbus_az, azdef.avoid_sun_len_table,
                               traj_lenght)  # writing how long the parameters lists are

        lb.Write_32_bit_floats(modbus_alt, altdef.avoid_sun_az_table_start, addr_az)
        lb.Write_32_bit_floats(modbus_alt, altdef.avoid_sun_alt_table_start, addr_alt)
        lb.Write_32_bit_floats(modbus_alt, altdef.avoid_sun_len_table,
                               traj_lenght)  # writing how long the parameters lists are

        # if the system is disabled enable all axis
        lb.Enable_Motion(modbus_az, "az", az_status, software_timeout)
        lb.Enable_Motion(modbus_alt, "alt", alt_status, software_timeout)

        # starting the sequence that allows to write on TABLE memory
        lb.Write_32_bit_floats(modbus_az, azdef.motion_command, azdef.usr_table_wr)
        lb.Write_32_bit_floats(modbus_alt, altdef.motion_command, altdef.usr_table_wr)
        # Waiting for the Trio-Controllers to have moved the modbus on 32-bit float Table memory
        lb.Wait_until_coils(modbus_az, azdef.io_user_modbus_table, software_timeout)
        lb.Wait_until_coils(modbus_alt, altdef.io_user_modbus_table, software_timeout)
        # Writing parameters on table memory
        lb.Write_32_bit_floats(modbus_az, addr_az, az_traj)
        lb.Write_32_bit_floats(modbus_az, addr_alt, alt_traj)
        lb.Write_32_bit_floats(modbus_alt, addr_az, az_traj)
        lb.Write_32_bit_floats(modbus_alt, addr_alt, alt_traj)
        # switching off Table memory on the Trio-Controller
        modbus_az.write_single_coil(azdef.io_user_modbus_table, False)
        modbus_alt.write_single_coil(altdef.io_user_modbus_table, False)
        lb.Wait_until_register(modbus_az, azdef.system_status, azdef.ready, software_timeout)
        lb.Wait_until_register(modbus_alt, altdef.system_status, altdef.ready, software_timeout)
        # Launching the encoder monitor
        # os.system("encoder_monitor.py") # da UTILIZZARE (per ora NON mi serve perchè sto lavorando su windows)
        # da ELIMINARE (per ora mi serve perchè sto lavorando su windows)
        # subprocess.run('start C:/Users/aless/OneDrive/Desktop/Borsa_INAF/LSPE_SWIPE/encoder_monitor.py', shell=True)
        # time.sleep(1.5)#waiting for encoder monitor to start

        # giving the comand to start the main sequence
        lb.Write_32_bit_floats(modbus_az, azdef.motion_command, azdef.usr_move_avoid_sun)
        lb.Write_32_bit_floats(modbus_alt, altdef.motion_command, altdef.usr_move_avoid_sun)
        for i in range(1, traj_lenght):
            print("AZ: " + str(az_traj[i - 1]) + "-->" + str(az_traj[i]))
            print("ALT: " + str(alt_traj[i - 1]) + "-->" + str(alt_traj[i]))

            if az_traj[i] != az_traj[i - 1]:
                lb.Wait_until_coils(modbus_az, azdef.io_user_azimuth_idle, motion_timeout)
                modbus_az.write_single_coil(azdef.io_user_azimuth_idle, False)
                modbus_alt.write_single_coil(altdef.in_user_azimuth_idle, True)
                print("AZ Moving!!!!")
            elif alt_traj[i] != alt_traj[i - 1]:
                lb.Wait_until_coils(modbus_alt, altdef.io_user_elevation_idle, motion_timeout)
                modbus_alt.write_single_coil(altdef.io_user_elevation_idle, False)
                modbus_az.write_single_coil(azdef.in_user_elevation_idle, True)
                print("ALT Moving!!!!")

    # if the system is disabled enable all axis
    lb.Enable_Motion(modbus_az, "az", az_status, software_timeout)
    lb.Enable_Motion(modbus_alt, "alt", alt_status, software_timeout)

    lb.Disconnect_Controller(modbus_az, "Azimuth")
    lb.Disconnect_Controller(modbus_alt, "Elevation")

except KeyboardInterrupt:
    lb.Disconnect_Controller(modbus_az, "Azimuth")
    lb.Disconnect_Controller(modbus_alt, "Elevation")