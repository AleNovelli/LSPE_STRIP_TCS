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

#determining the safety parameters of the motion
alt_max=driver_params["general"]["max_alt"]
alt_min=driver_params["general"]["min_alt"]
elongation_from_sun=driver_params["general"]["elongation_from_sun"]

elongation = Angle(elongation_from_sun, unit="degree") #elongation that must be kept from the sun


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
    params=np.transpose(params) #transposing the list in order to cicle over the points more easily
    params=list(params.astype(np.float32)) #converting the parameters from numpy strings to list of floats

    #Writing the total number of points on the controllers and setting the current point to -1
    lb.Write_32_bit_floats(modbus_az, azdef.num_of_points, len(params))
    lb.Write_32_bit_floats(modbus_alt, altdef.num_of_points, len(params))
    lb.Write_32_bit_floats(modbus_az, azdef.current_point, -1)
    lb.Write_32_bit_floats(modbus_alt, altdef.current_point, -1)

    # if the system is disabled enable all axis
    lb.Enable_Motion(modbus_az, "az", az_status, software_timeout)
    lb.Enable_Motion(modbus_alt, "alt", alt_status, software_timeout)

    for j, position in enumerate(params):
        print("Point", j, position)
        a=input("---")
        final_alt, final_az, duration = position

        # getting the position of the sun
        # observing location in Tenerife
        observing_location = EarthLocation(lat=telescope_lat, lon=telescope_lon, height=telescope_height * u.m)
        Location_Time = AltAz(location=observing_location, obstime=Time(datetime.now()))  # adding current time
        sun = get_sun(Time(datetime.now())).transform_to(Location_Time)  # getting the position of the sun
        sun.az.wrap_at("180d", inplace=True)  # defining Az coordinate to be kept between (-180deg and 180deg)

        # UTILIZZATO PER I TEST, DA ELIMINARE DOPO
        sun = np.array([[-89, 35]])
        sun = SkyCoord(sun, unit="degree", frame='altaz')
        sun.az.wrap_at("180d", inplace=True)

        # getting the position of the telescope and the position to be reached
        az_encoder_pos = lb.Read_32_bit_floats(modbus_az, azdef.ax0_mpos)
        alt_encoder_pos = lb.Read_32_bit_floats(modbus_alt, altdef.ax0_mpos)
        # start_pos=Somefunction(az_encoder_pos, alt_encoder_pos)

        # DA MODIFICARE
        alt = lb.Read_32_bit_floats(modbus_alt, altdef.ax_mpos)[0]
        az = lb.Read_32_bit_floats(modbus_az, azdef.ax_mpos)[0]
        print(alt,az)
        input("---")
        start = np.array([[az, alt]])
        start = SkyCoord(start, unit="degree", frame='altaz')
        start.az.wrap_at("180d", inplace=True)

        finish = np.array([[final_az, final_alt]])
        finish = SkyCoord(finish, unit="degree", frame='altaz')
        finish.az.wrap_at("180d", inplace=True)

        az_traj, alt_traj = lb.Traj_to_Pos(sun, start, finish, elongation, alt_max, alt_min)

        traj_lenght = len(
            az_traj)  # saving the lenght of the trajectory (they have the same len so i take len of az_traj)

        # The trajectory caluclated in order to avoid the sun will never exceed 8 points. If the trajectory exceeds 8 points
        #  there has been an error in its calculation and it can't fit in the allocated memory
        if traj_lenght > 8 or len(az_traj) != len(alt_traj):
            raise Exception("CRITICAL ERROR!\n" +
                            "Error in calculating the trjectory to avoid the sun")
        elif traj_lenght==1:
            print("The telescope is already in position!!!")
            lb.Disconnect_Controller(modbus_az, "Azimuth")
            lb.Disconnect_Controller(modbus_alt, "Elevation")
            exit()

        # writing the trajectory and its lenght on the TrioControllers
        lb.Write_32_bit_floats(modbus_az, azdef.jog_traj_lenght, traj_lenght)
        lb.Write_32_bit_floats(modbus_az, azdef.jog_traj, az_traj)
        lb.Write_32_bit_floats(modbus_alt, altdef.jog_traj_lenght, traj_lenght)
        lb.Write_32_bit_floats(modbus_alt, altdef.jog_traj, alt_traj)

        #if we have just loaded the trajectory to the first point of the list give the command to start calibration
        if j==0:
            lb.Write_32_bit_floats(modbus_az, azdef.motion_command, azdef.usr_calibration)
            lb.Write_32_bit_floats(modbus_alt, altdef.motion_command, altdef.usr_calibration)

            #And than wait for the calibration sequence to start
            lb.Wait_until_register(modbus_az, azdef.system_status, azdef.calibrating, software_timeout)
            lb.Wait_until_register(modbus_alt, altdef.system_status, altdef.calibrating, software_timeout)

        # signaling the controllers that they are ready to move to the next position
        lb.Write_32_bit_floats(modbus_az, azdef.next_point_ready, 1)
        lb.Write_32_bit_floats(modbus_alt, altdef.next_point_ready, 1)

        #mediating the communications between the controllers during the jog
        for i in range(1, traj_lenght):
            # printing on terminal the motion needed by the two axis (only one axis move at a time)
            print("AZ: " + str(az_traj[i - 1]) + "-->" + str(az_traj[i]))
            print("ALT: " + str(alt_traj[i - 1]) + "-->" + str(alt_traj[i]))

            # if it is the turn of the azimuth azis to move
            if az_traj[i]!=az_traj[i-1] and alt_traj[i]==alt_traj[i-1]:
                # wait for azimuth axis to be idle and ready for motion
                lb.Wait_until_coils(modbus_az, azdef.io_user_azimuth_idle, motion_timeout)
                lb.Wait_until_coils(modbus_az, azdef.io_user_azimuth_idle, motion_timeout)
                # switch off the ide signal on azimuth controller
                modbus_az.write_single_coil(azdef.io_user_azimuth_idle, False)
                # giving the signal to the alt controller has reached idle state
                modbus_alt.write_single_coil(altdef.in_user_azimuth_idle, True)
                print("AZ Moving!!!!")
            elif alt_traj[i] != alt_traj[i - 1] and az_traj[i] == az_traj[i - 1]:
                lb.Wait_until_coils(modbus_alt, altdef.io_user_elevation_idle, motion_timeout)
                modbus_alt.write_single_coil(altdef.io_user_elevation_idle, False)
                modbus_az.write_single_coil(azdef.in_user_elevation_idle, True)
                print("ALT Moving!!!!")
            else:
                raise Exception("CRITICAL ERROR:\n The trajectory calculated to avoid the sun is not valid!!")


        #waiting for the telescope to have reached the position
        lb.Wait_until_register(modbus_az, azdef.current_point, j, motion_timeout)
        lb.Wait_until_register(modbus_alt, altdef.current_point, j, motion_timeout)

        print("Waiting: ", duration)
        #waiting the specified time
        time.sleep(duration)

    lb.Disconnect_Controller(modbus_az, "Azimuth")
    lb.Disconnect_Controller(modbus_alt, "Elevation")

except KeyboardInterrupt:
    lb.Disconnect_Controller(modbus_az, "Azimuth")
    lb.Disconnect_Controller(modbus_alt, "Elevation")