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

from astropy.coordinates import SkyCoord, EarthLocation, AltAz, get_sun
from astropy.time import Time
from astropy import units as u

import os #da UTILIZZARE (per ora NON mi serve perchè sto lavorando su windows)
import subprocess #<---da ELIMINARE (per ora mi serve perchè sto lavorando su windows)

driver_params=json.load(open("TCS_driver_parameters.json"))

az_ip=driver_params["general"]["ip_az_controller"]
az_port=driver_params["general"]["port_az_controller"]
alt_ip=driver_params["general"]["ip_alt_controller"]
alt_port=driver_params["general"]["port_alt_controller"]
software_timeout=driver_params["general"]["software_timeout"]
motion_timeout=driver_params["general"]["movement_timeout"]
telescope_lon=driver_params["general"]["telescope_site_lon"]
telescope_lat=driver_params["general"]["telescope_site_lat"]
telescope_height=driver_params["general"]["telescope_height"]
elongation=driver_params["general"]["elongation_from_sun"]

alt=driver_params["raster_scan"]["alt"]
az_min=driver_params["raster_scan"]["az_min"]
az_max=driver_params["raster_scan"]["az_max"]
start_time=driver_params["raster_scan"]["start_time"]
duration=driver_params["raster_scan"]["duration"]
speed=driver_params["raster_scan"]["speed"]
accel_to_decel_frac=driver_params["raster_scan"]["accel_to_decel_frac"]




modbus_az = ModbusClient(host=az_ip, port=az_port, debug=False) #preparing modbus connection to Azimuth Controller
modbus_alt = ModbusClient(host=alt_ip, port=alt_port, debug=False) #preparing modbus connection to Elevation Controller

try:

    print("Checking elongation from the sun...")
    times = np.linspace(start_time, start_time + duration, 1000)
    times = Time(times, format="unix")
    observing_location = EarthLocation(lat=telescope_lat, lon=telescope_lon, height=telescope_height * u.m)
    Location_Time = AltAz(location=observing_location, obstime=times)
    sun_pos = get_sun(times).transform_to(Location_Time)
    min_pos=np.repeat([[az_min,alt]], 1000, axis=0)
    min_pos = SkyCoord(min_pos, unit="degree", frame=Location_Time)
    max_pos = np.repeat([[az_max, alt]], 1000, axis=0)
    max_pos = SkyCoord(max_pos, unit="degree", frame=Location_Time)
    sun_collision = np.any(sun_pos.separation(min_pos).deg<1.2 * elongation) or \
                    np.any(sun_pos.separation(max_pos).deg<1.2 * elongation)

    if sun_collision:
        raise Exception("CRITICAL ERROR:\n" +
                        "The nominal scanning that you are loading would make the telescope point on the sun\n" +
                        "SPIN SCAN ABORTED!")

    exit()

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