#**********************************************************************************************************
#PROJECT: LSPE-STRIP
#
#Run From: CLIENT
#Program: "move_to_start.py"
#
#Description:
#   This program moves the telescope to a given position avoiding the sun during the motion.
#   After the position of the telescope and the sun are determined a series of motions is calculated and sent to the
#   controllers to be executed.
#**********************************************************************************************************

import numpy as np
from astropy.coordinates import SkyCoord, Angle, AltAz
from astropy.coordinates import get_sun
from astropy.coordinates import EarthLocation
from astropy.time import Time
from astropy import units as u
from datetime import datetime

import controller_definitions.azimuth_definitions as azdef  #importing the I/O and VR allocations of Azimuth Controller
import controller_definitions.elevation_definitions as altdef #importing the I/O and VR allocations of Elevation Controller
from pyModbusTCP.client import ModbusClient #package to manage Modbus connections
from utils import library as lb
import json
import sys

import msgpack
import redis

#from utils.redis_definitions import *

import logging
log = logging.getLogger("moveto_motion")
log.setLevel(logging.INFO)
formatter = logging.Formatter('%(asctime)s :: %(levelname)s :: %(filename)s :: %(message)s')
stream_handler = logging.StreamHandler()
stream_handler.setFormatter(formatter)
log.addHandler(stream_handler)

driver_params=json.load(open("../configuration/TCS_driver_parameters.json"))

az_ip=driver_params["ip&ports"]["az_proxy_ip"]
az_port=driver_params["ip&ports"]["az_proxy_port"]
alt_ip=driver_params["ip&ports"]["alt_proxy_ip"]
alt_port=driver_params["ip&ports"]["alt_proxy_port"]

redis_ip=driver_params["ip&ports"]["redis_ip"]
redis_port=driver_params["ip&ports"]["redis_port"]
redis_answerback="failure"

telescope_lon=driver_params["telescope_site"]["lon"]
telescope_lat=driver_params["telescope_site"]["lat"]
telescope_height=driver_params["telescope_site"]["height"]

software_timeout=driver_params["safety_parameters"]["software_timeout"]
motion_timeout=driver_params["safety_parameters"]["movement_timeout"]
elongation_from_sun=driver_params["safety_parameters"]["elongation_from_sun"]
alt_max=driver_params["safety_parameters"]["max_alt"]
alt_min=driver_params["safety_parameters"]["min_alt"]

channel_motion_command=driver_params["redis_definitions"]["channels"]["motion_commands"]
channel_motion_answerback=driver_params["redis_definitions"]["channels"]["motion_answerbacks"]
current_motion_var=driver_params["redis_definitions"]["variables"]["current_motion"]

#print(sys.argv[0], sys.argv[1])
parameters=json.loads(sys.argv[1])
alt=parameters["alt"]
az=parameters["az"]

modbus_alt=None
modbus_az=None
try:
    log.info("Telescope motion to new fixed position started")
    if alt>alt_max or alt<alt_min:
        redis_answerback="invalid parameters"
        raise Exception("Moving telescope outside of elevation safety range")

    """
    alt=driver_params["move_to_start"]["alt"]
    az=driver_params["move_to_start"]["az"]
    addr_alt=driver_params["move_to_start"]["address"]
    """

    elongation = Angle(elongation_from_sun, unit="degree") #elongation that must be kept from the sun

    modbus_az = ModbusClient(host=az_ip, port=az_port, debug=False) #preparing modbus connection to Azimuth Controller
    modbus_alt = ModbusClient(host=alt_ip, port=alt_port, debug=False) #preparing modbus connection to Elevation Controller

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

    finish = np.array([[az, alt]])
    finish = SkyCoord(finish, unit="degree", frame='altaz')
    finish.az.wrap_at("180d", inplace=True)

    if sun.separation(finish) < elongation:
        redis_answerback="invalid parameters"
        raise Exception("Moving telescope too close to the sun")

    # connecting to the two controllers
    lb.Connect_to_Controller(modbus_az, "Azimuth")
    lb.Connect_to_Controller(modbus_alt, "Elevation")

    # control if the system can be used for motion
    az_status = lb.Test_Controller_Available(modbus_az, "az")
    alt_status = lb.Test_Controller_Available(modbus_alt, "alt")

    #getting the position of the telescope and the position to be reached
    az_encoder_pos=lb.Read_32_bit_floats(modbus_az, azdef.ax_mpos)[0]
    alt_encoder_pos=lb.Read_32_bit_floats(modbus_alt, altdef.ax_mpos)[0]

    start_alt, start_az=lb.Encoder_to_AltAZ(alt_encoder_pos, az_encoder_pos)
    start = np.array([[start_az, start_alt]])
    start = SkyCoord(start, unit="degree", frame='altaz')
    start.az.wrap_at("180d", inplace=True)

    #DA MODIFICARE
    #start=np.array([[10,40]])
    #start = SkyCoord(start, unit="degree", frame='altaz')
    #start.az.wrap_at("180d", inplace=True)
    if start.separation(finish).deg<1/36000:
        log.debug("The telescope is already positioned correctly with a precision higher than 1/10 arcsec")
    else:
        az_traj, alt_traj = lb.Traj_to_Pos(sun, start, finish, elongation, alt_max, alt_min)

        traj_lenght = len(az_traj)  # saving the lenght of the trajectory (they have the same len so i take len of az_traj)

        #The trajectory calculated in order to avoid the sun will never exceed 8 points. If the trajectory exceeds 8 points
        #  there has been an error in its calculation and it can't fit in the allocated memory
        if traj_lenght > 8 or len(az_traj)!=len(alt_traj):
            raise Exception("CRITICAL ERROR!\n"+
                            "Error in calculating the trjectory to avoid the sun")

        #writing the trajectory and its lenght on the TrioControllers
        lb.Write_32_bit_floats(modbus_az, azdef.jog_traj_lenght, traj_lenght)
        lb.Write_32_bit_floats(modbus_az, azdef.jog_traj, az_traj)
        lb.Write_32_bit_floats(modbus_alt, altdef.jog_traj_lenght, traj_lenght)
        lb.Write_32_bit_floats(modbus_alt, altdef.jog_traj, alt_traj)

        #if the system is disabled enable all axis
        lb.Enable_Motion(modbus_az, "az", az_status, software_timeout)
        lb.Enable_Motion(modbus_alt, "alt", alt_status, software_timeout)

        #giving the command to start the sequence
        lb.Write_32_bit_floats(modbus_az, azdef.motion_command, azdef.usr_move_avoid_sun)
        lb.Write_32_bit_floats(modbus_alt, altdef.motion_command, altdef.usr_move_avoid_sun)

        #for the pointings in the list
        for i in range(1, traj_lenght):
            #printing on terminal the motion needed by the two axis (only one axis move at a time)
            #print("AZ: "+str(az_traj[i-1])+"-->"+str(az_traj[i]))
            #print("ALT: " + str(alt_traj[i - 1]) + "-->" + str(alt_traj[i]))

            #if it is the turn of the azimuth azis to move
            if az_traj[i]!=az_traj[i-1] and alt_traj[i]==alt_traj[i-1]:
                #wait for azimuth axis to be idle and ready for motion
                lb.Wait_until_coils(modbus_az, azdef.io_user_azimuth_idle, motion_timeout)
                #switch off the ide signal on azimuth controller
                modbus_az.write_single_coil(azdef.io_user_azimuth_idle, False)
                #giving the signal to the alt controller has reached idle state
                modbus_alt.write_single_coil(altdef.in_user_azimuth_idle, True)
                #print("AZ Moving!!!!")
            elif alt_traj[i]!=alt_traj[i-1] and az_traj[i]==az_traj[i-1]:
                lb.Wait_until_coils(modbus_alt, altdef.io_user_elevation_idle, motion_timeout)
                modbus_alt.write_single_coil(altdef.io_user_elevation_idle, False)
                modbus_az.write_single_coil(azdef.in_user_elevation_idle, True)
                #print("ALT Moving!!!!")
            else:
                raise Exception("CRITICAL ERROR:\n The trajectory calculated to avoid the sun is not valid!!")

    lb.Disconnect_Controller(modbus_az, "Azimuth")
    lb.Disconnect_Controller(modbus_alt, "Elevation")

    redis_client = redis.Redis(host=redis_ip, port=redis_port)
    redis_client.set(current_motion_var, "none")
    answerback = {'type': "answerback", 'from': "move to",
            "answer":"success"}
    redis_client.publish(channel_motion_answerback, msgpack.packb(answerback))
    log.info("Telescope motion to new fixed position terminated")

except Exception as e:
    log.critical("Error in running move_to_start motion:", redis_answerback)
    log.exception(e)
    redis_client = redis.Redis(host=redis_ip, port=redis_port)
    answerback = {'type': "answerback", 'from': "move to",
                  "answer": redis_answerback}
    redis_client.publish(channel_motion_answerback, msgpack.packb(answerback))
    redis_client.set(current_motion_var, "none")

    if modbus_az:
        lb.Disconnect_Controller(modbus_az, "Azimuth")
    if modbus_alt:
        lb.Disconnect_Controller(modbus_alt, "Elevation")




