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
from utils import library as lb
import json
import time

import msgpack
import redis
import sys

import logging
log = logging.getLogger("raster_motion")
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

channel_motion_command=driver_params["redis_definitions"]["channels"]["motion_commands"]
channel_motion_answerback=driver_params["redis_definitions"]["channels"]["motion_answerbacks"]
current_motion_var=driver_params["redis_definitions"]["variables"]["current_motion"]

alt_max=driver_params["safety_parameters"]["max_alt"]
alt_min=driver_params["safety_parameters"]["min_alt"]
speed_max=driver_params["safety_parameters"]["max_speed"]

#print(sys.argv[0], sys.argv[1])
parameters=json.loads(sys.argv[1])
alt=parameters["alt"]
az_min=parameters["az_min"]
az_max=parameters["az_max"]
#start_time=parameters["start_time"]
duration=parameters["duration"]
speed=parameters["speed"]
accel_to_decel_frac=parameters["accel_to_decel_frac"]

modbus_alt=None
modbus_az=None

try:
	log.info("Raster scan sequence started")
    if alt>alt_max or alt<alt_min:
        redis_answerback="invalid parameters"
        raise Exception("Moving telescope outside of elevation safety range")
    if abs(speed)> speed_max:
        redis_answerback="invalid parameters"
        raise Exception("Moving telescope too fast")

    if accel_to_decel_frac< 1:
        redis_answerback="invalid parameters"
        raise Exception("Accel to decel frac parameter must be > 1")

    modbus_az = ModbusClient(host=az_ip, port=az_port, debug=False) #preparing modbus connection to Azimuth Controller
    modbus_alt = ModbusClient(host=alt_ip, port=alt_port, debug=False) #preparing modbus connection to Elevation Controller

    """
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
    """

    # connecting to the two controllers
    lb.Connect_to_Controller(modbus_az, "Azimuth")
    lb.Connect_to_Controller(modbus_alt, "Elevation")

    # control if the system can be used for motion
    az_status = lb.Test_Controller_Available(modbus_az, "az")
    alt_status = lb.Test_Controller_Available(modbus_alt, "alt")

    #Write the configuration of the raster scan on the Trio-Controllers
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

    # giving the comand to start the raster scan
    lb.Write_32_bit_floats(modbus_az, azdef.motion_command, azdef.usr_raster_scan)
    lb.Write_32_bit_floats(modbus_alt, altdef.motion_command, altdef.usr_raster_scan)

    # Waiting for the Elevation motor to be IDLE
    lb.Wait_until_coils(modbus_alt, altdef.io_user_elevation_idle, motion_timeout)
    modbus_alt.write_single_coil(altdef.io_user_elevation_idle, False)  # switchin off coil
    modbus_az.write_single_coil(azdef.in_user_elevation_idle, True)  # signaling that Elevation is IDLE to AZ controller

    # waiting for Azimuth motion to be almost finished
    time.sleep(duration - 2)
    lb.Wait_until_coils(modbus_az, azdef.io_user_azimuth_idle, motion_timeout)
    modbus_az.write_single_coil(azdef.io_user_azimuth_idle, False)  # switchin off coil
    modbus_alt.write_single_coil(altdef.in_user_azimuth_idle, True)  # signaling that AZ is IDLE to Elevation controller

    lb.Disconnect_Controller(modbus_az, "Azimuth")
    lb.Disconnect_Controller(modbus_alt, "Elevation")

    redis_client = redis.Redis(host=redis_ip, port=redis_port)
    redis_client.set(current_motion_var, "none")
    answerback = {'type': "answerback", 'from': "raster",
                  "answer": "success"}
    redis_client.publish(channel_motion_answerback, msgpack.packb(answerback))
    log.info("Raster scan sequence terminated")

except Exception as e:
    log.critical("Error in running raster_scan motion:", redis_answerback)
    log.exception(e)
    redis_client = redis.Redis(host=redis_ip, port=redis_port)
    answerback = {'type': "answerback", 'from': "raster",
                  "answer": redis_answerback}
    redis_client.publish(channel_motion_answerback, msgpack.packb(answerback))
    redis_client.set(current_motion_var, "none")

    if modbus_az:
        lb.Disconnect_Controller(modbus_az, "Azimuth")
    if modbus_alt:
        lb.Disconnect_Controller(modbus_alt, "Elevation")
