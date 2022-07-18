#**********************************************************************************************************
#PROJECT: LSPE-STRIP
#
#Run From: CLIENT
#Program: "encoder_monitor.py"
#
#Description:
#   This program must be running at any time the Trio-Controllers are turned on.
#   ....TBD
#
#**********************************************************************************************************

import controller_definitions.elevation_definitions as altdef #importing the I/O and VR allocations of Elevation Controller
import controller_definitions.azimuth_definitions as azdef #importing the I/O and VR allocations of Azimuth Controller
from pyModbusTCP.client import ModbusClient #package to manage Modbus connections
from utils import library as lb
import json
import time
import redis
import msgpack
import numpy as np

from astropy.coordinates import SkyCoord, AltAz
from astropy.coordinates import get_sun
from astropy.coordinates import EarthLocation
from astropy.time import Time
from astropy import units as u
from datetime import datetime

driver_params=json.load(open("../configuration/TCS_driver_parameters.json"))

az_ip=driver_params["ip&ports"]["az_proxy_ip"]
az_port=driver_params["ip&ports"]["az_proxy_port"]
alt_ip=driver_params["ip&ports"]["alt_proxy_ip"]
alt_port=driver_params["ip&ports"]["alt_proxy_port"]

software_timeout=driver_params["safety_parameters"]["software_timeout"]

telescope_lon=driver_params["telescope_site"]["lon"]
telescope_lat=driver_params["telescope_site"]["lat"]
telescope_height=driver_params["telescope_site"]["height"]

#determining the safety configuration of the motion
alt_max=driver_params["safety_parameters"]["max_alt"]
alt_min=driver_params["safety_parameters"]["min_alt"]
elongation_from_sun=driver_params["safety_parameters"]["elongation_from_sun"]

redis_ip=driver_params["ip&ports"]["redis_ip"]
redis_port=driver_params["ip&ports"]["redis_port"]

redis_recovery_channel=driver_params["redis_definitions"]["channels"]["fault_recovery_instructions"]
redis_fault_alert_channel=driver_params["redis_definitions"]["channels"]["fault_alert"]
current_motion_var=driver_params["redis_definitions"]["variables"]["current_motion"]

#preparing modbus connection to Controllers
modbus_alt = ModbusClient(host=alt_ip, port=alt_port, debug=False)
modbus_az = ModbusClient(host=az_ip, port=az_port, debug=False)


try:
    #connecting to the Redis server
    redis_client = redis.Redis(host=redis_ip, port=redis_port)

    #connecting to the Trio-Controller
    lb.Connect_to_Controller(modbus_alt, "Elevation")
    lb.Connect_to_Controller(modbus_az, "Azimuth")
    observing_location = EarthLocation(lat=telescope_lat, lon=telescope_lon, height=telescope_height * u.m)

    # while the Controller is running
    while True:

        # read the time and positions written on controller memory
        pos_time_alt = lb.Read_32_bit_floats(modbus_alt, altdef.master_time_to_ws, 3)
        pos_time_az = lb.Read_32_bit_floats(modbus_az, azdef.master_time_to_ws, 3)

        #saving encoder positions
        alt_encoder=pos_time_alt[2]
        az_encoder=pos_time_az[2]

        #creating the frame of reference to study elongation from the sun
        tel_frame = AltAz(location=observing_location, obstime=Time(datetime.now()))
        tel_alt , tel_az = lb.Encoder_to_AltAZ(alt_encoder, az_encoder)

        #creating a variable where the telescope elevation is normalized to 90 -90 deg
        if tel_alt>90:
            tel_alt_norm=tel_alt-2*(tel_alt-90)
        elif tel_alt<-90:
            tel_alt_norm=tel_alt+2*(-90-tel_alt)
        else:
            tel_alt_norm=tel_alt

        tel_altaz = SkyCoord(tel_az, tel_alt_norm, unit="deg", frame=tel_frame)

        #getting sun position
        sun_altaz = get_sun(Time(datetime.now())).transform_to(tel_frame)

        #if the telescope is too close to the sun
        elongation = sun_altaz.separation(tel_altaz).deg
        if elongation<=elongation_from_sun:
            lb.Write_32_bit_floats(modbus_alt, altdef.fault_status, altdef.sun_elongation)
            lb.Write_32_bit_floats(modbus_az, azdef.fault_status, azdef.sun_elongation)

            #saying to the az triocontroller in which direction to move in order to increase elongation from the sun
            antisun_dir = np.sign(tel_altaz.az.deg - sun_altaz.az.deg)
            if np.abs(tel_altaz.az.deg - sun_altaz.az.deg) > 180:
                antisun_dir = antisun_dir * -1
            lb.Write_32_bit_floats(modbus_az, azdef.antisun_direction, antisun_dir)

        elif tel_alt > alt_max: #if the telescope is outside the upper safety limit
            lb.Write_32_bit_floats(modbus_alt, altdef.fault_status, altdef.ax0_pos_sw_limit)
            lb.Write_32_bit_floats(modbus_az, azdef.fault_status, azdef.other_axis)
        elif tel_alt < alt_min: #if the telescope is outside the lower safety limit
            lb.Write_32_bit_floats(modbus_alt, altdef.fault_status, altdef.ax0_rev_sw_limit)
            lb.Write_32_bit_floats(modbus_az, azdef.fault_status, azdef.other_axis)

        #checking the status of the two axis
        alt_status=lb.Read_32_bit_floats(modbus_alt, altdef.fault_status)[0]
        az_status=lb.Read_32_bit_floats(modbus_az, azdef.fault_status)[0]

        #redis_client.publish("debug","alt_status: {:d}, az_status: {:d}\r".format(int(alt_status), int(az_status)))
        #if one of the two axis is blocked bc of a fault, block also the other axis
        if alt_status==altdef.no_fault and az_status!=azdef.no_fault:
            lb.Write_32_bit_floats(modbus_alt, altdef.fault_status, altdef.other_axis)
            #lb.Wait_until_register(modbus_az, azdef.fault_status, azdef.no_fault, 2**12)
            #lb.Write_32_bit_floats(modbus_alt, altdef.fault_status, altdef.no_fault)
        elif alt_status!=altdef.no_fault and az_status==azdef.no_fault:
            lb.Write_32_bit_floats(modbus_az, azdef.fault_status, azdef.other_axis)
            #lb.Wait_until_register(modbus_alt, altdef.fault_status, altdef.no_fault, 2 ** 12)
            #lb.Write_32_bit_floats(modbus_az, azdef.fault_status, azdef.no_fault)


        if alt_status!=altdef.no_fault or az_status!=azdef.no_fault:
            fault_message = {'alt_status': alt_status, 'az_status': az_status}
            redis_client.publish(redis_fault_alert_channel, msgpack.packb(fault_message))
            redis_client.set(current_motion_var, "recovery")
            lb.Fault_recovery(modbus_az, modbus_alt, alt_status, az_status, redis_client, redis_recovery_channel)
            redis_client.set(current_motion_var, "none")
        #increment the safety flags to a high value to signal that the workstation is still connected to TrioControllers

        #CHIAMARLO SAFETY COUNTERS
        lb.Write_32_bit_floats(modbus_alt, altdef.workstation_safety_flag, 1000)
        lb.Write_32_bit_floats(modbus_az, azdef.workstation_safety_flag, 1000)

        #print("Controlling Telescope Pointing:\t Elongation "+str(elongation)+"\t Alt "+str(tel_altaz.alt.deg)+"\t Az "+str(tel_altaz.az.deg), end="\r")

    lb.Disconnect_Controller(modbus_alt, "Elevation")
    lb.Disconnect_Controller(modbus_az, "Azimuth")

except KeyboardInterrupt:
    lb.Disconnect_Controller(modbus_alt, "Elevation")
    lb.Disconnect_Controller(modbus_az, "Azimuth")