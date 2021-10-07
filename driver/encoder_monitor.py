#**********************************************************************************************************
#PROJECT: LSPE-STRIP
#
#Run From: CLIENT
#Program: "encoder_monitor.py"
#
#Description:
#   This program is executed by any program which commands the telescope to execute a motion (main_sequence,
#   raster_scan, etc). It connects to the Trio-Controllers and monitors the pointing of the telescope over time.
#   The pointing of the telescope as a function of time during the motion is saved in
#   "/pointing/main_sequence_DATE_TIMESTAMP.dat".
#   This program also controls that the telescope doesn't point too close to the sun or the moon. If it does, it sends
#   the command to stop the telescope to the Trio-Controllers.
#
#**********************************************************************************************************

import controller_definitions.azimuth_definitions as azdef #importing the I/O and VR allocations of Azimuth Controller
from pyModbusTCP.client import ModbusClient #package to manage Modbus connections
import library as lb
import time
import json

driver_params=json.load(open("TCS_driver_parameters.json"))

az_ip=driver_params["general"]["ip_az_controller"]
az_port=driver_params["general"]["port_az_controller"]
alt_ip=driver_params["general"]["ip_alt_controller"]
alt_port=driver_params["general"]["port_alt_controller"]

software_timeout=driver_params["general"]["software_timeout"]

modbus_az = ModbusClient(host=az_ip, port=az_port, debug=False) #preparing modbus connection to Azimuth Controller
#modbus_alt = ModbusClient(host=alt_ip, port=alt_port, debug=False) #preparing modbus connection to Elevation Controller

try:
    print("Connecting to Azimuth Trio-Controller ...")
    if modbus_az.open() == True: #tring to open connection with azimuth controller
        print("Connected successfully !!!")
    else:
        print("Client could not connect to the given IP/port !!!")
        #exit() <--- This line is commented because I can't connect to two simulators simultaneously (to be changed!!)

    print("Connecting to Elevation Trio-Controller ...")
     #if modbus_alt.open() == True: #tring to open connection with azimuth controller
     #   print("Connected successfully !!!")
    #else:
     #  print("Client could not connect to the given IP/port !!!")
        #exit() <--- This line is commented because I can't connect to two simulators simultaneously (to be changed!!)

    #opening the file where i will save the telescope pointing and writing initial description
    point_file = open(r"pointing\simulation.dat",
                "w")  # encoder_positions_" + time.strftime("%Y_%b_%d__%H_%M_%S", time.gmtime()), "w+")
    point_file.write("#Axis Positions measured on: " + time.strftime("%Y %b %d %H:%M:%S", time.gmtime()) + "\n")
    point_file.write("#Master Time\tTicks\tAzimuth\n")

    #lb.Wait_until_register(modbus_az,azdef.system_status, azdef.running,30,0.02)
    # saving the time written on Controller memory on a temporary variable to check for time updates
    tmp = lb.Read_32_bit_floats(modbus_az, azdef.master_time_to_ws, 3)

    while lb.Read_32_bit_floats(modbus_az, azdef.system_status)[0]!=azdef.disabled:#while the Controller is running
        pos_time = lb.Read_32_bit_floats(modbus_az, azdef.master_time_to_ws, 3) #read the time written on controller memory
        #print("--------", pos_time)
        if pos_time[1]!= tmp[1]: #every time the ticks parameter is modified
            point_file.write(' '.join(map(str, pos_time)).replace(" ", "\t") + "\n")#Write time and pos on file
            print(' '.join(map(str, pos_time)).replace(" ", "\t"))
            tmp=pos_time #update temporary variable

    point_file.close()

    print("\nProgram ended!\nPress Crl-C to close")

    while True:
        pass

    print("\nEnd of program, disconnecting client ...")
    if modbus_az.close() == True: #disconnecting Azimuth Trio-Controller
        print("Azimuth disconnected correctly !!!")
    else:
        print("Error in disconnecting Azimuth !!!")

    #if modbus_alt.close() == True: #disconnecting Elevation Trio-Controller
    #    print("Elevation disconnected correctly !!!")
    #else:
    #print("Error in disconnecting Elevation !!!")

except KeyboardInterrupt:
    if modbus_az.close() == True:  # disconnecting Azimuth Trio-Controller
        print("Azimuth disconnected correctly !!!")
    else:
        print("Error in disconnecting Azimuth !!!")

    #if modbus_alt.close() == True:  # disconnecting Elevation Trio-Controller
    #    print("Elevation disconnected correctly !!!")
    #else:
    #    print("Error in disconnecting Elevation !!!")