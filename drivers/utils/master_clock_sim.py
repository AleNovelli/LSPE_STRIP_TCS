from pyModbusTCP.client import ModbusClient
import library as lb
import numpy as np
import time
import timeit
from datetime import datetime

import controller_definitions.azimuth_definitions as azdef #importing the I/O and VR allocations of Azimuth Controller
import controller_definitions.elevation_definitions as altdef #importing the I/O and VR allocations of Elevation Controller
import json

driver_params=json.load(open("../configuration/TCS_driver_parameters.json"))

#client=ModbusClient(host="127.0.0.1", port=502, debug=False)

az_ip=driver_params["ip&ports"]["az_proxy_ip"]
az_port=driver_params["ip&ports"]["az_proxy_port"]
alt_ip=driver_params["ip&ports"]["alt_proxy_ip"]
alt_port=driver_params["ip&ports"]["alt_proxy_port"]

modbus_az = ModbusClient(host=az_ip, port=az_port, debug=False) #preparing modbus connection to Azimuth Controller
modbus_alt = ModbusClient(host=alt_ip, port=alt_port, debug=False) #preparing modbus connection to Elevation Controller

try:
    # connecting to the two controllers
    lb.Connect_to_Controller(modbus_az, "Azimuth")
    lb.Connect_to_Controller(modbus_alt, "Elevation")

    sec=np.zeros(1)

    t0 = time.time()+2
    conn_on=True

    while conn_on:
        conn_on = conn_on and lb.Write_32_bit_floats(modbus_az, azdef.master_time_from_mc,sec)
        conn_on = conn_on and lb.Write_32_bit_floats(modbus_alt, altdef.master_time_from_mc, sec)
        while(((sec + t0 ) - time.time())[0]>0):
            pass
        conn_on = conn_on and modbus_az.write_single_coil(azdef.in_user_tos, True)
        conn_on = conn_on and modbus_alt.write_single_coil(altdef.in_user_tos, True)
        #t_=timeit.timeit(lambda: "client.write_single_coil(3, True)")
        print(time.time()-t0)
        time.sleep(.25)
        conn_on = conn_on and modbus_az.write_single_coil(azdef.in_user_tos, False)
        conn_on = conn_on and modbus_alt.write_single_coil(altdef.in_user_tos, False)
        sec = sec + 1

    lb.Disconnect_Controller(modbus_az, "Azimuth")
    lb.Disconnect_Controller(modbus_alt, "Elevation")


except KeyboardInterrupt:
    lb.Disconnect_Controller(modbus_az, "Azimuth")
    lb.Disconnect_Controller(modbus_alt, "Elevation")
