
import controller_definitions.azimuth_definitions as azdef #importing the I/O and VR allocations of Azimuth Controller
import controller_definitions.elevation_definitions as altdef #importing the I/O and VR allocations of Elevation Controller
from pyModbusTCP.client import ModbusClient #package to manage Modbus connections

from utils import library as lb

import time
import json

driver_params=json.load(open("./configuration/TCS_driver_parameters.json"))

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

    lb.Write_32_bit_floats(modbus_az, azdef.fault_acknowledge, 1)
    lb.Write_32_bit_floats(modbus_alt, altdef.fault_acknowledge, 1)

    #time.sleep(1)

    #lb.Write_32_bit_floats(modbus_az, azdef.fault_acknowledge, 0)
    #lb.Write_32_bit_floats(modbus_alt, altdef.fault_acknowledge, 0)

    lb.Disconnect_Controller(modbus_az, "Azimuth")
    lb.Disconnect_Controller(modbus_alt, "Elevation")

except KeyboardInterrupt:
    lb.Disconnect_Controller(modbus_az, "Azimuth")
    lb.Disconnect_Controller(modbus_alt, "Elevation")