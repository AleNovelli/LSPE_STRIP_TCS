#**********************************************************************************************************
#PROJECT: LSPE-STRIP
#
#Run From: CLIENT
#Program: "encoder_sampler_az.py"
#
#Description:
#   This program must be running at any time the telescope is not disabled.
#   It connects to the Azimuth Trio-Controller and permanent the pointing of the telescope over time.
#   The pointing of the telescope as a function of time during the motion is saved in
#   "/pointing/main_sequence_DATE_TIMESTAMP.dat".
#
#**********************************************************************************************************

import controller_definitions.azimuth_definitions as azdef #importing the I/O and VR allocations of Azimuth Controller
from pyModbusTCP.client import ModbusClient #package to manage Modbus connections
from utils import library as lb
import json
import numpy as np

import h5py

driver_params=json.load(open("configuration/TCS_driver_parameters.json"))

az_ip=driver_params["general"]["ip_az_proxy"]
az_port=driver_params["general"]["port_az_proxy"]

software_timeout=driver_params["general"]["software_timeout"]

#preparing modbus connection to Azimuth Controller
modbus_az = ModbusClient(host=az_ip, port=az_port, debug=False)

try:
    #connecting to the Trio-Controller
    lb.Connect_to_Controller(modbus_az, "Azimuth")

    hdf = h5py.File("pointing/azimuth.h5", 'w')

    group=hdf.require_group("azimuth")

    if not "time" in group.keys():
        group.create_dataset("master_time", data=np.array([]),dtype=np.int32, maxshape=(None,), chunks=True)
    if not "ticks" in group.keys():
        group.create_dataset("ticks", data=np.array([]), dtype=np.int32, maxshape=(None,), chunks=True)
    if not "pos" in group.keys():
        group.create_dataset("encoder", data=np.array([]), dtype=np.int32, maxshape=(None,), chunks=True)


    # saving the time written on Controller memory on a temporary variable to check for time updates
    tmp = lb.Read_32_bit_floats(modbus_az, azdef.master_time_to_ws, 4)
    buffer=[]

    # while the Controller is running
    while True: #lb.Read_32_bit_floats(modbus_az, azdef.system_status)[0]!=azdef.disabled:
        # read the time and positions written on controller memory
        pos_time = lb.Read_32_bit_floats(modbus_az, azdef.master_time_to_ws, 4)

        # every time the ticks parameter is modified
        if pos_time[1]!= tmp[1]:
            # Write time and pos on file
            buffer.append(pos_time)

            #hdlu.append(np.array([pos_time]))

            print(pos_time[0], pos_time[3]-pos_time[1], pos_time[2])
            # update temporary variable
            tmp=pos_time

            if int(pos_time[0])%3==0 and int(pos_time[3]-pos_time[1])==0:
                buffer = np.array(buffer).T
                add_lenght = buffer.shape[1]

                group["master_time"].resize((group["master_time"].shape[0] + add_lenght), axis=0)
                group["master_time"][-add_lenght:] = buffer[0]

                group["ticks"].resize((group["ticks"].shape[0] + add_lenght), axis=0)
                group["ticks"][-add_lenght:] = buffer[3]-buffer[1]

                group["encoder"].resize((group["encoder"].shape[0] + add_lenght), axis=0)
                group["encoder"][-add_lenght:] = buffer[2]

                hdf.flush()
                buffer=[]


    lb.Disconnect_Controller(modbus_az, "Azimuth")

except KeyboardInterrupt:
    hdf.close()
    lb.Disconnect_Controller(modbus_az, "Azimuth")