#**********************************************************************************************************
#PROJECT: LSPE-STRIP
#
#Run From: CLIENT
#Program: "library.py"
#
#Description:
#   This is a library containing all the most used commands to interact with the Trio-Controllers of the LSPE-SWIPE
#   experiment. It is included in several programs which move and communicate with the controllers
#
#**********************************************************************************************************

from pyModbusTCP import utils
import controller_definitions.azimuth_definitions as azdef #importing the I/O and VR allocations of Azimuth Controller
import controller_definitions.elevation_definitions as altdef #importing the I/O and VR allocations of Elevation Controller
import numpy as np
import time
import msgpack

#<--------------------------------------------------------------------------->
#These functions are used to write values on the memory of the Trio-Controllers (both VR and Table memory) using
#the modbus protocol
def Write_16_bit_ints(client, address, values):
    if type(values) != list:
        values=[values]
    #putting the values in the format which is interpreted by the TrioController
    values_signed = [value + 2 ** 16 if value < 0 else value for value in values]
    #dividing the values into chunks of size 100 (in order not to reach Modbus limits)
    chunks = [values_signed[x:x + 100] for x in range(0, len(values_signed), 100)]
    shift_addr=0 #the variable used to control the fact that the chunks must be written shifted by 100 one to the other
    outcome=True #describes if the function has connected and written succesfully
    for chunk in chunks:
        #writing the chunks and controlling if they have all been written correctly
        outcome = outcome and client.write_multiple_registers(address+shift_addr, chunk)
        shift_addr=shift_addr+100
    return outcome

def Read_16_bit_ints(client, address, number=1):
    reg_l = [] #initializing a list where to store the variables read from the registers
    for shift_addr in range(0, number, 100): #reading the addresses in chunks of 100 because of modbus limit
        reg=client.read_holding_registers(address+shift_addr, min(100, number-shift_addr))
        if reg == None:#if reading fails exit the program
            raise Exception("MODBUS: Reading values from Trio-Controller FAILED!!!")
        reg_l = reg_l + reg
        shift_addr = shift_addr + 100
    if reg_l != []:
        values_signed = [value - 2**16 if value>2**16/2 else value for value in reg_l]
        return values_signed
    else:
        return None

def Write_32_bit_ints(client, address, values):
    if type(values) != list:
        values=[values]
    # putting the values in the format which is interpreted by the TrioController
    values_signed=[value + 2 ** 32 if value < 0 else value for value in values]
    b16_l = utils.long_list_to_word(values_signed, big_endian=False)
    # dividing the values into chunks of size 100 (in order not to reach Modbus limits)
    chunks = [b16_l[x:x + 100] for x in range(0, len(b16_l), 100)]
    shift_addr = 0  # the variable used to control the fact that the chunks must be written shifted by 100 one to the other
    outcome = True  # describes if the function has connected and written succesfully
    for chunk in chunks:
        # writing the chunks and controlling if they have all been written correctly
        outcome = outcome and client.write_multiple_registers(address + shift_addr, chunk)
        shift_addr = shift_addr + 100
    return outcome


def Read_32_bit_ints(client, address, number=1):
    reg_l = [] #initializing a list where to store the variables read from the registers
    for shift_addr in range(0, number*2, 100): #reading the addresses in chunks of 100 because of modbus limit
        reg = client.read_holding_registers(address + shift_addr, min(100, number*2 - shift_addr))
        if reg == None:#if reading fails exit the program
            raise Exception("MODBUS: Reading values from Trio-Controller FAILED!!!")
        reg_l = reg_l + reg
        #shift_addr = shift_addr + 100
    if reg_l != []:
        values_unsigned=utils.word_list_to_long(reg_l, big_endian=False)
        values_signed = [ value - 2**32 if value>2**16/2 else value for value in values_unsigned]
        return values_signed
    else:
        return None

def Write_32_bit_floats(client, address, values):
    if type(values) != list:
        values=[values]
    #splitting 32 bit values in two 16 bits registers
    b32_l = [utils.encode_ieee(f) for f in values]
    b16_l = utils.long_list_to_word(b32_l, big_endian=False)
    # dividing the values into chunks of size 100 (in order not to reach Modbus limits)
    chunks = [b16_l[x:x + 100] for x in range(0, len(b16_l), 100)]
    shift_addr = 0  # the variable used to control the fact that the chunks must be written shifted by 100 one to the other
    outcome = True  # describes if the function has connected and written succesfully
    for chunk in chunks:
        # writing the chunks and controlling if they have all been written correctly
        outcome = outcome and client.write_multiple_registers(address + shift_addr, chunk)
        shift_addr = shift_addr + 100
    return outcome


def Read_32_bit_floats(client, address, number=1):
    reg_l = [] #initializing a list where to store the variables read from the registers
    for shift_addr in range(0, number * 2, 100): #reading the addresses in chunks of 100 because of modbus limit
        reg = client.read_holding_registers(address + shift_addr, min(100, number * 2 - shift_addr))
        if reg == None:#if reading fails exit the program
            raise Exception("MODBUS: Reading values from Trio-Controller FAILED!!!")
        reg_l = reg_l + reg
        #shift_addr = shift_addr + 100
    if reg_l !=[]:
        return [utils.decode_ieee(f) for f in utils.word_list_to_long(reg_l, big_endian=False)]
    else:
        return None


def Write_64_bit_floats(client, address, values):
    if type(values) != list:
        values=[values]
    #splitting 64 bit values in four 16 bits registers
    b32_l = [utils.encode_ieee(f, double=True) for f in values]
    b16_l = utils.long_list_to_word(b32_l, big_endian=False, long_long=True)
    # dividing the values into chunks of size 100 (in order not to reach Modbus limits)
    chunks = [b16_l[x:x + 100] for x in range(0, len(b16_l), 100)]
    shift_addr = 0  # the variable used to control the fact that the chunks must be written shifted by 100 one to the other
    outcome = True  # describes if the function has connected and written succesfully
    for chunk in chunks:
        # writing the chunks and controlling if they have all been written correctly
        outcome = outcome and client.write_multiple_registers(address + shift_addr, chunk)
        shift_addr = shift_addr + 100
    return outcome


def Read_64_bit_floats(client, address, number=1):
    reg_l = [] #initializing a list where to store the variables read from the registers
    for shift_addr in range(0, number * 4, 100): #reading the addresses in chunks of 100 because of modbus limit
        reg = client.read_holding_registers(address + shift_addr, min(100, number * 4 - shift_addr))
        if reg == None: #if reading fails exit the program
            raise Exception("MODBUS: Reading values from Trio-Controller FAILED!!!")
        reg_l = reg_l + reg
        #shift_addr = shift_addr + 100
    if reg_l !=[]:
        return [utils.decode_ieee(f, double=True) for f in utils.word_list_to_long(reg_l, big_endian=False, long_long=True)]
    else:
        return None

#<--------------------------------------------------------------------------->

#This function forces the program to wait until a given input is turned on the Trio-Controller
def Wait_until_coils(client, coils, timeout, period=0.25):
    mustend = time.time() + timeout #calculating when we the function will timeout
    if coils>7:
        while time.time() < mustend:
            if all(client.read_coils(coils)): return True
            time.sleep(period)
    else:
        while time.time() < mustend:
            if all(client.read_discrete_inputs(coils)): return True
            time.sleep(period)
    raise Exception("WAIT UNTIL COILS: Waiting for coils timed out!!!\nCoils:"+str(coils))

#<--------------------------------------------------------------------------->

#This function forces the program to wait until a given register has a given value
def Wait_until_register(client, address, value, timeout, period=0.25, number=1):
    if type(value) != list:
        value=[value]
    mustend = time.time() + timeout #calculating when we the function will timeout
    while time.time() < mustend:
        #print(Read_32_bit_floats(client, address, number), value)
        if Read_32_bit_floats(client, address, number) == value: return True
        time.sleep(period)
    raise Exception("WAIT UNTIL REGISTER: Waiting for register value timed out!!!\nAddress:"+str(address)+"\tValue:"+str(value))

#<--------------------------------------------------------------------------->

#This function reads the parameter files and returns them as a numpy array.
#It will automatically ignore comments
def Read_Parameters(path):
    #opening and reading the parameter file
    file = open(path, "r")
    lines = file.readlines()
    file.close()

    #preparing a list were to store the configuration of each line of the file
    lines_clean = []

    for i in range(len(lines)): #iterating over the lines of the files
        if lines[i][0] != "#":#ignoring comments
            lines_clean.append(lines[i].replace("\n", "").split("\t")) #extracting the values of each line
    lines_clean = np.transpose(np.array(lines_clean)) #transposing the array
    return lines_clean

#<--------------------------------------------------------------------------->

#this function tries to connect to a TrioController
def Connect_to_Controller(controller, name=""):
    print("Connecting to "+name+" Trio-Controller ...")
    if controller.open() == True:  # tring to open connection withcontroller
        print("Connected successfully !!!")
    else:
        print("Client could not connect to the given IP/port !!!")
        exit()

#<--------------------------------------------------------------------------->

#this function disconnects from the TrioController
def Disconnect_Controller(controller, name=""):
    print("\nDisconnecting client ...")
    controller.close()  # disconnecting Azimuth Trio-Controller
    print(name+" Disconnected!!!")

#<--------------------------------------------------------------------------->

#this function checks if a TrioController is available for motion
def Test_Controller_Available(controller, alt_az):
    if alt_az=="az":
        # control if the Azimuth controller can be used for motion
        status = Read_32_bit_floats(controller, azdef.system_status)[0]
        if status != azdef.disabled and status != azdef.ready:
            raise Exception("The Azimuth Tio-Controller was not ready for motion!!!\nController status:" + str(status))

    elif alt_az=="alt":
        # control if the Elevation Controller can be used for motion
        status = Read_32_bit_floats(controller, altdef.system_status)[0]
        if status != altdef.disabled and status != altdef.ready:
            raise Exception(
                "The Elevation Tio-Controller was not ready for motion!!!\nController status:" + str(status))
    else:
        raise Exception(
            "When checking for controller availability you must specify if you are connecting to 'alt' or 'az' controller")

    return status

#<--------------------------------------------------------------------------->

#This function enables motion on the TrioController
def Enable_Motion(controller, alt_az, status, timeout):

    if alt_az == "az":
        # if the Azimuth Controller is disabled enable all axis
        if status == azdef.disabled:
            print("Enabling Azimut Axis")
            if Read_32_bit_floats(controller, azdef.motion_command)[0] == azdef.disable_all:
                Write_32_bit_floats(controller, azdef.motion_command, azdef.await_all)
                time.sleep(.1)#waiting for the controllers to register the command changed from disable all
            Write_32_bit_floats(controller, azdef.motion_command, azdef.enable_all)

        # Waiting for the Trio-Controller to be enabled and ready for a motion command
        Wait_until_register(controller, azdef.system_status, azdef.ready, timeout)

    elif alt_az == "alt":
        # if the Azimuth Controller is disabled enable all axis
        if status == altdef.disabled:
            print("Enabling Elevation Axis")
            if Read_32_bit_floats(controller, altdef.motion_command)[0] == altdef.disable_all:
                Write_32_bit_floats(controller, altdef.motion_command, altdef.await_all)
                time.sleep(.1)  # waiting for the controllers to register the command changed from disable all
            Write_32_bit_floats(controller, altdef.motion_command, altdef.enable_all)

        # Waiting for the Trio-Controller to be enabled and ready for a motion command
        Wait_until_register(controller, altdef.system_status, altdef.ready, timeout)

    else:
        raise Exception(
            "When enabling axis you must specify if you are connecting to 'alt' or 'az' controller")

# <--------------------------------------------------------------------------->

#This function calculates the trajectory to a given position avoiding the sun
# The trajectory is given as a series of az-alt coordinates. Since the two axis are independent between successive
# points of the trajectory only one of between alt or az is modified. the telescope will move in straigh lines from a
# point to the next.
def Traj_to_Pos(sun, start, finish, elongation, alt_max, alt_min):

    from astropy.coordinates import SkyCoord, Angle

    traj=start
    obj=finish
    # this function appends a point to the telescope trajectory adding Alt Az to the current position of the telescope
    # (by using reverse=True the point is added to the start of the trajectory instead of the end)
    def move(traj_, az, alt, reverse=False):

        if reverse == False:
            #last_enc_alt, last_enc_az = AltAz_to_Encoder(traj_.alt[-1], traj_.az[-1])
            #next_enc_alt, next_enc_az = AltAz_to_Encoder(alt, az)
            if True: #not (abs(next_enc_az - last_enc_az) < 10 and abs(next_enc_alt - last_enc_alt) < 10):
                if az.shape == ():
                    full_az = np.concatenate((traj_.az.deg, np.array([az.deg + traj_[-1].az.deg])))
                else:
                    full_az = np.concatenate((traj_.az.deg, az.deg + traj_[-1].az.deg))

                if alt.shape == ():
                    full_alt = np.concatenate((traj_.alt.deg, np.array([alt.deg + traj_[-1].alt.deg])))
                else:
                    full_alt = np.concatenate((traj_.alt.deg, alt.deg + traj_[-1].alt.deg))
        else:
            #last_enc_alt, last_enc_az = AltAz_to_Encoder(traj_.alt[0], traj_.az[0])
            #next_enc_alt, next_enc_az = AltAz_to_Encoder(alt, az)
            if True: #not (abs(next_enc_az - last_enc_az) < 10 and abs(next_enc_alt - last_enc_alt) < 10):
                if az.shape == ():
                    full_az = np.concatenate((np.array([az.deg + traj_[0].az.deg]), traj_.az.deg))
                else:
                    full_az = np.concatenate((az.deg + traj_[0].az.deg, traj_.az.deg))

                if alt.shape == ():
                    full_alt = np.concatenate((np.array([alt.deg + traj_[0].alt.deg]), traj_.alt.deg))
                else:
                    full_alt = np.concatenate((alt.deg + traj_[0].alt.deg, traj_.alt.deg))

        traj_ = SkyCoord(full_az, full_alt, unit="deg", frame="altaz")
        traj_.az.wrap_at("180d", inplace=True)
        return traj_

    # this function appends a point (Az Alt) to the telescope trajectory
    def moveabs(traj_, new_az, new_alt):

        #print(AltAz_to_Encoder(traj_.alt[-1], traj_.az[-1]), AltAz_to_Encoder(new_alt, new_az))
        #last_enc_alt, last_enc_az = AltAz_to_Encoder(traj_.alt[-1], traj_.az[-1])
        #next_enc_alt, next_enc_az = AltAz_to_Encoder(new_alt, new_az)
        if True: #not( abs(next_enc_az-last_enc_az)<10 and abs(next_enc_alt-last_enc_alt)<10):
            if new_az.shape == ():
                full_az = np.concatenate((traj_.az.deg, np.array([new_az.deg])))
            else:
                full_az = np.concatenate((traj_.az.deg, new_az.deg))

            if new_alt.shape == ():
                full_alt = np.concatenate((traj_.alt.deg, np.array([new_alt.deg])))
            else:
                full_alt = np.concatenate((traj_.alt.deg, new_alt.deg))
            traj_ = SkyCoord(full_az, full_alt, unit="deg", frame="altaz")
        return traj_

    # calculating the points which the telescope must not cross (to keep elongation from the sun)
    ang = range(0, 359, 1)
    sun_lim = sun.directional_offset_by(Angle(ang, unit="degree"), elongation)
    sun_lim.az.wrap_at("180d", inplace=True)

    # turning these points into Alt Az limits
    sun_az_min = ((sun_lim.az - sun.az).wrap_at("180d").min() + sun.az).wrap_at("180d")
    sun_az_max = ((sun_lim.az - sun.az).wrap_at("180d").max() + sun.az).wrap_at("180d")
    sun_alt_min = sun.alt - elongation
    sun_alt_max = sun.alt + elongation
    # and calculating the horizontal and vertical radius of the sun in Alt AZ space
    hor_rad = (sun_az_max - sun_az_min).wrap_at("180d") / 2
    ver_rad = elongation

    if sun_az_min < sun_az_max:  # case in which the sun does not coincide with the Az periodic limit (180 to -180)
        print(-1)
        traj_az_in_sun = sun_az_min < traj.az < sun_az_max  # check if the starting point is in the Az range occupied by the sun
        obj_az_in_sun = sun_az_min < obj.az < sun_az_max  # check if the final point is in the Az range occupied by the sun
    else:  # case in which the sun coincides with the Az periodic limit (180 to -180)
        print(-2)
        traj_az_in_sun = not sun_az_max < traj.az < sun_az_min  # check if the starting point is in the Az range occupied by the sun
        obj_az_in_sun = not sun_az_max < obj.az < sun_az_min  # check if the final point is in the Az range occupied by the sun

    traj_alt_in_sun = sun_alt_min < traj.alt < sun_alt_max  # check if the starting point is in the Alt range occupied by the sun
    obj_alt_in_sun = sun_alt_min < obj.alt < sun_alt_max  # check if the final point is in the Alt range occupied by the sun

    if traj_az_in_sun and traj_alt_in_sun:  # if telescope is close to the sun program a move to move it away
        print(1)
        traj = move(traj, 1.2*np.sign((traj[-1].az - sun.az).wrap_at("180d")) * (
                    hor_rad - abs((traj[-1].az - sun.az).wrap_at("180d"))), Angle(0, unit="deg"))
        traj_az_in_sun = False  # I moved the telescope outside the sun Az range

    if obj_az_in_sun and obj_alt_in_sun:  # if final pos is close to the sun program a move to get close safely
        print(2)
        obj = move(obj, 1.2*np.sign((obj[0].az - sun.az).wrap_at("180d")) * (
                    hor_rad - abs((obj[0].az - sun.az).wrap_at("180d"))), Angle(0, unit="deg"), reverse=True)
        obj_az_in_sun = False  # I moved the objective outside the sun Az range

    # calculating the relative position of the telescope, sun and final pos
    traj_to_sun = (sun.az - traj[-1].az).wrap_at("180d")
    traj_to_obj = (obj[0].az - traj[-1].az).wrap_at("180d")
    # checking if the sun is in the way of the telescope while moving straight to the final pos
    sun_in_the_way = min(0, traj_to_obj) - hor_rad < traj_to_sun < max(0, traj_to_obj) + hor_rad

    if not sun_in_the_way:
        print(3)  # if the sun is not in the way move straight to the objective
        if traj[-1].az != obj[0].az:
            traj = moveabs(traj, obj[0].az, traj[-1].alt)
        if traj[-1].alt != obj[0].alt:
            traj = moveabs(traj, traj[-1].az, obj[0].alt)
    elif traj_az_in_sun and obj_az_in_sun:
        if abs((traj[-1].az - obj[0].az) / 2 + obj[0].az - sun_az_min) >= abs(
                (traj[-1].az - obj[0].az) / 2 + obj[0].az - sun_az_max):
            print(4.1)

            traj = moveabs(traj, sun_az_max, traj[-1].alt)
            traj = moveabs(traj, traj[-1].az, obj[0].alt)
            traj = moveabs(traj, obj[0].az, traj[-1].alt)
        else:
            print(4.2)
            traj = moveabs(traj, sun_az_min, traj[-1].alt)
            traj = moveabs(traj, traj[-1].az, obj[0].alt)
            traj = moveabs(traj, obj[0].az, traj[-1].alt)
    elif not traj_alt_in_sun:
        print(5)
        traj = moveabs(traj, obj[0].az, traj[-1].alt)
        traj = moveabs(traj, traj[-1].az, obj[0].alt)
    elif not obj_alt_in_sun:
        print(6)
        traj = moveabs(traj, traj[-1].az, obj[0].alt)
        traj = moveabs(traj, obj[0].az, traj[-1].alt)
    else:
        if sun.alt.deg > (alt_max - alt_min) / 2 + alt_min:
            print(7)
            traj = moveabs(traj, traj[-1].az, sun_alt_min)
            traj = moveabs(traj, obj[0].az, traj[-1].alt)
            traj = moveabs(traj, traj[-1].az, obj[0].alt)
        else:
            print(8)
            traj = moveabs(traj, traj[-1].az, sun_alt_max)
            traj = moveabs(traj, obj[0].az, traj[-1].alt)
            traj = moveabs(traj, traj[-1].az, obj[0].alt)

    if len(obj) > 1:
        full_traj = SkyCoord(np.concatenate([traj.az.deg, obj[1:].az.deg]),
                             np.concatenate([traj.alt.deg, obj[1:].alt.deg]), unit="deg", frame="altaz")
    else:
        full_traj = traj
    full_traj.az.wrap_at("180d", inplace=True)

    print(full_traj.az.deg, full_traj.alt.deg)

    az_traj = full_traj.az.deg.tolist()  # converting the trajectory from numpy strings to list of floats
    alt_traj = full_traj.alt.deg.tolist()
    return az_traj, alt_traj

# <--------------------------------------------------------------------------->

#This function convert encoder coordinates into alt and az coordinates
def Encoder_to_AltAZ(alt_encoder, az_encoder):

    import json
    driver_params = json.load(open("../configuration/TCS_driver_parameters.json"))
    alt_enc_at_0=driver_params["encoder_pos_at_zero_deg"]["alt"]
    az_enc_at_0 = driver_params["encoder_pos_at_zero_deg"]["az"]

    alt = (alt_encoder-alt_enc_at_0)*360./(2**32-1)
    az = (az_encoder-az_enc_at_0)*360./(2**32-1)
    return alt, az

#This function convets alt az  coordinates in encoder coordinates
# DEPRECATED!!!!
def AltAz_to_Encoder(alt, az):

    alt_encoder=alt.deg*(2**26-1)/360
    az_encoder=az.deg*(2**29-1)/360

    if type(alt_encoder)==float:
        alt_encoder = int(alt_encoder)
    elif type(alt_encoder)==np.ndarray or type(alt_encoder)==np.float64:
        alt_encoder = alt_encoder.astype(int)

    if type(az_encoder)==float:
        az_encoder = int(az_encoder)
    elif type(az_encoder) == np.ndarray or type(az_encoder)==np.float64:
        az_encoder = az_encoder.astype(int)

    return alt_encoder, az_encoder

# <--------------------------------------------------------------------------->

def Fault_recovery(modbus_az, modbus_alt, alt_fault, az_fault, redis_client, redis_recovery_channel):

    az_recovered=False
    alt_recovered=False

    #recovery of az axis fault
    if az_fault==azdef.estop:
        redis_client.publish(redis_recovery_channel, msgpack.packb("Az ESTOP pressed\nrelease ESTOP + press CONTROL"))
        Wait_until_coils(modbus_az, azdef.in_estop_healthy, 24 * 3600, period=.1)
        az_recovered = True
        #while az_recovered==False:
        #    Wait_until_coils(modbus_az, azdef.in_start_button, 24*3600, period=.1)
        #    if modbus_az.read_discrete_inputs(azdef.in_estop_healthy)[0]:
        #        az_recovered=True

    #CANIO is not used
    #elif az_fault==azdef.canio_fault:
    #    redis_client.publish(redis_recovery_channel, msgpack.packb("Az CANBUS disconnected\nIf CANBUS should be connected press START"))
    #    Wait_until_coils(modbus_az, azdef.in_start_button, 24 * 3600, period=.1)
    #    az_recovered=True

    elif az_fault==azdef.ax0_drv_comms_fault:
        redis_client.publish(redis_recovery_channel, msgpack.packb("Az error in Ethercat communications\nWhen ready press START"))
        Wait_until_coils(modbus_az, azdef.in_start_button, 24 * 3600, period=.1)
        az_recovered=True

    elif az_fault==azdef.ax0_drive_fault:
        redis_client.publish(redis_recovery_channel,
                             msgpack.packb("Az remote servo drive error\nWhen ready press START"))
        Wait_until_coils(modbus_az, azdef.in_start_button, 24 * 3600, period=.1)
        az_recovered = True

    elif az_fault == azdef.ax0_fe_fault:
        redis_client.publish(redis_recovery_channel,
                             msgpack.packb("Az following error is too large\nWhen ready press START"))
        Wait_until_coils(modbus_az, azdef.in_start_button, 24 * 3600, period=.1)
        az_recovered = True

    elif az_fault == azdef.encoder_samplig_fault:
        redis_client.publish(redis_recovery_channel,
                             msgpack.packb("Az encoder sampling freq is too high\nChange sampling frequency in the firmware and restart the TrioControllers"))
        Wait_until_coils(modbus_az, azdef.in_start_button, 24 * 3600, period=.1)
        az_recovered = True

    elif az_fault == azdef.workstation:
        redis_client.publish(redis_recovery_channel,
                             msgpack.packb("Az axis lost connection to the workstation\nRestore the connection and press START.py"))
        Wait_until_coils(modbus_az, azdef.in_start_button, 24 * 3600, period=.1)
        Write_32_bit_floats(modbus_az, azdef.workstation_safety_flag, 3000)
        az_recovered = True

    elif az_fault == azdef.master_clock:
        print("STUCK HERE")
        redis_client.publish(redis_recovery_channel,
                             msgpack.packb("Az axis lost connection to the master clock\nRestore the connection and press START.py"))
        Wait_until_coils(modbus_az, azdef.in_start_button, 24 * 3600, period=.1)
        print("Solved")
        az_recovered = True

    elif az_fault == azdef.basic_error:
        err_code=Read_32_bit_floats(modbus_az, azdef.err_code_no)
        err_line = Read_32_bit_floats(modbus_az, azdef.err_line_no)
        err_proc = Read_32_bit_floats(modbus_az, azdef.err_proc_no)
        redis_client.publish(redis_recovery_channel,
                             msgpack.packb("Az Trio-Controller crashed - Firmware error: {:f} in process: {:f} line: {:f}\nCheck the cause of the error and reboot the controller".format(err_code, err_proc, err_line)))
        Wait_until_coils(modbus_az, azdef.in_start_button, 24 * 3600, period=.1)
        az_recovered = True




    #Recovery of elevation axis faults
    if alt_fault==altdef.estop:
        redis_client.publish(redis_recovery_channel, msgpack.packb("Alt ESTOP pressed\nrelease ESTOP + press CONTROL"))
        Wait_until_coils(modbus_alt, altdef.in_estop_healthy, 24*3600, period=.1)
        alt_recovered = True
        #while alt_recovered==False:
            #Wait_until_coils(modbus_az, azdef.in_start_button, 24*3600, period=.1)
            #if modbus_alt.read_discrete_inputs(altdef.in_estop_healthy)[0]:
            #    alt_recovered=True

    #CANIO is not used
    #elif alt_fault==altdef.canio_fault:
    #    redis_client.publish(redis_recovery_channel, msgpack.packb("Alt CANBUS disconnected\nIf CANBUS should be connected press START"))
    #    Wait_until_coils(modbus_az, azdef.in_start_button, 24 * 3600, period=.1)
    #    alt_recovered=True

    elif alt_fault == altdef.ax0_drv_comms_fault:
        redis_client.publish(redis_recovery_channel,msgpack.packb("Alt error in Ethercat communications\nWhen ready press START"))
        Wait_until_coils(modbus_az, azdef.in_start_button, 24 * 3600, period=.1)
        alt_recovered = True

    elif alt_fault == altdef.ax0_drv_comms_fault:
        redis_client.publish(redis_recovery_channel,msgpack.packb("Alt remote servo drive error\nWhen ready press START"))
        Wait_until_coils(modbus_az, azdef.in_start_button, 24 * 3600, period=.1)
        alt_recovered = True

    elif alt_fault == altdef.ax0_fe_fault:
        redis_client.publish(redis_recovery_channel,msgpack.packb("Alt following error is too large\nWhen ready press START"))
        Wait_until_coils(modbus_az, azdef.in_start_button, 24 * 3600, period=.1)
        alt_recovered = True

    elif alt_fault == altdef.encoder_samplig_fault:
        redis_client.publish(redis_recovery_channel,
                             msgpack.packb("Alt encoder sampling freq is too high\nChange sampling frequency in the firmware and restart the TrioControllers"))
        Wait_until_coils(modbus_az, azdef.in_start_button, 24 * 3600, period=.1)
        alt_recovered = True

    elif alt_fault == altdef.workstation:
        redis_client.publish(redis_recovery_channel,
                             msgpack.packb("Alt axis lost connection to the workstation\nRestore the connection and press START.py"))
        Wait_until_coils(modbus_az, azdef.in_start_button, 24 * 3600, period=.1)
        Write_32_bit_floats(modbus_alt, altdef.workstation_safety_flag, 3000)
        alt_recovered = True

    elif alt_fault == altdef.master_clock:
        redis_client.publish(redis_recovery_channel,
                             msgpack.packb("Alt axis lost connection to the master clock\nRestore the connection and press START.py"))
        Wait_until_coils(modbus_az, azdef.in_start_button, 24 * 3600, period=.1)
        alt_recovered = True

    elif alt_fault == altdef.ax0_pos_hw_limit or alt_fault == altdef.ax0_rev_hw_limit:
        redis_client.publish(redis_recovery_channel,
                             msgpack.packb("Alt axis lost connection to the master clock\nRestore the connection and press START.py"))
        Wait_until_coils(modbus_az, azdef.in_start_button, 24 * 3600, period=.1)
        alt_recovered = True

    elif alt_fault == altdef.ax0_pos_sw_limit or alt_fault == altdef.ax0_rev_sw_limit:
        redis_client.publish(redis_recovery_channel,
                             msgpack.packb("Alt axis lost connection to the master clock\nRestore the connection and press START.py"))
        Wait_until_coils(modbus_az, azdef.in_start_button, 24 * 3600, period=.1)
        alt_recovered = True

    elif alt_fault == altdef.basic_error:
        err_code=Read_32_bit_floats(modbus_alt, altdef.err_code_no)
        err_line = Read_32_bit_floats(modbus_alt, altdef.err_line_no)
        err_proc = Read_32_bit_floats(modbus_alt, altdef.err_proc_no)
        redis_client.publish(redis_recovery_channel,
                             msgpack.packb("Alt Trio-Controller crashed - Firmware error: {:f} in process: {:f} line: {:f}\nCheck the cause of the error and reboot the controller".format(err_code, err_proc, err_line)))
        Wait_until_coils(modbus_az, azdef.in_start_button, 24 * 3600, period=.1)
        alt_recovered = True


    if az_fault==azdef.other_axis or az_fault==azdef.no_fault:
        if alt_recovered:
            az_recovered=True
    if alt_fault==altdef.other_axis or az_fault==azdef.no_fault:
        if az_recovered:
            alt_recovered=True

    if az_fault==azdef.sun_elongation and alt_fault==altdef.sun_elongation:
        Wait_until_coils(modbus_az, azdef.io_user_azimuth_idle, 10*60, period=.1)
        Wait_until_coils(modbus_alt, altdef.io_user_elevation_idle, 10*60 * 3600, period=.1)
        modbus_az.write_single_coil(azdef.io_user_azimuth_idle, False)
        modbus_alt.write_single_coil(altdef.io_user_elevation_idle, False)
        Sun_recovery(modbus_az, modbus_alt)
        Wait_until_coils(modbus_az, azdef.in_start_button, 24 * 3600, period=.1)
        az_recovered=True
        alt_recovered=True

    if az_fault==azdef.other_axis and alt_fault==altdef.other_axis:
        Write_32_bit_floats(modbus_az, azdef.fault_status, azdef.no_fault)
        Write_32_bit_floats(modbus_alt, altdef.fault_status, altdef.no_fault)
        az_recovered = True
        alt_recovered = True

    if az_recovered and alt_recovered:
        Write_32_bit_floats(modbus_az, azdef.fault_acknowledge, 1)
        Write_32_bit_floats(modbus_alt, altdef.fault_acknowledge, 1)

        Wait_until_register(modbus_az, azdef.fault_status, azdef.no_fault, 24*3600)
        Wait_until_register(modbus_alt, altdef.fault_status, altdef.no_fault, 24 * 3600)

        redis_client.publish(redis_recovery_channel, msgpack.packb("Recovery sequence completed"))

    return az_recovered and alt_recovered


#This function recovers the telescope from going too close to the sun
def Sun_recovery(modbus_az, modbus_alt):

    from astropy.coordinates import SkyCoord, AltAz, Angle
    from astropy.coordinates import get_sun
    from astropy.coordinates import EarthLocation
    from astropy.time import Time
    from astropy import units as u
    from datetime import datetime
    import json

    driver_params = json.load(open("../configuration/TCS_driver_parameters.json"))
    parking_alt=driver_params["safety_parameters"]["parking_alt"]
    parking_az=driver_params["safety_parameters"]["parking_az"]
    software_timeout = driver_params["safety_parameters"]["software_timeout"]
    motion_timeout = driver_params["safety_parameters"]["movement_timeout"]
    elongation_from_sun = driver_params["safety_parameters"]["elongation_from_sun"]
    alt_max = driver_params["safety_parameters"]["max_alt"]
    alt_min = driver_params["safety_parameters"]["min_alt"]
    telescope_lon = driver_params["telescope_site"]["lon"]
    telescope_lat = driver_params["telescope_site"]["lat"]
    telescope_height = driver_params["telescope_site"]["height"]
    elongation_from_sun = driver_params["safety_parameters"]["elongation_from_sun"]
    elongation = Angle(elongation_from_sun, unit="degree")  # elongation that must be kept from the sun

    observing_location = EarthLocation(lat=telescope_lat, lon=telescope_lon, height=telescope_height * u.m)
    Location_Time = AltAz(location=observing_location, obstime=Time(datetime.now()))  # adding current time
    sun = get_sun(Time(datetime.now())).transform_to(Location_Time)  # getting the position of the sun
    sun.az.wrap_at("180d", inplace=True)  # defining Az coordinate to be kept between (-180deg and 180deg)

    az_encoder_pos = Read_32_bit_floats(modbus_az, azdef.ax_mpos)[0]
    alt_encoder_pos = Read_32_bit_floats(modbus_alt, altdef.ax_mpos)[0]

    start_alt, start_az = Encoder_to_AltAZ(alt_encoder_pos, az_encoder_pos)
    start = np.array([[start_az, start_alt]])
    start = SkyCoord(start, unit="degree", frame='altaz')
    start.az.wrap_at("180d", inplace=True)

    finish = np.array([[parking_az, parking_alt]])
    finish = SkyCoord(finish, unit="degree", frame='altaz')
    finish.az.wrap_at("180d", inplace=True)

    if start.separation(finish).deg < 1 / 3600:
        print("The telescope is already positioned correctly with a precision higher than 1 arcsec")
    else:
        az_traj, alt_traj = Traj_to_Pos(sun, start, finish, elongation, alt_max, alt_min)

        traj_lenght = len(
            az_traj)  # saving the lenght of the trajectory (they have the same len so i take len of az_traj)

        # The trajectory calculated in order to avoid the sun will never exceed 8 points. If the trajectory exceeds 8 points
        #  there has been an error in its calculation and it can't fit in the allocated memory
        if traj_lenght > 8 or len(az_traj) != len(alt_traj):
            raise Exception("CRITICAL ERROR!\n" +
                            "Error in calculating the trjectory to avoid the sun")

        # writing the trajectory and its lenght on the TrioControllers
        Write_32_bit_floats(modbus_az, azdef.jog_traj_lenght, traj_lenght)
        Write_32_bit_floats(modbus_az, azdef.jog_traj, az_traj)
        Write_32_bit_floats(modbus_alt, altdef.jog_traj_lenght, traj_lenght)
        Write_32_bit_floats(modbus_alt, altdef.jog_traj, alt_traj)

        # giving the command to start the sequence
        #Write_32_bit_floats(modbus_az, azdef.motion_command, azdef.usr_move_avoid_sun)
        #Write_32_bit_floats(modbus_alt, altdef.motion_command, altdef.usr_move_avoid_sun)

        # for the pointings in the list
        for i in range(1, traj_lenght):
            # printing on terminal the motion needed by the two axis (only one axis move at a time)
            # print("AZ: "+str(az_traj[i-1])+"-->"+str(az_traj[i]))
            # print("ALT: " + str(alt_traj[i - 1]) + "-->" + str(alt_traj[i]))

            # if it is the turn of the azimuth azis to move
            if az_traj[i] != az_traj[i - 1] and alt_traj[i] == alt_traj[i - 1]:
                # wait for azimuth axis to be idle and ready for motion
                Wait_until_coils(modbus_az, azdef.io_user_azimuth_idle, motion_timeout)
                # switch off the ide signal on azimuth controller
                modbus_az.write_single_coil(azdef.io_user_azimuth_idle, False)
                # giving the signal to the alt controller has reached idle state
                modbus_alt.write_single_coil(altdef.in_user_azimuth_idle, True)
                # print("AZ Moving!!!!")
            elif alt_traj[i] != alt_traj[i - 1] and az_traj[i] == az_traj[i - 1]:
                Wait_until_coils(modbus_alt, altdef.io_user_elevation_idle, motion_timeout)
                modbus_alt.write_single_coil(altdef.io_user_elevation_idle, False)
                modbus_az.write_single_coil(azdef.in_user_elevation_idle, True)
                # print("ALT Moving!!!!")
            else:
                raise Exception("CRITICAL ERROR:\n The trajectory calculated to avoid the sun is not valid!!")
