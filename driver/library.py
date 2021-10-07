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
    while time.time() < mustend:
        if all(client.read_coils(coils)): return True
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

    #preparing a list were to store the parameters of each line of the file
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
    if controller.close() == True:  # disconnecting Azimuth Trio-Controller
        print(name+" Disconnected correctly !!!")
    else:
        print("Error in disconnecting "+name+" !!!")

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
                Write_32_bit_floats(controller, azdef.motion_command, azdef.stop_all)
                time.sleep(.1)#waiting for the controllers to register the command changed from disable all
            Write_32_bit_floats(controller, azdef.motion_command, azdef.enable_all)

        # Waiting for the Trio-Controller to be enabled and ready for a motion command
        Wait_until_register(controller, azdef.system_status, azdef.ready, timeout)

    elif alt_az == "alt":
        # if the Azimuth Controller is disabled enable all axis
        if status == altdef.disabled:
            print("Enabling Azimut Axis")
            if Read_32_bit_floats(controller, altdef.motion_command)[0] == altdef.disable_all:
                Write_32_bit_floats(controller, altdef.motion_command, altdef.stop_all)
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
            if az.shape == ():
                full_az = np.concatenate((traj_.az.deg, np.array([az.deg + traj_[-1].az.deg])))
            else:
                full_az = np.concatenate((traj_.az.deg, az.deg + traj_[-1].az.deg))

            if alt.shape == ():
                full_alt = np.concatenate((traj_.alt.deg, np.array([alt.deg + traj_[-1].alt.deg])))
            else:
                full_alt = np.concatenate((traj_.alt.deg, alt.deg + traj_[-1].alt.deg))
        else:
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
        traj = move(traj, np.sign((traj[-1].az - sun.az).wrap_at("180d")) * (
                    hor_rad - abs((traj[-1].az - sun.az).wrap_at("180d"))), Angle(0, unit="deg"))
        traj_az_in_sun = False  # I moved the telescope outside the sun Az range

    if obj_az_in_sun and obj_alt_in_sun:  # if final pos is close to the sun program a move to get close safely
        print(2)
        obj = move(obj, np.sign((obj[0].az - sun.az).wrap_at("180d")) * (
                    hor_rad - abs((obj[0].az - sun.az).wrap_at("180d"))), Angle(0, unit="deg"), reverse=True)
        obj_az_in_sun = False  # I moved the objective outside the sun Az range

    # calculating the relative position of the telescope, sun and final pos
    traj_to_sun = (sun.az - traj[-1].az).wrap_at("180d")
    traj_to_obj = (obj[0].az - traj[-1].az).wrap_at("180d")
    # checking if the sun is in the way of the telescope while moving straight to the final pos
    sun_in_the_way = min(0, traj_to_obj) - hor_rad < traj_to_sun < max(0, traj_to_obj) + hor_rad

    if not sun_in_the_way:
        print(3)  # if the sun is not in the way move straight to the objective
        traj = moveabs(traj, obj[0].az, traj[-1].alt)
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