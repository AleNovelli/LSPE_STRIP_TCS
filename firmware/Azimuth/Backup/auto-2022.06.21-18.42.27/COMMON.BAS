'**********************************************************************************************************
'PROJECT: LSPE-STRIP
'
'TrioController: AZIMUTH
'Program: COMMON
'
'Description:
'   This program contains the difinitions of all the I/O, VR and variable allocations of the Azimuth
'   TrioController. It is not run from any other program but it must be included at the beginning of each program.
'**********************************************************************************************************


'--------------------------------------------------------------------------------
'I/O Allocations
'--------------------------------------------------------------------------------

'Digital input allocations (on main controller)
in_estop_healthy=0 'this input turns OFF if the emergency stop button is pressed
in_start_button=1 'allows the user to start the main sequences  <------ ci lego l'enabling degli assi??
in_stop_button=2 'allows the user to stop the main sequences????  <---- ci lego l'enabling degli assi??
in_user_tos=3 'input coming from the master clock with the Top Of Second
in_user_4=4
in_user_5=5
in_user_6=6
in_user_7=7

'Digital I/O allocations (on main controller)
out_fault_lamp=8 'Turning on this output will turn on the faul lamp
io_user_azimuth_idle=9 'turns on to signal that the azimuth is idle - turned off via modbus when new movement must start
in_user_elevation_idle=10 'input coming from the client which signals that the elevation motor is idle
io_user_modbus_table=11 'signals when the user moved the modbus communication on table memory (NOT USED)

io_user_12=12
io_user13=13
out_relay_0=14 'switching OFF these relays will turn off digital outputs <---- li vogliamo???
out_relay_1=15 'switching OFF these relays will turn off digital outputs <---- li vogliamo???

'Axis 0 Drive I/O via EtherCAT
in_ax0_pos_limit=32
in_ax0_neg_limit=33
in_ax0_home=34
io_user_35=35
io_user_36=36
io_user_37=37
io_user_38=38
io_user_39=39

'--------------------------------------------------------------------------------
'VR Allocations
'--------------------------------------------------------------------------------
software_version=0 'indica la versione del software utlizzato (secondo me non necessaria)
firmware_version=1 'indica la versione del firmware utlizzato (secondo me non necessaria)
err_proc_no=2 'indicates the processor which suffered a programming error
err_line_no=3 'indicates the line of code at which the programming error occurred
err_code_no=4 'indicates which error occured as described by the RUN_ERROR variable of TrioBASIC
default_vr=5 'indicates if the default parameters have already been applied
mask_basic_err=6 'decides if a TrioBASIC error should raise an error(ON quando vogliamo stoppare i programmi da
'MONITOR o da un altro programma senza causare errori)
mask_limits=7 'decides if a axes limit error should raise an error (ON per autorizzare movimenti di recovery dopo che
'sono stati superati i limiti)
basic_err_fault=8 'Used to Signal programming error to MONITOR program
ethercat_state=9 'Describes the state of ethercat network as desctibed by the ETHERCAT comand from TrioBASIC

'Status VR's
system_status=10 'describes the status of the overall system (ready, running, stopping, etc...)
fault_status=11 'describes which kind of error, if any, the system has encountered
fault_latch=12 'decides if errors are automatically corrected or if the system should stop<---NO ERR RECOVER IMPLEMENTED
home_status=13 'describes if the system is homed <------ NOT NECESSARY BECAUSE WE HAVE ABSOLUTE ENCODERS
system_warning=14 'indicates eventual warnings <---- STILL NOT IMPLEMENTED
workstation_safety_flag=15 'if this memory location contains a number <0 the connection to the workstation has failed
'master_clock_safety_flag=16 'if this memory location contains a number <0 the connection to the master clock has failed
'NON PIù USATA
antisun_direction=17
'i_o_status_0_31=15 'it is a 32-bit variable which describes bit by bit which inputs & outputs are on between 0 and 31
'i_o_status_32_63=16 'it is a 32-bit variable which describes bit by bit which inputs & outputs are on between 32 and 63
'ain_0_status=17 'describes the status of analogic input 0 <--- WE DO NOT HAVE ANALOGIC INPUTS
'ain_1_status=18 'describes the status of analogic input 1 <--- WE DO NOT HAVE ANALOGIC INPUTS

'Command VR's
motion_command=20 'describes the motion to be executed
'motion_axis=21 'decides to which axes the motion should be applied to
fault_acknowledge=22 'decribes if an error has been acknowledged <-- RIGHT NOW THERE IS NOT A WAY TO ACKNOWLEDGE ERRORS
'aout_0_value=26 'indicates a value to be transmitted to the DAC <--- WE DO NOT HAVE ANALOGIC OUTPUTS
'aout_1_value=27 'indicates a value to be transmitted to the DAC <--- WE DO NOT HAVE ANALOGIC OUTPUTS

'USER:
'communications with master clock and encoder position monitor
enc_update_freq=30 'where we store how many times in a second we want to sample the encoder
master_time_from_mc=31 'where the time from the master clock is stored
master_time_to_ws=32 'where the workstation reads the time from
ticks_write=33 'where we store the ticks parameter so that the client can track the position of the motors
ax_mpos=34 'where we store the mpos parameter so that the client can track the position of the motors
ticks_at_tos=35 'where we stored the number of processor ticks at the beggining of the current Top Of Second
mc_warning=36 'This falg becomes 1 if the signal from the MC is lost


'locations on VR MEMORY where to read the main sequence parameters (elevation & spin_speed & spin_duration)
nominal_survey_alt=40 'elevation at which to perform the main sequence
nominal_survey_speed=41 'speed at which to perform the main sequence
nominal_survey_duration=42 'duration of the main sequence

'raster scan parameters VR memory locations
raster_scan_alt=50
raster_scan_az_min=51
raster_scan_az_max=52
raster_scan_duration=53
raster_scan_speed=54
raster_scan_accel_to_decel_frac=55

'Calibration parameters VR memory locations
num_of_points=60 'number of points in the calibration list
current_point=61 'point which we are currently observing
next_point_ready=62 'this location of memory is turned on when th axis is ready to move to the next point of the list


'Avoid Sun motion VR memory locations
jog_traj_lenght=70
jog_traj=71'WARNING!!! VR addresses from 71 to 79 will be used to store the trajectory to jog avoiding the sun, DO NOT
'USE!

az_pos_def=80
az_enc_def=81

'Parametri per i test a Bologna
encoder_test_ready=90
encoder_test_value=91

'Axis 0 parameters
ax0_limit_config=101 'descrive se usiamo limit switch e di che tipo (0:No switch / 1:Switch NC / 2:Switch NO)
ax0_home_config=102 'in which direction to run a homing process <--- NON USATO (ABSOLUTE ENCODERS)
ax0_steps_rev=103 'how many encoder steps constitute a revolution of the axis
ax0_units_rev=104 'user units in which we want to divide a motor revolution
ax0_rollover_dist=105 'when do we want the axis rollover to happen
ax0_rollover_option=106 'if rollover axis va da -180->180 o da 0->360
ax0_min_pos=107 'minimum acceptable position of the axis
ax0_max_pos=108 'maximum acceptable position of the axis
ax0_jogspeed=109 'jogging speed
ax0_creep_speed=110 'creeping speed
ax0_move_speed=111 'motion speed
ax0_accel_decel=112 'acceleration and deceleration of the motor
ax0_s_curve_time=113 'quanto arrotonda movimenti "spigolosi"
ax0_move_dist=120 'utilizzato dal jog/move_incremental che aveva implementato triomotion
ax0_move_pos=121 'utilizzato dal move_absolute che aveva implementato triomotion
ax0_limit_dist=122 'The distance the axis will move to recover from hitting a limit
ax0_mpos=130 'position of axis 0

'Encoder Axis parameters
encoder_steps_rev=503 'step in una rivoluzione dell'asse encoder
encoder_units_rev=504 'unità in una rivoluzione dell'asse encoder
encoder_rollover_dist=505 'distanza di rollover dell'encoder
encoder_rollover_option=506 'see Trio REP_DIST command
encoder_mpos=530 'posizione encoder

'--------------------------------------------------------------------------------
'Program Constants
'--------------------------------------------------------------------------------
'Configuration constants

'fault_status constants
no_fault=0 'there are no errors
estop=1 'the ESTOP button has been pressed
canio_fault=2 'error in the CAN communication channel
ax0_pos_limit=3 'axis 0 has reached the positive travel limit
ax0_rev_limit=4 'axis 0 has reached the negative travel limit
ax0_drv_comms_fault=5 'communications with the drives of axis0 have failed (USATI IN MONITOR: LINES 36-38) <---------
ax0_drive_fault=6 'error in axis0 drives (USATI IN MONITOR: LINES 36-38) <----------
ax0_fe_fault=7 'axis 0 has an eccessive following error (USATI IN MONITOR: LINES 36-38) <----------

'sun_fault=25 'the telescope is pointing too close to the sun 'DOPPIONE DI sun_elongation
encoder_samplig_fault=26 'the encoder sampling required is too high for the capabilities of the TrioController
workstation=27 'the communications to the workstation have failed
master_clock=28 'no signal from the master clock
other_axis=29 'error raised in the other axis of motion
'alt_min=96 'telescope elevation is below safety parameters
'alt_max=97 'telescope elevation is above safety parameters
sun_elongation=98 'telescope elongation from sun is blow safety limit
basic_error=99 'errore nell'esecuzione di un programma TRIOBASIC

'system_status constants
initialising=0
disabled=1
ready=2
homing=3
jogging=4
moving=5
running=6
stopping=7
table_update=8
scanning=9
calibrating=10
spinning=11
defining_pos=12
testing=13 '<------ TO BE REMOVED BEFORE DEPLOYMENT

fault=99

'system_warning constants
no_warning=0
'bit 0 = ax0 Positive SW limit
'bit 1 = ax0 Reverse SW limit
'bit 2 = ax1 Positive SW limit
'bit 3 = ax1 Reverse SW limit
'bit 4 = ax2 Positive SW limit
'bit 5 = ax2 Reverse SW limit
'bit 6 = ax3 Positive SW limit
'bit 7 = ax4 Reverse SW limit
'bit 8 = ax0 Warning Following Error
'bit 9 = ax1 Warning Following Error
'bit 10 = ax2 Warning Following Error
'bit 11 = ax3 Warning Following Error

'motion_command constants
await_all=0 'await instructions
enable_all=1 'controlling axes faults and enabling motion
disable_all=2 'disabling all axes
stop_all=3 'stopping all axes
'home_all=3 'homing axis
'ax0_jog_pos=4
'ax0_jog_rev=8
'move_abs=12
'move_inc=13
usr_nominal_survey=14
usr_table_wr=15
usr_raster_scan=16
usr_calibration=17
usr_move_avoid_sun=18
redefine_pos=19
encoder_test=20
timing_test=21
'set_aout_0=15
'set_aout_1=16

'motion_axis constants
x=1
'y=2
'xy=3

'Home_status constants <-- NOT RELEVANT, ABSOLUTE ENCODERS
'not_homed=0
'home_complete=1

'USER: Communication constants
'parameters for the communication of encoder positions
encoder_update_freq=4 'how many times per second i want to store the position of the motor

'--------------------------------------------------------------------------------
'Axis Allocations
'--------------------------------------------------------------------------------
ax0=0
encoder_axis=31 '<-------------------------------POCO CHIAROO ???????

axis_count=0 'The maximum axis number used for motion



