'**********************************************************************************************************
'PROJECT: LSPE-STRIP
'
'TrioController: ELEVATION
'Program: COMMON
'
'Description:
'   This program contains the difinitions of all the I/O, VR and variable allocations of the Azimut
'   TrioController. It is not run from any other program but it must be included at the beginning of each program.
'**********************************************************************************************************


'--------------------------------------------------------------------------------
'I/O Allocations
'--------------------------------------------------------------------------------

'Digital input allocations (on main controller)
in_estop_healthy=0 'this input turns OFF if the emergency stop button is pressed
in_start_button=1 'allows the user to start the main sequences
in_stop_button=2 'allows the user to stop the main sequences????  <--------------------------------
in_user_tos=3 'input coming from the master clock with the Top OF Second
in_user_azimuth_idle=4 'input coming from the client which signals that the elevation motor is idle
in_user_5=5
in_user_6=6
in_user_7=7

'Digital I/O allocations (on main controller)
out_fault_lamp=8 'Turning on this output will turn on the faul lamp
io_user_modbus_table=9 'signals when the user moved the modbus communication on table memory
io_user_elevation_idle=10
io_user_11=11
io_user_12=12
io_user13=13
out_relay_0=14 'switching OFF these relays will turn off digital outputs??? (in MONITOR) <--------------------
out_relay_1=15 'switching OFF these relays will turn off digital outputs??? (in MONITOR) <--------------------


'--------------------------------------------------------------------------------
'VR Allocations
'--------------------------------------------------------------------------------
software_version=0 'indica la versione del software utlizzato (secondo me non necessaria)
firmware_version=1 'indica la versione del firmware utlizzato (secondo me non necessaria)
err_proc_no=2 'indicates the processor which suffered a programming error
err_line_no=3 'indicates the line of code at which the programming error occurred
err_code_no=4 'indicates which error occured as described by the RUN_ERROR variable of TrioBASIC
default_vr=5 'indicates if the default parameters have been applied?? (in STARTUP) <--------------------
mask_basic_err=6 'decides if a TrioBASIC error should cancel the execution and stop motion???? <--------------------
mask_limits=7 'decides if a axes limit error should cancel the execution and stop motion???? <--------------------
basic_err_fault=8 'Used to Signal programming error to MONITOR program
ethercat_state=9 'Describes the state of ethercat network as desctibed by the ETHERCAT comand from TrioBASIC

'Status VR's
system_status=10 'describes the status of the overall system (ready, running, stopping, etc...)
fault_status=11 'describes which kind of error, if any, the system has encountered
fault_latch=12 'decides if errors are automatically correcter or if the system should stop???  <--------------------
home_status=13 'describes if the system is homed
system_warning=14 'indicates eventual warnings
i_o_status_0_31=15 'it is a 32-bit variable which describes bit by bit which inputs & outputs are on between 0 and 31
i_o_status_32_63=16 'it is a 32-bit variable which describes bit by bit which inputs & outputs are on between 32 and 63
'ain_0_status=17 'describes the status of analogic input 0???
'ain_1_status=18 'describes the status of analogic input 1???

'Command VR's
motion_command=20 'describes the motion to be executed
motion_axis=21 'decides to which axes the motion should be applied to
fault_acknowledge=22 'decribes if an error has been acknowledged (from user or from program??)???? <--------------------
aout_0_value=26 'indicates a value to be transmitted to the DAC (do we need it??)???? <--------------------
aout_1_value=27 'indicates a value to be transmitted to the DAC (do we need it??)???? <--------------------

'USER:
'communications with master clock and encoder position monitor
master_time=30 'where the time from the master clock is stored <------------(might need more variables for hh:mm:ss)????
ticks_write=31 'where we store the ticks parameter so that the client can track the position of the motors
ax0_mpos=32 'where we store the mpos parameter so that the client can track the position of the motors

'locations on table where to read the main sequence list from the client (elevation & spin_speed & spin_duration)
nominal_survey_alt=40 'where on table memory the client writes the list of the elevations for the main sequence
nominal_survey_speed=41 'where on table memory the client writes the list of the spin speeds for the main sequence
nominal_survey_duration=42 'where on table memory the client writes the list of the spin durations for the main sequence

'raster scan parameters VR memory locations
raster_scan_alt=50
raster_scan_az_min=51
raster_scan_az_max=52
raster_scan_duration=53
raster_scan_speed=54
raster_scan_accel_to_decel_frac=55

'Calibration parameters VR memory locations
calibration_table_start=60
calibration_table_lenght=61
calibration_target_pos=62

'Avoid Sun motion VR memory locations
'avoid_sun_alt_table_start=70
'avoid_sun_az_table_start=71
'avoid_sun_len_table=72
jog_traj_lenght=70
jog_traj=71 'WARNING!!! VR addresses from 71 to 79 will be used to store the trajectory to jog avoiding the sun
'KEEP VR 71 to 79 FRE!!!

'Axis 0 parameters
ax0_limit_config=101
ax0_home_config=102
ax0_steps_rev=103
ax0_units_rev=104
ax0_rollover_dist=105
ax0_rollover_option=106
ax0_min_pos=107
ax0_max_pos=108
ax0_jogspeed=109
ax0_creep_speed=110
ax0_move_speed=111
ax0_accel_decel=112
ax0_s_curve_time=113
ax0_move_dist=120
ax0_move_pos=121
ax0_limit_dist=122
ax0_mpos=130

'Encoder Axis parameters
encoder_steps_rev=503
encoder_units_rev=504
encoder_rollover_dist=505
encoder_rollover_option=506
encoder_mpos=530

'--------------------------------------------------------------------------------
'Program Constants
'--------------------------------------------------------------------------------
'Configuration constants

'fault_status constants
no_fault=0 'there are no errors
estop=1 'the ESTOP button has been pressed
canio_fault=2 'error in the CAN communication channel
ax0_pos_limit=3 'axis 0 has reached the positive travel limit
ax0_rev_limit=7 'axis 0 has reached the negative travel limit
ax0_drv_comms_flt=11 'communications with the drives of axis0 have failed (USATI IN MONITOR: LINES 36-38) <---------
ax0_drive_fault=15 'error in axis0 drives (USATI IN MONITOR: LINES 36-38) <----------
ax0_fe_fault=19 'axis 0 has an eccessive following error (USATI IN MONITOR: LINES 36-38) <----------
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
stop_all=0 'stopping all axes
enable_all=1 'controlling axes faults and enabling motion
disable_all=2 'disabling all axes
home_all=3 'homing axis
ax0_jog_pos=4
ax0_jog_rev=8
move_abs=12
move_inc=13
usr_nominal_survey=14
usr_table_wr=15
usr_raster_scan=16
usr_calibration=17
usr_move_avoid_sun=18
'set_aout_0=15
'set_aout_1=16

'motion_axis constants
x=1
y=2
z=4
r=8
xy=3
xz=5
yz=6
xr=9
yr=10
zr=12
xyz=7
xyr=11
xzr=13
yzr=14
xyzr=15

'Home_status constants
not_homed=0
home_complete=1

'USER: Communication constants
'parameters for the communication of encoder positions
encoder_update_freq=4 'how many times per second i want to store the position of the motor

'--------------------------------------------------------------------------------
'Axis Allocations
'--------------------------------------------------------------------------------
ax0=0
encoder_axis=31 '<-------------------------------POCO CHIAROO ???????

axis_count=0 'The maximum axis number used for motion




