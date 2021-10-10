'**********************************************************************************************************
'PROJECT: LSPE-STRIP
'
'TrioController: AZIMUTH
'Program: MAIN
'
'Description
'   This is the main motion and process control program.
'   It is run as process 3 from MONITOR
'**********************************************************************************************************

INCLUDE "COMMON"
ignore_basic_err=FALSE
ON BASICERROR GOSUB basic_err
PRINT#5,"MAIN Running"
VR(system_status)=disabled
VR(motion_command)=stop_motion
VR(system_status)=WDOG+1

'Continuous loop to monitor motion commands
REPEAT
    IF WDOG=ON THEN
        IF VR(motion_command)=disable_all THEN
            'Disable all axis
            WDOG=OFF
            VR(system_status)=disabled
            WAIT UNTIL VR(motion_command)<>disable_all
            GOSUB clear_commands

        ELSEIF VR(motion_command)=usr_nominal_survey THEN
            'Run main machine sequence
            VR(system_status)=running
            GOSUB nominal_survey:
            VR(system_status)=ready
            GOSUB clear_commands

        ELSEIF VR(motion_command)=usr_table_wr THEN
            'Run sequence to Write on Table
            VR(system_status)=table_update
            GOSUB table_write
            VR(system_status)=ready
            GOSUB clear_commands

        ELSEIF VR(motion_command)=usr_raster_scan THEN
            'Run raster scan sequence
            VR(system_status)=scanning
            GOSUB raster_scan
            VR(system_status)=ready
            GOSUB clear_commands

        ELSEIF VR(motion_command)=usr_calibration_old AND FALSE THEN
            'Run calibration sequence
            VR(system_status)=calibrating
            GOSUB calibration
            VR(system_status)=ready
            GOSUB clear_commands

        ELSEIF VR(motion_command)=usr_calibration THEN
            'perform instrument calibration
            VR(system_status)=calibrating
            VR(current_point)=-1

            FOR point=0 TO VR(num_of_points)-1:
                WAIT UNTIL VR(next_point_ready)=1
                VR(next_point_ready)=0
                GOSUB move_avoid_sun
                VR(current_point)=point
                PRINT "Waiting camera"
            NEXT point
            VR(system_status)=ready
            GOSUB clear_commands



        ELSEIF VR(motion_command)=usr_move_avoid_sun THEN
            'Move avoiding the sun scan sequence
            VR(system_status)=moving
            GOSUB move_avoid_sun
            VR(system_status)=ready
            GOSUB clear_commands

        ENDIF
    ELSE
        VR(system_status)=disabled
        IF VR(motion_command)=enable_all THEN
            FOR ax=0 TO axis_count
                PRINT#5,"Clearing Axis ";ax;" errors"
                BASE(ax)
                CANCEL(2) 'Clear buffer
                WAIT IDLE
                DATUM(0) 'Clear FE
                AXIS_ENABLE=ON
                SERVO=ON
            NEXT ax
            WDOG=ON 'Enable Axes
            WA(200) 'Wait for axis to enable
            VR(system_status)=ready
            'WAIT UNTIL VR(motion_command)<>enable_all
            GOSUB clear_commands
        ENDIF
    ENDIF
    WA(20)
UNTIL FALSE

STOP


'-------------------------------------------------------------------------------
'Make sure motion command is clear so no unexpected motion starts
'-------------------------------------------------------------------------------
clear_commands:
VR(motion_command)=stop_all
RETURN


'-------------------------------------------------------------------------------
'Setup axis move speed and accel/decel rates
'-------------------------------------------------------------------------------
speed_setup:
BASE(ax0)
SPEED=VR(ax0_move_speed)
ACCEL=VR(ax0_accel_decel)
DECEL=VR(ax0_accel_decel)
RETURN

'-------------------------------------------------------------------------------
'Writing on TABLE memory
'-------------------------------------------------------------------------------
table_write:
PRINT #5, "Modbus moved on Table"
'assigning modbus to TABLE memory so that the client can write the spinning speeds and durations
ETHERNET(1, -1, 14, 0, 1, 4) 'Modbus TCP data assigned to TABLE
ETHERNET(1, -1, 14, 0, 2, 2) 'Modbus TCP 32 bit float mode
WA(10)
OP(io_user_modbus_table, ON) 'signaling to the client that the modbus has been moved to TABLE memory
WA(10) 'giving time to the controller to register the input as ON
WAIT UNTIL IN (io_user_modbus_table)=OFF 'Wait the client to finish writing on table memory and switch off the i/o

'assigning the modbus tback to VR memory
ETHERNET(1, -1, 14, 0, 1, 3) 'Modbus TCP data assigned to VR
ETHERNET(1, -1, 14, 0, 2, 2) 'Modbus TCP 16 bit int mode
PRINT #5, "Modbus back to VR"
RETURN

'-------------------------------------------------------------------------------
'Nominal Scanning Strategy
'-------------------------------------------------------------------------------
nominal_survey:
PRINT#5,"Nominal survey started"
'Setup axis speeds
GOSUB speed_setup

BASE(ax0)'select axis
SPEED=VR(nominal_survey_speed)'select desired speed

WAIT UNTIL IN(in_user_elevation_idle)=ON'wating for elevation motor to be idle
OP(in_user_elevation_idle, OFF)

start_time=VR(master_time)'saving the starting time
FORWARD
WAIT UNTIL VR(master_time)>start_time+VR(nominal_survey_duration) 'waiting the appropriate duration
CANCEL(2) 'stopping motion
WAIT IDLE

PRINT#5,"Nominal survey stopped"
RETURN

'-------------------------------------------------------------------------------
'Raster Scan
'-------------------------------------------------------------------------------
raster_scan:
PRINT#5,"Raster scan started"
start_time=VR(master_time)
'Setup axis speeds
GOSUB speed_setup
BASE(ax0)
SPEED=VR(raster_scan_speed)
DECEL=ACCEL/VR(raster_scan_accel_to_decel_frac)

WAIT UNTIL IN(in_user_elevation_idle)=ON'wating for elevation motor to be idle
OP(in_user_elevation_idle, OFF)

TRIGGER
WA(50)
MOVE(-1*VR(raster_scan_speed)^2/ACCEL) 'moving back the motor so that it will reach the speed in time
WAIT IDLE

WHILE VR(master_time)<start_time+VR(raster_scan_duration): 'while we are in the duration
    FORWARD
    'if the scan contains the axis wrapping limit (es Az from 170deg->-170deg passing from 179deg and -179deg)
    IF VR(raster_scan_az_max)<VR(raster_scan_az_min) THEN
        WAIT UNTIL MPOS<0 'wait until you passed the axis wrapping
        PRINT MPOS
    ENDIF
    WAIT UNTIL MPOS>VR(raster_scan_az_max) 'then wait until you passed the end of the box you had to scan
    CANCEL(2) 'cancel motion
    WAIT IDLE 'wait the motor to stop
    REVERSE
    'if the scan contains the axis wrapping limit (es Az from 170deg->-170deg passing from 179deg/-179deg limit)
    IF VR(raster_scan_az_max)<VR(raster_scan_az_min) THEN
        WAIT UNTIL MPOS>0 'wait until you passed the axis wrapping
        PRINT MPOS
    ENDIF
    WAIT UNTIL MPOS<VR(raster_scan_az_min) 'wait until you moved back to the start of the box you had to scan
    CANCEL(2) 'cancel motion
    WAIT IDLE 'wait the motor to stop
WEND
TRIGGER
GOSUB speed_setup

PRINT#5,"Raster Scan stopped"
RETURN

'-------------------------------------------------------------------------------
'calibration
'-------------------------------------------------------------------------------
calibration:
PRINT#5,"Calibration sequence started"
'Setup axis speeds
GOSUB speed_setup
BASE(ax0)
VR(calibration_target_pos)=-1 'initializing the VR variable where the target position will be stored
current_pos=-1 'variable that keeps track of which pointing of the list the the one that is currently being used
OP(io_user_azimuth_idle,ON) 'signals that the system is ready to recive an instruction
WAIT UNTIL VR(calibration_target_pos)<>-1 'waiting until the desired pointing from the list is specified

WHILE VR(calibration_target_pos)<>99: 'until the target position is different than the switch-off value
    IF current_pos<>VR(calibration_target_pos) THEN 'if we are not in the desired position
        OP(io_user_azimuth_idle,OFF) 'signal that the sistem is not idle

        'if the desired pos - current pos is more than 180deg the motor should roll around the -180deg,180deg limit
        IF ABS(TABLE(VR(calibration_table_start)+VR(calibration_target_pos))-MPOS)>(VR(ax0_units_rev)/2) THEN
            'since MOVEABS odes not take into consideration the rollover at 180deg we must add the
            ' following motions to move the azimuth past the rollover if it is convenient
            IF MPOS>=0 THEN
                MOVE(180-MPOS+1)
                WAIT IDLE
            ELSE
                MOVE(-(180+MPOS)-1)
                WAIT IDLE
            ENDIF
        ENDIF
        'we move the telescope to the desired azimuth
        MOVEABS (TABLE(VR(calibration_table_start) + VR(calibration_target_pos))) 'move to the desired position
        WAIT IDLE
        current_pos=VR(calibration_target_pos) 'update the variable
        OP(io_user_azimuth_idle,ON)
    ENDIF

WEND
OP(io_user_azimuth_idle,OFF)


PRINT#5,"Calibration sequence stopped"
RETURN

'-------------------------------------------------------------------------------
'Move to start
'-------------------------------------------------------------------------------
move_avoid_sun:
PRINT#5,"Moving to starting position"
'Setup axis speeds
GOSUB speed_setup
BASE(ax0)
TRIGGER

FOR pos=1 TO VR(jog_traj_lenght)-1: 'cicling over the positions of the trajectory
    'if the step of the trajectory changes Alt and not Az wait for Alt motor to be idle
    IF VR(jog_traj+pos)=VR(jog_traj+pos-1) THEN
        PRINT #5, "Waiting for elevation to be idle"
        PRINT #5, VR(jog_traj+pos-1), "--->", VR(jog_traj+pos)
        WA(20)
        WAIT UNTIL IN(in_user_elevation_idle)
        OP(in_user_elevation_idle, OFF)
    ELSE 'if instead the step of the trajectory changes Az
        PRINT #5, "Moving:"
        PRINT #5, VR(jog_traj+pos-1), "--->", VR(jog_traj+pos)

        WAIT UNTIL IN(io_user_azimuth_idle)=OFF

        'if the desired pos - current pos is more than 180deg the motor should roll around the -180deg,180deg limit
        IF ABS (VR(jog_traj+pos)-MPOS)>(VR(ax0_units_rev)/2) THEN
            'since MOVEABS odes not take into consideration the rollover at 180deg we must add the
            ' following motions to move the azimuth past the rollover if it is needed
            IF MPOS>=0 THEN
                MOVE(180-MPOS+1)
                WAIT IDLE
            ELSE
                MOVE(-(180+MPOS)-1)
                WAIT IDLE
            ENDIF
        ENDIF

        MOVEABS( VR(jog_traj+pos) ) 'move the motor in position
        WAIT IDLE 'wait end of motion
        OP(io_user_azimuth_idle, ON) 'signal that the motor is idle to workstation and Alt TrioController
    ENDIF
NEXT pos

TRIGGER

PRINT#5,"Telescope in starting position"
RETURN


'-------------------------------------------------------------------------------
'Move to start OLD
'-------------------------------------------------------------------------------
move_avoid_sun_old:
PRINT#5,"Moving to starting position"
'Setup axis speeds
GOSUB speed_setup
BASE(ax0)
TRIGGER

FOR pos=1 TO VR(avoid_sun_len_table)-1: 'cicling over the positions of the trajectory
    'if the step of the trajectory changes Alt and not Az wait for Alt motor to be idle
    IF TABLE(VR(avoid_sun_az_table_start)+pos)=TABLE(VR(avoid_sun_az_table_start)+pos-1) THEN
        PRINT #5, "Waiting for elevation to be idle"
        PRINT #5, TABLE(VR(avoid_sun_az_table_start)+pos-1), "--->", TABLE(VR(avoid_sun_az_table_start)+pos)
        WA(20)
        WAIT UNTIL IN(in_user_elevation_idle)
        OP(in_user_elevation_idle, OFF)
    ELSE 'if instead the step of the trajectory changes Az
        PRINT #5, "Moving:"
        PRINT #5, TABLE(VR(avoid_sun_az_table_start)+pos-1), "--->", TABLE(VR(avoid_sun_az_table_start)+pos)

        WAIT UNTIL IN(io_user_azimuth_idle)=OFF

        'if the desired pos - current pos is more than 180deg the motor should roll around the -180deg,180deg limit
        IF ABS (TABLE(VR(avoid_sun_az_table_start)+pos)-MPOS)>(VR(ax0_units_rev)/2) THEN
            'since MOVEABS odes not take into consideration the rollover at 180deg we must add the
            ' following motions to move the azimuth past the rollover if it is needed
            IF MPOS>=0 THEN
                MOVE(180-MPOS+1)
                WAIT IDLE
            ELSE
                MOVE(-(180+MPOS)-1)
                WAIT IDLE
            ENDIF
        ENDIF

        MOVEABS( TABLE(VR(avoid_sun_az_table_start)+pos) ) 'move the motor in position
        WAIT IDLE 'wait end of motion
        OP(io_user_azimuth_idle, ON) 'signal that the motor is idle to workstation and Alt TrioController
    ENDIF
NEXT pos

TRIGGER

PRINT#5,"Telescope in starting position"
RETURN

'-------------------------------------------------------------------------------
'Programming Errors are trapped here.
'Before the program terminates we set the 'ignore_basic_err' flag to prevent
'an error message being generated by a normal program termination.
'-------------------------------------------------------------------------------
basic_err:
IF ignore_basic_err<>TRUE AND VR(mask_basic_err)<>TRUE THEN
    IF RUN_ERROR=31 THEN
        PRINT#5,"MAIN - PROGRAM ABORTED AT LINE ";ERROR_LINE
        STOP
    ELSE
        PRINT#5,"MAIN - basic_err Error Line ";ERROR_LINE
        PRINT#5,"MAIN - basic_err Run Error ";RUN_ERROR
    ENDIF
    VR(err_proc_no)=PROCNUMBER
    VR(err_line_no)=ERROR_LINE
    VR(err_code_no)=RUN_ERROR
    VR(basic_err_fault)=TRUE 'Signal fault to MONITOR program
    RAPIDSTOP 'Stop motion on all axis
    FOR ax=0 TO axis_count 'Clear any buffered moves
        CANCEL(2) AXIS(ax)
    NEXT ax
    WA(10)
    FOR ax=0 TO axis_count 'Wait for all axis to stop
        WAIT IDLE AXIS(ax)
    NEXT ax

    WDOG=OFF
    WA(10)
ENDIF
STOP

RETURN




