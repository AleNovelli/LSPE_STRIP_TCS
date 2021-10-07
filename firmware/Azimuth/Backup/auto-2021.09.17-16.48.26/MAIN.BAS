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

        ELSEIF VR(motion_command)=home_all THEN
            'Datum  all axis
            VR(system_status)=homing
            GOSUB datum_seq
            VR(system_status)=ready
            GOSUB clear_commands

        ELSEIF VR(motion_command)=ax0_jog_pos OR VR(motion_command)=ax1_jog_pos OR VR(motion_command)=ax2_jog_pos _
                OR VR(motion_command)=ax3_jog_pos THEN
            'Jog axis in positive direction
            VR(system_status)=jogging
            GOSUB forward_jog
            VR(system_status)=ready
            GOSUB clear_commands

        ELSEIF VR(motion_command)=ax0_jog_rev OR VR(motion_command)=ax1_jog_rev OR VR(motion_command)=ax2_jog_rev _
                OR VR(motion_command)=ax3_jog_rev THEN
            'Jog axis in reverse direction
            VR(system_status)=jogging
            GOSUB reverse_jog
            VR(system_status)=ready
            GOSUB clear_commands

        ELSEIF VR(motion_command)=move_abs AND VR(home_status)=home_complete THEN
            'Move to absolute position
            VR(system_status)=moving
            GOSUB move_absolute
            VR(system_status)=ready
            GOSUB clear_commands

        ELSEIF VR(motion_command)=move_inc AND VR(home_status)=home_complete THEN
            'Move to incremental position
            VR(system_status)=moving
            GOSUB move_incremental
            VR(system_status)=ready
            GOSUB clear_commands

        ELSEIF VR(motion_command)=start_seq AND VR(home_status)=home_complete THEN
            'Run main machine sequence
            VR(system_status)=running
            GOSUB run_seq
            VR(system_status)=ready
            'GOSUB clear_commands
            VR(motion_command)=disable_all

        ELSEIF VR(motion_command)=usr_table_wr THEN
            'Run sequence to Write on Table
            VR(system_status)=table_update
            GOSUB table_write
            VR(system_status)=ready
            GOSUB clear_commands

        ELSEIF VR(motion_command)=usr_raster_scan THEN
            'Run raster scan sequence
            GOSUB raster_scan
            VR(system_status)=ready
            'GOSUB clear_commands
            VR(motion_command)=disable_all

        ELSEIF VR(motion_command)=usr_drone_tracking THEN
            'Run raster scan sequence
            GOSUB drone_tracking
            VR(system_status)=ready
            'GOSUB clear_commands
            VR(motion_command)=disable_all

        ELSEIF VR(motion_command)=usr_move_avoid_sun THEN
            'Move avoiding the sun scan sequence
            GOSUB move_avoid_sun
            VR(system_status)=ready
            'GOSUB clear_commands
            VR(motion_command)=disable_all

'        ELSEIF VR(motion_command)=set_aout_0 THEN
'            'Write analogue output 0 value
'            IF VR(aout_0_value)<-2048 THEN VR(aout_0_value)=-2048 'Check and set value within range
'            IF VR(aout_0_value)>2047 THEN VR(aout_0_value)=2047
'            DAC AXIS(aout_user_0)=VR(aout_0_value) 'Write value to output
'            GOSUB clear_commands

'        ELSEIF VR(motion_command)=set_aout_1 THEN
'            'Write analogue output 1 value
'            IF VR(aout_1_value)<-2048 THEN VR(aout_1_value)=-2048 'Check and set value within range
'            IF VR(aout_1_value)>2047 THEN VR(aout_1_value)=2047
'            DAC AXIS(aout_user_1)=VR(aout_1_value) 'Write value to output
'            GOSUB clear_commands
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
'Routine to datum all axis
'Axes will datum in the direction as set in home_config
'Any axis without a datum switch will be set to zero without any motion
'Axes will home in order axis0, axis1, axis2, axis3
'Axis jogspeed will be used for homing
'-------------------------------------------------------------------------------
datum_seq:
PRINT#5,"Datum sequence started"
VR(home_status)=not_homed

'Home Axis 0
PRINT#5,"Homing Axis 0"
BASE(ax0)
SPEED=VR(ax0_jogspeed)
ACCEL=VR(ax0_accel_decel)
DECEL=VR(ax0_accel_decel)
FS_LIMIT=2^30
RS_LIMIT=-(2^30)
ax0_home_found=FALSE
VR(mask_limits)=TRUE 'Disable limits so that they can be monitored without tripping the system
IF VR(ax0_home_config)=1 OR VR(ax0_home_config)=2 THEN 'Home in a positive direction
    FORWARD
    REPEAT
        IF IN(in_ax0_pos_limit)=OFF THEN 'If limit switch is found first
            CANCEL
            WA(10)
            WAIT IDLE
            REVERSE 'reverse move looking for the home switch
            WAIT UNTIL IN(in_ax0_home)=OFF
            ax0_home_found=TRUE
            WA(50)
            CANCEL
        ELSEIF IN(in_ax0_home)=OFF THEN 'If home switch is found first
            ax0_home_found=TRUE
            CANCEL
        ENDIF
        WA(10)
    UNTIL ax0_home_found=TRUE
    WA(10)
    WAIT IDLE
    VR(mask_limits)=FALSE 'Re-enable limit switch monitoring
    DATUM(3) 'home axis in forward direction
ELSEIF VR(ax0_home_config)=3 OR VR(ax0_home_config)=4 THEN 'Home in a negative direction
    REVERSE
    REPEAT
        IF IN(in_ax0_rev_limit)=OFF THEN 'If limit switch is found first
            CANCEL
            WA(10)
            WAIT IDLE
            FORWARD 'forward move looking for the home switch
            WAIT UNTIL IN(in_ax0_home)=OFF
            ax0_home_found=TRUE
            WA(50)
            CANCEL
        ELSEIF IN(in_ax0_home)=OFF THEN 'If home switch is found first
            ax0_home_found=TRUE
            CANCEL
        ENDIF
        WA(10)
    UNTIL ax0_home_found=TRUE
    WA(10)
    WAIT IDLE
    VR(mask_limits)=FALSE 'Re-enable limit switch monitoring
    DATUM(4) 'home axis in reverse direction
ELSE 'Zero the current position only
    CANCEL
ENDIF
WA(10)
WAIT IDLE
VR(mask_limits)=FALSE 'Re-enable limit switch monitoring
DEFPOS(0) 'Zero the position register
FS_LIMIT=VR(ax0_max_pos)+1
RS_LIMIT=VR(ax0_min_pos)-1
PRINT#5,"Axis 0 homing successful"

'Home Axis 1
'PRINT#5,"Homing Axis 1"
'BASE(ax1)
'SPEED=VR(ax1_jogspeed)
'ACCEL=VR(ax1_accel_decel)
'DECEL=VR(ax1_accel_decel)
'FS_LIMIT=2^30
'RS_LIMIT=-(2^30)
'ax1_home_found=FALSE
'VR(mask_limits)=TRUE 'Disable limits so that they can be monitored without tripping the system
'IF VR(ax1_home_config)=1 OR VR(ax1_home_config)=2 THEN 'Home in a positive direction
'    FORWARD
'    REPEAT
'        IF IN(in_ax1_pos_limit)=OFF THEN 'If limit switch is found first
'            CANCEL
'            WA(10)
'            WAIT IDLE
'            REVERSE 'reverse move looking for the home switch
'            WAIT UNTIL IN(in_ax1_home)=OFF
'            ax1_home_found=TRUE
'            WA(50)
'            CANCEL
'        ELSEIF IN(in_ax1_home)=OFF THEN 'If home switch is found first
'            ax1_home_found=TRUE
'            CANCEL
'        ENDIF
'   WA(10)
'    UNTIL ax1_home_found=TRUE
'    WA(10)
'    WAIT IDLE
'    VR(mask_limits)=FALSE 'Re-enable limit switch monitoring
'    DATUM(3) 'home axis in forward direction
'ELSEIF VR(ax1_home_config)=3 OR VR(ax1_home_config)=4 THEN 'Home in a negative direction
'    REVERSE
'    REPEAT
'        IF IN(in_ax1_rev_limit)=OFF THEN 'If limit switch is found first
'            CANCEL
'            WA(10)
'            WAIT IDLE
'            FORWARD 'forward move looking for the home switch
'            WAIT UNTIL IN(in_ax1_home)=OFF
'            ax1_home_found=TRUE
'            WA(50)
'            CANCEL
'        ELSEIF IN(in_ax1_home)=OFF THEN 'If home switch is found first
'            ax1_home_found=TRUE
'            CANCEL
'        ENDIF
'        WA(10)
'    UNTIL ax1_home_found=TRUE
'    WA(10)
'    WAIT IDLE
'    VR(mask_limits)=FALSE 'Re-enable limit switch monitoring
'    DATUM(4) 'home axis in reverse direction
'ELSE 'Zero the current position only
'    CANCEL
'ENDIF
'WA(10)
'WAIT IDLE
'VR(mask_limits)=FALSE 'Re-enable limit switch monitoring
'DEFPOS(0) 'Zero the position register
'FS_LIMIT=VR(ax1_max_pos)+1
'RS_LIMIT=VR(ax1_min_pos)-1
'PRINT#5,"Axis 1 homing successful"

'Home Axis 2
'PRINT#5,"Homing Axis 2"
'BASE(ax2)
'SPEED=VR(ax2_jogspeed)
'ACCEL=VR(ax2_accel_decel)
'DECEL=VR(ax2_accel_decel)
'FS_LIMIT=2^30
'RS_LIMIT=-(2^30)
'ax2_home_found=FALSE
'VR(mask_limits)=TRUE 'Disable limits so that they can be monitored without tripping the system
'IF VR(ax2_home_config)=1 OR VR(ax2_home_config)=2 THEN 'Home in a positive direction
'    FORWARD
'    REPEAT
'        IF IN(in_ax2_pos_limit)=OFF THEN 'If limit switch is found first
'            CANCEL
'            WA(10)
'            WAIT IDLE
'            REVERSE 'reverse move looking for the home switch
'            WAIT UNTIL IN(in_ax2_home)=OFF
'            ax2_home_found=TRUE
'            WA(50)
'            CANCEL
'        ELSEIF IN(in_ax2_home)=OFF THEN 'If home switch is found first
'            ax2_home_found=TRUE
'            CANCEL
'        ENDIF
'        WA(10)
'    UNTIL ax2_home_found=TRUE
'    WA(10)
'    WAIT IDLE
'    VR(mask_limits)=FALSE 'Re-enable limit switch monitoring
'    DATUM(3) 'home axis in forward direction
'ELSEIF VR(ax2_home_config)=3 OR VR(ax2_home_config)=4 THEN 'Home in a negative direction
'    REVERSE
'    REPEAT
'        IF IN(in_ax2_rev_limit)=OFF THEN 'If limit switch is found first
'            CANCEL
'            WA(10)
'            WAIT IDLE
'            FORWARD 'forward move looking for the home switch
'            WAIT UNTIL IN(in_ax2_home)=OFF
'            ax2_home_found=TRUE
'            WA(50)
'            CANCEL
'        ELSEIF IN(in_ax2_home)=OFF THEN 'If home switch is found first
'            ax2_home_found=TRUE
'            CANCEL
'        ENDIF
'        WA(10)
'    UNTIL ax2_home_found=TRUE
'    WA(10)
'    WAIT IDLE
'    VR(mask_limits)=FALSE 'Re-enable limit switch monitoring
'    DATUM(4) 'home axis in reverse direction
'ELSE 'Zero the current position only
'    CANCEL
'ENDIF
'WA(10)
'WAIT IDLE
'VR(mask_limits)=FALSE 'Re-enable limit switch monitoring
'DEFPOS(0) 'Zero the position register
'FS_LIMIT=VR(ax2_max_pos)+1
'RS_LIMIT=VR(ax2_min_pos)-1
'PRINT#5,"Axis 2 homing successful"

'Home Axis 3
'PRINT#5,"Homing Axis 3"
'BASE(ax3)
'SPEED=VR(ax3_jogspeed)
'ACCEL=VR(ax3_accel_decel)
'DECEL=VR(ax3_accel_decel)
'FS_LIMIT=2^30
'RS_LIMIT=-(2^30)
'ax2_home_found=FALSE
'VR(mask_limits)=TRUE 'Disable limits so that they can be monitored without tripping the system
'IF VR(ax3_home_config)=1 OR VR(ax3_home_config)=2 THEN 'Home in a positive direction
'    FORWARD
'    REPEAT
'        IF IN(in_ax3_pos_limit)=OFF THEN 'If limit switch is found first
'            CANCEL
'            WA(10)
'            WAIT IDLE
'            REVERSE 'reverse move looking for the home switch
'            WAIT UNTIL IN(in_ax3_home)=OFF
'            ax3_home_found=TRUE
'            WA(50)
'            CANCEL
'        ELSEIF IN(in_ax3_home)=OFF THEN 'If home switch is found first
'            ax3_home_found=TRUE
'            CANCEL
'        ENDIF
'        WA(10)
'    UNTIL ax3_home_found=TRUE
'    WA(10)
'    WAIT IDLE
'    VR(mask_limits)=FALSE 'Re-enable limit switch monitoring
'    DATUM(3) 'home axis in forward direction
'ELSEIF VR(ax3_home_config)=3 OR VR(ax3_home_config)=4 THEN 'Home in a negative direction
'    REVERSE
'    REPEAT
'        IF IN(in_ax3_rev_limit)=OFF THEN 'If limit switch is found first
'            CANCEL
'            WA(10)
'            WAIT IDLE
'            FORWARD 'forward move looking for the home switch
'            WAIT UNTIL IN(in_ax3_home)=OFF
'            ax3_home_found=TRUE
'            WA(50)
'            CANCEL
'        ELSEIF IN(in_ax3_home)=OFF THEN 'If home switch is found first
'            ax3_home_found=TRUE
'            CANCEL
'        ENDIF
'        WA(10)
'    UNTIL ax3_home_found=TRUE
'    WA(10)
'    WAIT IDLE
'    VR(mask_limits)=FALSE 'Re-enable limit switch monitoring
'    DATUM(4) 'home axis in reverse direction
'ELSE 'Zero the current position only
'    CANCEL
'ENDIF
'WA(10)
'WAIT IDLE
'VR(mask_limits)=FALSE 'Re-enable limit switch monitoring
'DEFPOS(0) 'Zero the position register
'FS_LIMIT=VR(ax3_max_pos)+1
'RS_LIMIT=VR(ax3_min_pos)-1
'PRINT#5,"Axis 3 homing successful"

VR(mask_limits)=FALSE 'Re-enable limit switch monitoring
VR(home_status)=home_complete 'Set the home complete latch
PRINT#5,"Datum sequence completed successfully"

RETURN


'-------------------------------------------------------------------------------
'Positive jog of each axis
'-------------------------------------------------------------------------------
forward_jog:
'Select the axis and setup the jog parameters
IF VR(motion_command)=ax0_jog_pos THEN
    jog_command=ax0_jog_pos
    jog_ax=ax0
    jog_ax_speed=VR(ax0_jogspeed)
    jog_ax_acc_dec=VR(ax0_accel_decel)
    jog_ax_pos=FS_LIMIT AXIS(ax0)-1
ELSEIF VR(motion_command)=ax1_jog_pos THEN
    jog_command=ax1_jog_pos
    jog_ax=ax1
    jog_ax_speed=VR(ax1_jogspeed)
    jog_ax_acc_dec=VR(ax1_accel_decel)
    jog_ax_pos=FS_LIMIT AXIS(ax1)-1
ELSEIF VR(motion_command)=ax2_jog_pos THEN
    jog_command=ax2_jog_pos
    jog_ax=ax2
    jog_ax_speed=VR(ax2_jogspeed)
    jog_ax_acc_dec=VR(ax2_accel_decel)
    jog_ax_pos=FS_LIMIT AXIS(ax2)-1
ELSEIF VR(motion_command)=ax3_jog_pos THEN
    jog_command=ax3_jog_pos
    jog_ax=ax3
    jog_ax_speed=VR(ax3_jogspeed)
    jog_ax_acc_dec=VR(ax3_accel_decel)
    jog_ax_pos=FS_LIMIT AXIS(ax3)-1
ELSE
    PRINT#5,"Jog+ error. No axis selected"
    GOTO jog_pos_end
ENDIF

'Setup the axis
BASE(jog_ax)
SPEED=jog_ax_speed 'Set to relevant axis jogspeed and accel_decel
ACCEL=jog_ax_acc_dec
DECEL=jog_ax_acc_dec

'Start axis forward motion to maximum axis position
PRINT#5,"Jogging Axis ";jog_ax;" Forward"
MOVEABS(jog_ax_pos)

'Move until either move completes or jog command is cancelled
WA(10)
WAIT UNTIL VR(motion_command)<>jog_command OR IDLE
jog_pos_end:
CANCEL
WAIT IDLE
PRINT#5,"Jog stopped"

RETURN


'-------------------------------------------------------------------------------
'Reverse jog of each axis
'-------------------------------------------------------------------------------
reverse_jog:
'Select the axis and setup the jog parameters
IF VR(motion_command)=ax0_jog_rev THEN
    jog_command=ax0_jog_rev
    jog_ax=ax0
    jog_ax_speed=VR(ax0_jogspeed)
    jog_ax_acc_dec=VR(ax0_accel_decel)
    jog_ax_pos=RS_LIMIT AXIS(ax0)+1
ELSEIF VR(motion_command)=ax1_jog_rev THEN
    jog_command=ax1_jog_rev
    jog_ax=ax1
    jog_ax_speed=VR(ax1_jogspeed)
    jog_ax_acc_dec=VR(ax1_accel_decel)
    jog_ax_pos=RS_LIMIT AXIS(ax1)+1
ELSEIF VR(motion_command)=ax2_jog_rev THEN
    jog_command=ax2_jog_rev
    jog_ax=ax2
    jog_ax_speed=VR(ax2_jogspeed)
    jog_ax_acc_dec=VR(ax2_accel_decel)
    jog_ax_pos=RS_LIMIT AXIS(ax2)+1
ELSEIF VR(motion_command)=ax3_jog_rev THEN
    jog_command=ax3_jog_rev
    jog_ax=ax3
    jog_ax_speed=VR(ax3_jogspeed)
    jog_ax_acc_dec=VR(ax3_accel_decel)
    jog_ax_pos=RS_LIMIT AXIS(ax3)+1
ELSE
    PRINT#5,"Jog- error. No axis selected"
    GOTO jog_rev_end
ENDIF

'Setup the axis
BASE(jog_ax)
SPEED=jog_ax_speed 'Set to relevant axis jogspeed and accel_decel
ACCEL=jog_ax_acc_dec
DECEL=jog_ax_acc_dec

'Start axis forward motion to maximum axis position
PRINT#5,"Jogging Axis ";jog_ax;" Reverse"
MOVEABS(jog_ax_pos)

'Move until either move completes or jog command is cancelled
WA(10)
WAIT UNTIL VR(motion_command)<>jog_command OR IDLE
jog_rev_end:
CANCEL
WAIT IDLE
PRINT#5,"Jog stopped"

RETURN


'-------------------------------------------------------------------------------
'Single absolute move on any axis or group of axes
'-------------------------------------------------------------------------------
move_absolute:

'Setup axis speeds
GOSUB speed_setup

PRINT#5,"Move absolute active"
IF VR(motion_axis)=1 THEN 'Just Axis 0
    BASE(ax0)
    MOVEABS(VR(ax0_move_pos))
ELSEIF VR(motion_axis)=2 THEN 'Just Axis 1
    BASE(ax1)
    MOVEABS(VR(ax1_move_pos))
ELSEIF VR(motion_axis)=4 THEN 'Just Axis 2
    BASE(ax2)
    MOVEABS(VR(ax2_move_pos))
ELSEIF VR(motion_axis)=8 THEN 'Just Axis 3
    BASE(ax3)
    MOVEABS(VR(ax3_move_pos))
ELSEIF VR(motion_axis)=1+2 THEN 'Axis 0,1
    BASE(ax0,ax1)
    MOVEABS(VR(ax0_move_pos),VR(ax1_move_pos))
ELSEIF VR(motion_axis)=1+4 THEN 'Axis 0,2
    BASE(ax0,ax2)
    MOVEABS(VR(ax0_move_pos),VR(ax2_move_pos))
ELSEIF VR(motion_axis)=1+8 THEN 'Axis 0,3
    BASE(ax0,ax3)
    MOVEABS(VR(ax0_move_pos),VR(ax3_move_pos))
ELSEIF VR(motion_axis)=2+4 THEN 'Axis 1,2
    BASE(ax1,ax2)
    MOVEABS(VR(ax1_move_pos),VR(ax2_move_pos))
ELSEIF VR(motion_axis)=2+8 THEN 'Axis 1,3
    BASE(ax1,ax3)
    MOVEABS(VR(ax1_move_pos),VR(ax3_move_pos))
ELSEIF VR(motion_axis)=4+8 THEN 'Axis 2,3
    BASE(ax2,ax3)
    MOVEABS(VR(ax2_move_pos),VR(ax3_move_pos))
ELSEIF VR(motion_axis)=1+2+4 THEN 'Axis 0,1,2
    BASE(ax0,ax1,ax2)
    MOVEABS(VR(ax0_move_pos),VR(ax1_move_pos),VR(ax2_move_pos))
ELSEIF VR(motion_axis)=1+2+8 THEN 'Axis 0,1,3
    BASE(ax0,ax1,ax3)
    MOVEABS(VR(ax0_move_pos),VR(ax1_move_pos),VR(ax3_move_pos))
ELSEIF VR(motion_axis)=1+4+8 THEN 'Axis 0,2,3
    BASE(ax0,ax2,ax3)
    MOVEABS(VR(ax0_move_pos),VR(ax2_move_pos),VR(ax3_move_pos))
ELSEIF VR(motion_axis)=2+4+8 THEN 'Axis 1,2,3
    BASE(ax1,ax2,ax3)
    MOVEABS(VR(ax1_move_pos),VR(ax2_move_pos),VR(ax3_move_pos))
ELSEIF VR(motion_axis)=1+2+4+8 THEN 'Axis 0,1,2,3
    BASE(ax0,ax1,ax2,ax3)
    MOVEABS(VR(ax0_move_pos),VR(ax1_move_pos),VR(ax2_move_pos),VR(ax3_move_pos))
ELSE
    PRINT#5,"Move abs error. No axis selected",VR(motion_axis)
ENDIF

WA(10)
'Wait for axis to complete move or for move to be cancelled
WAIT UNTIL IDLE OR VR(motion_command)<>move_abs
CANCEL
WAIT IDLE

PRINT#5,"Move absolute complete"

RETURN


'-------------------------------------------------------------------------------
'Single incremental move on any axis or group of axes
'-------------------------------------------------------------------------------
move_incremental:

'Setup axis speeds
GOSUB speed_setup

PRINT#5,"Move incremental active"
IF VR(motion_axis)=1 THEN 'Just Axis 0
    BASE(ax0)
    MOVE(VR(ax0_move_dist))
ELSEIF VR(motion_axis)=2 THEN 'Just Axis 1
    BASE(ax1)
    MOVE(VR(ax1_move_dist))
ELSEIF VR(motion_axis)=4 THEN 'Just Axis 2
    BASE(ax2)
    MOVE(VR(ax2_move_dist))
ELSEIF VR(motion_axis)=8 THEN 'Just Axis 3
    BASE(ax3)
    MOVE(VR(ax3_move_dist))
ELSEIF VR(motion_axis)=1+2 THEN 'Axis 0,1
    BASE(ax0,ax1)
    MOVE(VR(ax0_move_dist),VR(ax1_move_dist))
ELSEIF VR(motion_axis)=1+4 THEN 'Axis 0,2
    BASE(ax0,ax2)
    MOVE(VR(ax0_move_dist),VR(ax2_move_dist))
ELSEIF VR(motion_axis)=1+8 THEN 'Axis 0,3
    BASE(ax0,ax3)
    MOVE(VR(ax0_move_dist),VR(ax3_move_dist))
ELSEIF VR(motion_axis)=2+4 THEN 'Axis 1,2
    BASE(ax1,ax2)
    MOVE(VR(ax1_move_dist),VR(ax2_move_dist))
ELSEIF VR(motion_axis)=2+8 THEN 'Axis 1,3
    BASE(ax1,ax3)
    MOVE(VR(ax1_move_dist),VR(ax3_move_dist))
ELSEIF VR(motion_axis)=4+8 THEN 'Axis 2,3
    BASE(ax2,ax3)
    MOVE(VR(ax2_move_dist),VR(ax3_move_dist))
ELSEIF VR(motion_axis)=1+2+4 THEN 'Axis 0,1,2
    BASE(ax0,ax1,ax2)
    MOVE(VR(ax0_move_dist),VR(ax1_move_dist),VR(ax2_move_dist))
ELSEIF VR(motion_axis)=1+2+8 THEN 'Axis 0,1,3
    BASE(ax0,ax1,ax3)
    MOVE(VR(ax0_move_dist),VR(ax1_move_dist),VR(ax3_move_dist))
ELSEIF VR(motion_axis)=1+4+8 THEN 'Axis 0,2,3
    BASE(ax0,ax2,ax3)
    MOVE(VR(ax0_move_dist),VR(ax2_move_dist),VR(ax3_move_dists))
ELSEIF VR(motion_axis)=2+4+8 THEN 'Axis 1,2,3
    BASE(ax1,ax2,ax3)
    MOVE(VR(ax1_move_dist),VR(ax2_move_dist),VR(ax3_move_dist))
ELSEIF VR(motion_axis)=1+2+4+8 THEN 'Axis 0,1,2,3
    BASE(ax0,ax1,ax2,ax3)
    MOVE(VR(ax0_move_dist),VR(ax1_move_dist),VR(ax2_move_dist),VR(ax3_move_dist))
ELSE
    PRINT#5,"Move inc error. No axis selected",VR(motion_axis)
ENDIF

WA(10)
'Wait for axis to complete move or for move to be cancelled
WAIT UNTIL IDLE OR VR(motion_command)<>move_inc
CANCEL
WAIT IDLE

PRINT#5,"Move incremental complete"

RETURN

'-------------------------------------------------------------------------------
'Setup axis move speed and accel/decel rates
'-------------------------------------------------------------------------------
speed_setup:
BASE(ax0)
SPEED=VR(ax0_move_speed)
ACCEL=VR(ax0_accel_decel)
DECEL=VR(ax0_accel_decel)
BASE(ax1)
SPEED=VR(ax1_move_speed)
ACCEL=VR(ax1_accel_decel)
DECEL=VR(ax1_accel_decel)
BASE(ax2)
SPEED=VR(ax2_move_speed)
ACCEL=VR(ax2_accel_decel)
DECEL=VR(ax2_accel_decel)
BASE(ax3)
SPEED=VR(ax3_move_speed)
ACCEL=VR(ax3_accel_decel)
DECEL=VR(ax3_accel_decel)
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
run_seq:
PRINT#5,"Main sequence started"
'Setup axis speeds
GOSUB speed_setup

BASE(ax0)'select axis
SPEED=VR(main_seq_speed)'select desired speed

WAIT UNTIL IN(in_user_elevation_idle)=ON'wating for elevation motor to be idle

start_time=VR(master_time)'saving the starting time
FORWARD
WAIT UNTIL VR(master_time)>start_time+VR(main_seq_duration) 'waiting the appropriate duration
CANCEL(2) 'stopping motion
WAIT IDLE
OP(io_user_azimuth_idle, ON)

PRINT#5,"Main sequence stopped"
RETURN

'-------------------------------------------------------------------------------
'OLD Main routine
'-------------------------------------------------------------------------------
run_old_seq:
PRINT#5,"Main sequence started"
'Setup axis speeds
GOSUB speed_setup
BASE(ax0)
DEFPOS(0)


WAIT UNTIL IN(in_user_elevation_idle)=ON'wating for elevation motor to be idle

BASE(ax0)
TRIGGER
FOR i=0 TO VR(main_seq_lenght)-1 'cicle over the spin-scan list (elevation & spin speed & duration)
    SPEED=TABLE(VR(main_seq_speed)+i) 'setting the speed of the axis
    IF i=0 THEN 'if it is the first value of the list the axis must be started
        FORWARD AXIS(ax0)
    ELSEIF TABLE(VR(main_seq_alt)+i)<>TABLE(VR(main_seq_alt)+i-1) THEN 'it must be started also if the elevation has
'just changed because we stopped the azimuth from spinning in order to move the elevation axis in a safer manner
        FORWARD AXIS(ax0)
    ENDIF
    WAIT UNTIL MSPEED>0.99*TABLE(VR(main_seq_speed)+i) 'waiting for the axis to reach at least 99% of the desiderd speed
    WA(TABLE(VR(main_seq_duration)+i)) 'waiting the required duration of the scan
    IF i=VR(main_seq_lenght)-1 THEN 'if this is the last element of the list stop the motion and turn on the idle I/O
        CANCEL (2)
        WAIT IDLE
        OP(io_user_azimuth_idle,ON)
    ELSEIF TABLE(VR(main_seq_alt)+i)<>TABLE(VR(main_seq_alt)+i+1) THEN 'if we need to change the elevation of the
'telescope stop the motion, turn on the idle I/O and wait for the elevation to be changed
        CANCEL (2)
        WAIT IDLE
        OP(io_user_azimuth_idle,ON)
        WAIT UNTIL IN(in_user_elevation_idle)=ON
    ENDIF
NEXT i
TRIGGER

PRINT#5,"Main sequence stopped"
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
    'if the scan contains the axis wrapping limit (es Az from 170deg->-170deg passing from 179deg and -179deg)
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
DECEL=ACCEL
OP(io_user_azimuth_idle, ON)

PRINT#5,"Raster Scan stopped"
RETURN

'-------------------------------------------------------------------------------
'Drone tracking
'-------------------------------------------------------------------------------
drone_tracking:
PRINT#5,"Drone tracking started"
'Setup axis speeds
GOSUB speed_setup
BASE(ax0)
VR(drone_target_pos)=-1 'initializing the VR variable where the target position will be stored
current_pos=-1 'variable that keeps track of which pointing of the list the the one that is currently being used
OP(io_user_azimuth_idle,ON) 'signals that the system is ready to recive an instruction
WAIT UNTIL VR(drone_target_pos)<>-1 'waiting until the desired pointing from the list is specified

WHILE VR(drone_target_pos)<>99: 'until the target position is different than the switch-off value
    IF current_pos<>VR(drone_target_pos) THEN 'if we are not in the desired position
        OP(io_user_azimuth_idle,OFF) 'signal that the sistem is not idle

        'if the desired pos - current pos is more than 180deg the motor should roll around the -180deg,180deg limit
        PRINT ABS(TABLE(VR(drone_table_start)+VR(drone_target_pos))-MPOS), ">", VR(ax0_units_rev)/2
        PRINT ABS(TABLE(VR(drone_table_start)+VR(drone_target_pos))-MPOS)>VR(ax0_units_rev)/2
        IF ABS(TABLE(VR(drone_table_start)+VR(drone_target_pos))-MPOS)>(VR(ax0_units_rev)/2) THEN:
            PRINT ABS(TABLE(VR(drone_table_start)+VR(drone_target_pos))-MPOS), ">", VR(ax0_units_rev)/2
            PRINT ABS(TABLE(VR(drone_table_start)+VR(drone_target_pos))-MPOS)>VR(ax0_units_rev)/2
            PRINT "in"
            IF MPOS>=0 THEN
                MOVE(-MPOS-1)
            ELSE
                MOVE((180+MPOS)+1)
            ENDIF
        MOVEABS (TABLE(VR(drone_table_start) + VR(drone_target_pos))) 'move to the desired position
        WAIT IDLE
        current_pos=VR(drone_target_pos) 'update the variable
        OP(io_user_azimuth_idle,ON)
    ENDIF
WEND
OP(io_user_azimuth_idle,OFF)


PRINT#5,"Drone tracking stopped"
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
FOR pos=1 TO VR(avoid_sun_len_table)-1: 'cicling over the positions of the trajectory
    'if the step of the trajectory changes Alt and not Az wait for Alt motor to be idle
    IF TABLE(VR(avoid_sun_az_table_start)+pos)=TABLE(VR(avoid_sun_az_table_start)+pos-1) THEN
        PRINT #5, "Wait"
        PRINT #5, TABLE(VR(avoid_sun_az_table_start)+pos-1), "--->", TABLE(VR(avoid_sun_az_table_start)+pos)
        WA(20)
        WAIT UNTIL IN(in_user_elevation_idle)
    ELSE 'if instead the step of the trajectory changes Az
        PRINT #5, "Run"
        PRINT #5, TABLE(VR(avoid_sun_az_table_start)+pos-1), "--->", TABLE(VR(avoid_sun_az_table_start)+pos)
        OP(io_user_azimuth_idle, OFF) 'switch off idle imput
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




