'**********************************************************************************************************
'System Designed, Built, and Programmed by -
'            MICROMECH SYSTEMS LIMITED      +44(0)1376 333333
'            A Force in Motion              www.micromech.com
'----------------------------------------------------------------------------------------------------------
'Project: MSS-K 1-AXIS Standard Servo Control System
'Customer: Varies
'Author: Mick Stone
'Controller: MC4N-ECAT
'Drives: Kollmorgen AKD
'Program: STARTUP_DELAY
'
'Description
'   This program is set to run on startup only as Process 0, immediately after the MC_CONFIG has completed.
'
'Program Change Log
'   v1.000  22/11/2017  First Draft
'
'If the software is changed update:-
'   The master Software Version History in SYSTEM_INFO
'   The Program Change Log in this program
'   The VR(software_version) in STARTUP
'**********************************************************************************************************

INCLUDE "COMMON"

'Time to allow AKD to boot up
FOR counter=40 TO 0 STEP -1
    PRINT#5,counter [2,0];" Seconds until connect"
    WA(1000)
NEXT counter

'Start STARTUP process
IF PROC_STATUS PROC(1) = 0 THEN RUN "STARTUP",1
WAIT UNTIL PROC_STATUS PROC(1)=1

STOP

