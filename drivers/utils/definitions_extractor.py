#**********************************************************************************************************
#PROJECT: LSPE-STRIP
#
#Run From: CLIENT
#Program: "definitions_extractor.py"
#
#Description:
#   This program copies the definitions, I/O and variable allocations of the Trio-Controllers which are saved in the
#   program "COMMON.bas" (there is one for each controller), puts them to a python readable format and saves them in
#   "/controller_definitions/azimuth_definitions.py" and "/controller_definitions/elevation_definitions.py".
#   This program MUST BE RUN each time that the firmware of the controllers is updated.
#
#**********************************************************************************************************

#paths to the firmware of the two controllers
az_def_path=r"C:\Users\aless\OneDrive\Documenti\Motion Perfect v5.0\Projects\LSPE_STRIP\Azimuth"
#alt_def_path=r"C:\Users\aless\OneDrive\Documenti\Condivisa\Projects\LSPE_STRIP\Elevation"
alt_def_path=r"C:\Users\aless\OneDrive\Documenti\Motion Perfect v5.0\Projects\LSPE_STRIP\Elevation"

#I open and read the COMMON.bas files in the firmware projects
az_def_file = open(az_def_path+r"\COMMON.bas","r")
alt_def_file = open(alt_def_path+r"\COMMON.bas","r")
az_def = az_def_file.read()
alt_def = alt_def_file.read()
az_def_file.close()
alt_def_file.close()

#I replace the TrioBASIC comments with python comments
az_def = az_def.replace("'","#")
alt_def = alt_def.replace("'","#")

#I find where the descriptions of the programs end
az_end_desc = len(az_def) -az_def[::-1].find("*")
alt_end_desc = len(alt_def) -alt_def[::-1].find("*")
#I remove the description of the programs
az_def=az_def[az_end_desc:]
alt_def=alt_def[alt_end_desc:]

#I open the new files for the client
az_def_file_py = open(r"../controller_definitions/azimuth_definitions.py", "w")
alt_def_file_py = open(r"../controller_definitions/elevation_definitions.py", "w")

#this is the description of the python files (to be included at the beginning)
az_desc="""#**********************************************************************************************************
#PROJECT: LSPE-STRIP
#
#Run From: CLIENT
#Program: "/controller_definitions/azimuth_definitions.py"
#
#Description:
#   This file is not run from any other program but it must be included at the beginning of each program.
#   This file was created by "definitions_extractor.py", a new instance of this file should be created each time 
#   the firmware of the azimuth controller is updated. This file contains a python friendly copy of all the definitions,
#   I/O and variable allocations of the Azimuth Trio-Controller.
#
#**********************************************************************************************************"""
alt_desc="""#**********************************************************************************************************
#PROJECT: LSPE-STRIP
#
#Run From: CLIENT
#Program: "/controller_definitions/elevation_definitions.py"
#
#Description:
#   This file is not run from any other program but it must be included at the beginning of each program.
#   This file was created by "definitions_extractor.py", a new instance of this file should be created each time 
#   the firmware of the azimuth controller is updated. This file contains a python friendly copy of all the definitions,
#   I/O and variable allocations of the Elevation Trio-Controller.
#
#**********************************************************************************************************"""
#I write the description and the definitions on the file
az_def_file_py.write(az_desc+az_def)
alt_def_file_py.write(alt_desc+alt_def)
az_def_file_py.close()
alt_def_file_py.close()


