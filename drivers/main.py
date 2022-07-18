
import redis
import msgpack
import subprocess
import os
import signal
import time
import json
import traceback

#from utils.redis_definitions import *

perm_processes = None
perm_files_list=None

driver_params=json.load(open("./configuration/TCS_driver_parameters.json"))

redis_ip=driver_params["ip&ports"]["redis_ip"]
redis_port=driver_params["ip&ports"]["redis_port"]

tcs_channel_base=driver_params["redis_definitions"]["base"]
channel_motion_commands=driver_params["redis_definitions"]["channels"]["motion_commands"]
channel_motion_answerbacks=driver_params["redis_definitions"]["channels"]["motion_answerbacks"]
current_motion_var=driver_params["redis_definitions"]["variables"]["current_motion"]
channel_fault_alert=driver_params["redis_definitions"]["channels"]["fault_alert"]


try:
    #connect to the REDIS server
    redis_client = redis.Redis(host=redis_ip, port=redis_port)
    pub_sub=redis_client.pubsub()
    pub_sub.psubscribe(tcs_channel_base+"*")

    #indicates that we are connected to the simulators and not to the real Controllers
    simulator=False

    # get a list of all the files that must be running permanently
    perm_files_list=os.listdir("./permanent")
    perm_files_list.remove("__init__.py")

    # function to determine if a file is a modbus proxy
    def file_is_proxy(file_name):
        if "proxy" in file_name:
            return 1
        else:
            return 0

    #order the file list so that the modbus proxies are launched first (otherwise the other might fail)
    perm_files_list.sort(key=file_is_proxy, reverse=True)

    #Se sono collegato ai simulatori non devo gestire il proxy di elevazione
    if simulator:
        perm_files_list.remove("proxy_alt.py")
        #perm_files_list.remove("encoder_sampler_alt.py")

    #lancio i permanent process e salvo i loro subprocess in una lista
    perm_processes=[]
    for file in perm_files_list:
        cmd="python "+file
        proc=subprocess.Popen(cmd.split(), stdout=subprocess.PIPE, cwd="./permanent")
        time.sleep(.1)
        perm_processes.append(proc)

    #se sono collegato ai simulatori lancio il simulatore del master clock e
    # resetto il fault dovuto al fatto che non ero collegato al master clock
    if simulator==True:
        cmd="python master_clock_sim.py"
        #mc_proc = subprocess.Popen(cmd.split(), stdout=subprocess.PIPE, cwd="./utils")
        #time.sleep(3)
        #cmd = "python fault_acknowledge.py"
        #ackn_proc = subprocess.Popen(cmd.split(), stdout=subprocess.PIPE)
        #ackn_proc.communicate()


    motion_comands={"calibration": "calibration_sequence.py",
                    "move to": "move_to_start.py",
                    "nominal": "nominal_survey.py",
                    "raster":"raster_scan.py",
                    "park": "move_to_start.py",
                    "stop": "MISSING<-----------!!!!!"
                    }

    redis_client.set(current_motion_var, "none")
    motion_proc=None
    print("Connection to TCS made, ready to accept comands.")

    while True:

        #controllo che tutti i processi permanenti siano ancora in funzione
        for proc_name, proc in zip(perm_files_list, perm_processes):
            if proc.poll()!=None:
                raise Exception("PERMANENT SUBPROCESS {} HAS CRASHED!!!".format(proc_name))

        #ricevo eventuali messaggi
        message = pub_sub.get_message()
        if message and message["data"] != 1:#<--- rivedere da dove arriva

            message_channel=(message["channel"]).decode("utf-8")

            if message_channel==channel_motion_commands:
                command = msgpack.unpackb(message["data"])
                curr_motion=(redis_client.get(current_motion_var)).decode("utf-8")
                #print("MOTION:   ", curr_motion)
                if curr_motion=="none":
                    redis_client.set(current_motion_var, command["motion"])
                    redis_client.publish(channel_motion_answerbacks, "motion '" + command["motion"] + "' RUNNING")

                    print("motion '" + command["motion"] + "' LAUNCHED")
                    cmd= "python "+motion_comands[command["motion"]]+" "+json.dumps(command["parameters"]).replace(" ", "")
                    print(cmd)
                    motion_proc = subprocess.Popen(cmd.split(), cwd="./motion", shell=True) #Per Windows

                    #Per LINUX
                    #motion_proc = subprocess.Popen(cmd.split(), cwd="./motion", shell=True , preexec_fn=os.setsid)
                else:
                    redis_client.publish(channel_motion_answerbacks,  "new motion '" + command["motion"] + "' FAILED")
                    redis_client.publish(channel_motion_answerbacks, "Running: "+ curr_motion)
                    print("motion '" + command["motion"] + "' FAILED\n"+"Running: "+ curr_motion)

            elif message_channel==channel_fault_alert:
                if motion_proc:
                    if motion_proc.poll()==None:
                        #os.killpg(os.getpgid(pro.pid), signal.SIGTERM) <---- Per LINUX
                        #per Windows
                        print("PROCESS KILLEd")
                        kill=subprocess.Popen("TASKKILL /F /PID {pid} /T".format(pid=motion_proc.pid))
                        kill.wait()

except redis.exceptions.ConnectionError:
    traceback.print_exc()
    print("\nERROR:\nFailing to connect to the Redis server:\nAttempting connection to '{}:{:d}'".format(redis_ip, redis_port))
except Exception as exception:
    print("\nGENERAL ERROR:")
    #if the list with all the permanent processes has already been defined
    if perm_processes and perm_files_list:
        for  process, proc_name in zip(perm_processes, perm_files_list):
            print("-------\nProcess: {}".format(proc_name))
            if process.poll()!=None:
                print("Process was still running")
            else:
                print("Process seemed to have crashed")
            process.terminate()
            output, error = process.communicate()
            if output:
                print("Output:")
                print(output.decode("utf-8"))
            if error:
                print("Errors:")
                print(error.decode("utf-8"))
    traceback.print_exc()
