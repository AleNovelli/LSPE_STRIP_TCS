import redis
import msgpack

#answerback = {'type':"answerback", 'outcome':True, output=""}

move_to = {'type':"command", 'motion':"move to",
        "parameters":{"alt":40,"az":100}}

calibration = {'type':"command", 'motion':"calibration",
        "parameters":{"pointing_list":[[80.0, 120, 10], [60.0, 100, 20]]}}

nominal =  {'type':"command", 'motion':"nominal",
        "parameters":{"alt": 40,"speed": 1,"duration": 40}}

raster = {'type':"command", 'motion':"raster",
        "parameters":{"alt": 40,"duration": 20,"speed": 6,"accel_to_decel_frac": 2,"az_min": 15,"az_max": 18}}



motion=move_to
print(msgpack.packb(motion))


client = redis.Redis(host="localhost", port=6379)

client.publish('tcs.motion.command', msgpack.packb(motion))

