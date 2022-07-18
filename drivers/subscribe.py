import redis
import msgpack

client = redis.Redis(host="localhost", port=6379)

p = client.pubsub()

p.subscribe("channel1")
while True:
    message = p.get_message()
    if message and message["data"]!=1:
        print(msgpack.unpackb(message["data"]))
        message= None