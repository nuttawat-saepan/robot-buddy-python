import paho.mqtt.client as mqtt

BROKER = "192.168.31.58"
PORT = 1883


def on_connect(client, userdata, flags, rc):
    print(f"[MQTT] connected rc={rc}")

    # subscribe ทุกอย่าง
    client.subscribe("navigate_to_pose")
    print("[MQTT] subscribed to #")


def on_message(client, userdata, msg):
    print("\n====================")
    print("TOPIC :", msg.topic)
    print("RAW   :", msg.payload)

    try:
        text = msg.payload.decode()
        print("TEXT  :", text)
    except:
        print("decode failed")


def on_disconnect(client, userdata, rc):
    print(f"[MQTT] disconnected rc={rc}")


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.on_disconnect = on_disconnect

print("[MQTT] connecting...")
client.connect(BROKER, PORT, 60)

client.loop_forever()