from flask import Flask, jsonify, render_template
import paho.mqtt.client as paho
from paho import mqtt

app = Flask(__name__)

"""
# MQTT broker configuration 1883
broker_address = "broker.hivemq.com"
client_mqtt = paho.Client()
client_mqtt.connect(broker_address)
mqtt_topic = "newtesting"
"""

# MQTT broker configuration 8883
broker_address = "c09422c3110542c394f85fd6edf9bcf5.s2.eu.hivemq.cloud"
client_mqtt = paho.Client(client_id="", userdata=None, protocol=paho.MQTTv5)
client_mqtt.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
client_mqtt.username_pw_set(username="20520597", password="Brs13125")
client_mqtt.connect(broker_address, port=8883)
mqtt_topic = "20520597/pub" 

# Khởi tạo temperature và humidity
temperature = 0
gas = 0
fireOccurrence = 1


# Trang chủ của ứng dụng web
@app.route("/")
def index():
    return render_template("index.html")


# API trả về dữ liệu nhiệt độ và độ ẩm mới nhất
@app.route("/api/data")
def get_data():
    # Gửi yêu cầu lấy dữ liệu mới nhất tới MQTT broker
    client_mqtt.publish("antgroup0905/get_data", "get_data")
    # Trả về dữ liệu dưới dạng JSON
    return jsonify({"temperature": temperature, "gas": gas, "fireOccurrence": fireOccurrence})


# Hàm xử lý khi nhận được dữ liệu từ MQTT broker
def on_message(client, userdata, message):
    global temperature, gas, fireOccurrence
    # Get data from message payload
    data = message.payload.decode()
    temperature, gas, fireOccurrence = data.split(",")
    temperature = float(temperature)
    gas = float(gas)


# Thiết lập callback function cho MQTT client
client_mqtt.on_message = on_message
client_mqtt.subscribe(mqtt_topic)

if __name__ == "__main__":
    # Start MQTT client
    client_mqtt.loop_start()
    # Start Flask app
    app.run(debug=True)
