#include <esp_now.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>

// Wi-Fi credentials
const char* ssid = "A7.15";
const char* password = "nhatquang48123";

// MQTT broker details
const char* mqttBroker = "c09422c3110542c394f85fd6edf9bcf5.s2.eu.hivemq.cloud";
const int mqttPort = 8883;
const char* mqttUsername = "20520597";
const char* mqttPassword = "Brs13125";
const char* pub_topic = "20520597/pub";
const char* sub_topic = "20520597/sub";

WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

// Structure for receiving data
typedef struct {
  int id;
  float x;
  int y;
} SensorData;

// Variables to store last readings
float lastTemperature = 0;
int lastFlameStatus = 1;
float lastGasPPM = 0;

// Function prototypes
void publishData(const char* payload);
void connectToWiFi();
void connectToMQTTBroker();
void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status);
void onDataRecv(const uint8_t* mac_addr, const uint8_t* incomingData, int len);

void publishData(const char* payload) {
  mqttClient.publish(pub_topic, payload);
}

void connectToWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to Wi-Fi network...");
  }
  Serial.println("Wi-Fi connected!");
}

void connectToMQTTBroker() {
  wifiClient.setInsecure();
  mqttClient.setServer(mqttBroker, mqttPort);

  String clientId = "ESP32-";
  clientId += String(random(0xffff), HEX);

  if (mqttClient.connect(clientId.c_str(), mqttUsername, mqttPassword)) {
    Serial.println("Connected to MQTT broker!");
  } else {
    Serial.print("Failed to connect to MQTT broker, error = ");
    Serial.println(mqttClient.state());
  }
}

void onDataRecv(const uint8_t* mac_addr, const uint8_t* incomingData, int len) {
  SensorData data;
  memcpy(&data, incomingData, sizeof(data));

  if (data.id == 1) {
    lastTemperature = static_cast<float>(data.x);
    lastFlameStatus = static_cast<float>(data.y);
  } else if (data.id == 2) {
    lastGasPPM = static_cast<float>(data.x);
  }

  // Print latest values
  Serial.println();
  Serial.print("Temperature: ");
  Serial.println(lastTemperature);
  Serial.print("Gas PPM: ");
  Serial.println(lastGasPPM);
  Serial.print("Flame Status: ");
  Serial.println(lastFlameStatus);
}

void setup() {
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  connectToWiFi();
  connectToMQTTBroker();

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callback function for received data
  esp_now_register_recv_cb(onDataRecv);
}

void loop() {
  mqttClient.loop();

  // Prepare data to publish
  String data = String(lastTemperature) + "," + String(lastGasPPM) + "," + String(lastFlameStatus);

  // Publish data if MQTT client is connected, otherwise reconnect
  if (mqttClient.connected()) {
    publishData(data.c_str());
  } else {
    connectToMQTTBroker();
  }

  delay(1000);
}