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
const char* sub_topic = "20520597/pub";

WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

int lastFlameStatus = 0; // Initial value
float lastGasPPM = 0;     // Initial value

const int buzzerPin = 22;  // Buzzer connected to pin 22

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
    mqttClient.subscribe(sub_topic);
  } else {
    Serial.print("Failed to connect to MQTT broker, error = ");
    Serial.println(mqttClient.state());
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  // Convert the payload byte array to a String
  String receivedData = "";
  for (int i = 0; i < length; i++) {
    receivedData += (char)payload[i];
  }
  
  // Process the received data
  Serial.print("Received data: ");
  Serial.println(receivedData);
  
  String temperature = receivedData.substring(0, receivedData.indexOf(','));
  receivedData = receivedData.substring(receivedData.indexOf(',') + 1);
  String gasPPM = receivedData.substring(0, receivedData.indexOf(','));
  receivedData = receivedData.substring(receivedData.indexOf(',') + 1);
  String flameStatus = receivedData;
  
  //String to data
  float temperatureValue = temperature.toFloat();
  float gasPPMValue = gasPPM.toFloat();
  int flameStatusValue = flameStatus.toInt();
  
  // Use the extracted values as needed in your code
  checkFlameAndGasPPM(flameStatusValue, gasPPMValue);
  // For example, you can store them in variables or perform actions based on the values
}

void setupBuzzer() {
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, HIGH);  // Turn off the buzzer initially
}

void buzzerOn() {
  digitalWrite(buzzerPin, LOW);  // Turn on the buzzer
  Serial.println("ON");
}

void buzzerOff() {
  digitalWrite(buzzerPin, HIGH);  // Turn off the buzzer
  Serial.println("OFF");
}

void checkFlameAndGasPPM(int flame, int gasPPM) {
  static bool buzzerState = true;  // Current state of the buzzer (on/off)
  
  if (flame == 1 && gasPPM < 40) {
    if (buzzerState) {
      buzzerOff();  // Turn off the buzzer
      buzzerState = false;
    }
  } else {
    if (!buzzerState) {
      buzzerOn();  // Turn on the buzzer
      buzzerState = true;
    }
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  connectToWiFi();
  connectToMQTTBroker();
  setupBuzzer();
  mqttClient.setCallback(callback);
}

void loop() {
  if (!mqttClient.connected()) {
    connectToMQTTBroker();
  }
  mqttClient.loop();
}
