#include <MQUnifiedsensor.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// REPLACE WITH THE RECEIVER'S MAC Address
uint8_t broadcastAddress[] = {0x08, 0x3A, 0x8D, 0x95, 0x99, 0x74};

typedef struct {
  int id; // must be unique for each sender board
  float x;
  int y;
} SensorData;

SensorData myData;

#define board "ESP32"
#define voltageResolution 3.3
#define analogPin 36 // Analog input 0 of your ESP32
#define sensorType "MQ-6" // MQ6
#define adcBitResolution 12 // For ESP32
#define ratioMQ6CleanAir 10 // RS / R0 = 10 ppm

MQUnifiedsensor MQ6(board, voltageResolution, adcBitResolution, analogPin, sensorType);

float gasPPM;

esp_now_peer_info_t peerInfo;

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

int32_t getWiFiChannel(const char *ssid) {
  int32_t channel = 0;
  
  if (int32_t n = WiFi.scanNetworks()) {
    for (uint8_t i = 0; i < n; i++) {
      if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
        channel = WiFi.channel(i);
        break;
      }
    }
  }
  
  return channel;
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  int32_t channel = getWiFiChannel("A7.15");
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(onDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
  MQ6.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ6.setA(1009.2);
  MQ6.setB(-2.35);
  
  MQ6.init();

  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i++) {
    MQ6.update();
    calcR0 += MQ6.calibrate(10);
    Serial.print(".");
  }
  MQ6.setR0(calcR0 / 10);
  Serial.println("  done!.");
  
  if(isinf(calcR0)) {
    Serial.println("Warning: Connection issue, R0 is infinite (Open circuit detected). Please check your wiring and supply.");
    while(1);
  }
  if(calcR0 == 0) {
    Serial.println("Warning: Connection issue found, R0 is zero (Analog pin shorts to ground). Please check your wiring and supply.");
    while(1);
  }

  MQ6.serialDebug(true);
}

void loop() {
  MQ6.update();
  gasPPM = MQ6.readSensor();

  // Set values to send
  myData.id = 2;
  myData.x = gasPPM;
  myData.y = 0; // You can change this value if needed

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  
  delay(1000); // Sampling frequency
}
