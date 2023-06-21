#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_AHTX0.h>
#include <esp_wifi.h>
// REPLACE WITH THE RECEIVER'S MAC Address
uint8_t broadcastAddress[] = {0x08, 0x3A, 0x8D, 0x95, 0x99, 0x74};

typedef struct {
  int id;   // must be unique for each sender board
  float x;
  int y;
} SensorData;

SensorData myData;
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

Adafruit_AHTX0 aht20;
float temperature;
int flameSensor = 19;
int flameStatus;

void readSensorData() {
  if (!aht20.begin()) {
    Serial.println("Could not find AHT20 sensor!");
    while (1);
  }
  
  sensors_event_t temperatureEvent;
  aht20.getEvent(NULL, &temperatureEvent);
  temperature = temperatureEvent.temperature;
  
  pinMode(flameSensor, INPUT);
  flameStatus = digitalRead(flameSensor);
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
}

void sendData() {
  myData.id = 1;
  myData.x = temperature;
  myData.y = flameStatus;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
}

void loop() {
  readSensorData();
  sendData();
  
  delay(1000);
}
