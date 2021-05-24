#include "WiFi.h"
#include <U8x8lib.h>
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <U8g2lib.h>

const float MAX_CURSOR_ACC = 16; 

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

  // U8G2_SH1107_64X128_F_4W_HW_SPI u8g2(U8G2_R3, /* cs=*/ 14, /* dc=*/ 27, /* reset=*/ 33);

#define WIFI_SSID "PUTTICHAMP"
#define WIFI_PASSWORD "p9210764"

#define MQTT_HOST IPAddress(203, 185, 64, 8) 	


#define MQTT_PORT 1883

#define MQTT_PUB_WIFI "esp32/test/Scan-wifi"

String ssid;
String rssi;


AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
unsigned long previousMillis =100; 
const long interval = 1000000;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    connectToMqtt();
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("WiFi lost connection");
    xTimerStop(
        mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
    xTimerStart(wifiReconnectTimer, 0);
    break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  // u8g2.begin();
  // u8g2.drawStr(2, 20, "Smart Maintenance V.1");
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);
  delay(100);

  Serial.println("WiFi Netwoek Scan Started");

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer =xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  connectToWifi();

}

void loop() {

  Serial.print(": ");
  Serial.print(WIFI_SSID);
  Serial.print(": ");
  Serial.print(WiFi.RSSI());
  
      ssid = WIFI_SSID;
      rssi = WiFi.RSSI();
 

    String json = "{";
      json += "\"WiFi_SSID\": \""+ssid+"\"";
      json += ", \"WiFi_RSSI\": "+String(rssi);
      json += "}";
  

  uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_WIFI, 2, true, json.c_str()); 
  Serial.printf("Publishing on topic %s at QoS 2, packetId: ", MQTT_PUB_WIFI);
  Serial.println(packetIdPub1);
 
  Serial.println(json.c_str());
  delay(2000);

}