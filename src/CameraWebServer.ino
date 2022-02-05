#include "conf.h"
#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "BluetoothSerial.h"
#include "ArduinoJson.h"


// Select camera model
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#define LISTEN_PORT 81 // http://<IP or DDNS>:<LISTEN_PORT>/?percentage=<0..100>
#define LEASE_DURATION 36000  // seconds
#define FRIENDLY_NAME "FriendlyDevice"  // this name will appear in your router port forwarding section
#define SERVER_ADD "http://192.168.13.155:5000/intahon/us-central1/app"


#include "camera_pins.h"

void startCameraServer();

/* TinyUPnP tinyUPnP(20000); */
BluetoothSerial SerialBT;
String macAddress;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile int sendLogCounter;
int64_t duration = 0;

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

    camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

  WiFiClass wifi = WiFiClass();
  if(!ssid || !password){
    Serial.println("Waiting for pairing and infos");
    SerialBT.begin("ESP32 CAM"); //Bluetooth device name
    Serial.println("Bluetooth enabled");
  }
  while(wifi.status() != WL_CONNECTED){
    while(!ssid || !password){
      delay(1000);
      Serial.println("Waiting for infos");
      if (SerialBT.available()) {
        String btContent = SerialBT.readString();
        Serial.println(btContent);
        DynamicJsonDocument doc(300);
        deserializeJson(doc, btContent);
        client_token = doc["token"];
        ssid = doc["ssid"];
        password = doc["pwd"];
      }
    }
    wifi.begin(ssid, password);
    int timeOutCounter = 20;
    while (wifi.status() != WL_CONNECTED && timeOutCounter > 0) {
      delay(500);
      Serial.print(".");
      timeOutCounter--;
    }
    if(timeOutCounter == 0){
      ssid = NULL;
      password = NULL;
    }
  }
  
  macAddress = wifi.macAddress();
  HTTPClient http;
  http.begin(SERVER_ADD "/object/connexion");
  http.addHeader("Content-Type", "application/json");
  String payload = String("{\"address_ip\": \"" + wifi.localIP().toString() + "\",\"address_mac\": \"" + macAddress + "\", \"client_token\": \"" + client_token + "\", \"port\" : " + 81 + " }");
  Serial.println(payload);
  int res = http.POST(payload);
  Serial.println(http.getString());
  if(res<=0){
    Serial.println("Error during server connexion: ");
    Serial.println(http.getString());
  }
  // Free resources
  http.end();

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(wifi.localIP());
  Serial.println("' to connect");
}

void loop() {
  // If Stream Ended send Log
  if (sendLogCounter > 0){
    // Read Duration
    portENTER_CRITICAL(&timerMux);
    sendLogCounter--;
    portEXIT_CRITICAL(&timerMux);
    HTTPClient http;
    Serial.print("Stream Ended duration : ");
    Serial.println(duration);
    char durationStr[20];
    itoa(duration, durationStr, 10);
    String payload = String("{\"address_mac\": \"" + macAddress + "\", \"client_token\": \"" + client_token + "\", \"duration\" : " + durationStr + " }");
    http.begin(SERVER_ADD "/object/logging");
    http.addHeader("Content-Type", "application/json");
    int res = http.POST(payload);
    Serial.println("Logger result : " + res );
    Serial.println(http.getString());
    if(res<=0){
      Serial.println("Error during server connexion.");
    }
    // Free resources
    http.end();
  }
  // put your main code here, to run repeatedly:
  delay(100);
}