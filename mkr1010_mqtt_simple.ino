// Duncan Wilson Oct 2025 - v1 - MQTT messager to vespera

// works with MKR1010

#include <SPI.h>
#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <utility/wifi_drv.h>   // library to drive to RGB LED on the MKR1010
#include <Wire.h>
#include <BH1750.h>
BH1750 lightMeter(BH1750::CONTINUOUS_HIGH_RES_MODE);

/*
**** please enter your sensitive data in the Secret tab/arduino_secrets.h
**** using format below
#define SECRET_SSID "ssid name"
#define SECRET_PASS "ssid password"
#define SECRET_MQTTUSER "user name - eg student"
#define SECRET_MQTTPASS "password";
 */
const char* ssid          = "CE-Hub-Student";
const char* password      = "casa-ce-gagarin-public-service";
const char* ssid1         = "Hyperoptic Fibre 3343 5GHz";
const char* password1     = "icdkxfNjhpK4F3";
const char* mqtt_username = "student";
const char* mqtt_password = "ce2021-mqtt-forget-whale";
const char* mqtt_server   = "mqtt.cetools.org";
const int mqtt_port       = 1884;

// create wifi object and mqtt object
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Make sure to update your lightid value below with the one you have been allocated
String lightId = "7"; // the topic id number or user number being used.

// Here we define the MQTT topic we will be publishing data to
String mqtt_topic = "student/CASA0014/luminaire/" + lightId;            
String clientId = ""; // will set once i have mac address so that it is unique

// NeoPixel Configuration - we need to know this to know how to send messages 
// to vespera 
const int num_leds = 72;
const int payload_size = num_leds * 3; // x3 for RGB

// Create the byte array to send in MQTT payload this stores all the colours 
// in memory so that they can be accessed in for example the rainbow function
byte RGBpayload[payload_size];

float luxEMA = -1.0;                 // Exponential Moving Average
uint8_t lastBr = 255;                 // Last bright
unsigned long lastSend = 0;           // Last time
const float ALPHA = 0.20;             // Smoothing
const uint8_t BASE_R = 255, BASE_G = 0, BASE_B = 0; //red light


void setup() {
  Serial.begin(115200);
  //while (!Serial); // Wait for serial port to connect (useful for debugging)
  Serial.println("Vespera");

  Wire.begin();
  if (!lightMeter.begin()) {
    Serial.println("BH1750 init failed, check wiring.");
  } else {
    Serial.println("BH1750 ready.");
  }
  send_all_off(); //turn off all the light after open



  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  printMacAddress(mac);

  Serial.print("This device is Vespera ");
  Serial.println(lightId);

  // Connect to WiFi
  startWifi();

  // Connect to MQTT broker
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setBufferSize(2000);
  mqttClient.setCallback(callback);
  
  Serial.println("Set-up complete");
}
 
void loop() {
  // Reconnect if necessary
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  
  if (WiFi.status() != WL_CONNECTED){
    startWifi();
  }
  // keep mqtt alive
  mqttClient.loop();

  //change brightness with the lightness
  float lux = lightMeter.readLightLevel(); // if <0, fail
  if (lux >= 0) {
    // Exponential smoothing
    if (luxEMA < 0) luxEMA = lux;
    luxEMA = ALPHA * lux + (1.0f - ALPHA) * luxEMA;

    // brightness range (1~2000 lux → 255~20)
    float clamped = constrain(luxEMA, 1.0f, 1000.0f);
    uint8_t br = (uint8_t)constrain(map((int)clamped, 1, 2000, 255, 5), 0, 255);

    // only update when bright and time change enough
    if (abs((int)br - (int)lastBr) >= 4 && (millis() - lastSend) > 120) {
      uint16_t r = (uint16_t)BASE_R * br / 255;
      uint16_t g = (uint16_t)BASE_G * br / 255;
      uint16_t b = (uint16_t)BASE_B * br / 255;

      for (int p = 0; p < num_leds; p++) {
        RGBpayload[p*3 + 0] = (byte)r;
        RGBpayload[p*3 + 1] = (byte)g;
        RGBpayload[p*3 + 2] = (byte)b;
      }
      if (mqttClient.connected()) {
        mqttClient.publish(mqtt_topic.c_str(), RGBpayload, payload_size);
      }
      lastBr = br;
      lastSend = millis();

    }
  }
  delay(20);




}

// Function to update the R, G, B values of a single LED pixel
// RGB can a value between 0-254, pixel is 0-71 for a 72 neopixel strip
void send_RGB_to_pixel(int r, int g, int b, int pixel) {
  // Check if the mqttClient is connected before publishing
  if (mqttClient.connected()) {
    // Update the byte array with the specified RGB color pattern
    RGBpayload[pixel * 3 + 0] = (byte)r; // Red
    RGBpayload[pixel * 3 + 1] = (byte)g; // Green
    RGBpayload[pixel * 3 + 2] = (byte)b; // Blue

    // Publish the byte array
    mqttClient.publish(mqtt_topic.c_str(), RGBpayload, payload_size);
    
    Serial.println("Published whole byte array after updating a single pixel.");
  } else {
    Serial.println("MQTT mqttClient not connected, cannot publish from *send_RGB_to_pixel*.");
  }
}

void send_all_off() {
  // Check if the mqttClient is connected before publishing
  if (mqttClient.connected()) {
    // Fill the byte array with the specified RGB color pattern
    for(int pixel=0; pixel < num_leds; pixel++){
      RGBpayload[pixel * 3 + 0] = (byte)0; // Red
      RGBpayload[pixel * 3 + 1] = (byte)0; // Green
      RGBpayload[pixel * 3 + 2] = (byte)0; // Blue
    }
    // Publish the byte array
    mqttClient.publish(mqtt_topic.c_str(), RGBpayload, payload_size);
    
    Serial.println("Published an all zero (off) byte array.");
  } else {
    Serial.println("MQTT mqttClient not connected, cannot publish from *send_all_off*.");
  }
}

void send_all_random() {
  // Check if the mqttClient is connected before publishing
  if (mqttClient.connected()) {
    // Fill the byte array with the specified RGB color pattern
    for(int pixel=0; pixel < num_leds; pixel++){
      RGBpayload[pixel * 3 + 0] = (byte)random(50,256); // Red - 256 is exclusive, so it goes up to 255
      RGBpayload[pixel * 3 + 1] = (byte)random(50,256); // Green
      RGBpayload[pixel * 3 + 2] = (byte)random(50,256); // Blue
    }
    // Publish the byte array
    mqttClient.publish(mqtt_topic.c_str(), RGBpayload, payload_size);
    
    Serial.println("Published an all random byte array.");
  } else {
    Serial.println("MQTT mqttClient not connected, cannot publish from *send_all_random*.");
  }
}

void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}



