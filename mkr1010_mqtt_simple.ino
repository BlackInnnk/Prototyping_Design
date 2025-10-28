// Duncan Wilson Oct 2025 - v1 - MQTT messager to vespera

// works with MKR1010

#include <SPI.h>
#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <utility/wifi_drv.h>   // library to drive to RGB LED on the MKR1010
#include <Wire.h>
#include <BH1750.h>
#include <math.h>
#include "arduino_secrets.h" 
BH1750 lightMeter(BH1750::CONTINUOUS_LOW_RES_MODE);

/*
**** please enter your sensitive data in the Secret tab/arduino_secrets.h
**** using format below
#define SECRET_SSID "ssid name"
#define SECRET_PASS "ssid password"
#define SECRET_MQTTUSER "user name - eg student"
#define SECRET_MQTTPASS "password";
 */
const char* ssid          = SECRET_SSID;
const char* password      = SECRET_PASS;
const char* mqtt_username = SECRET_MQTTUSER;
const char* mqtt_password = SECRET_MQTTPASS;
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

// Grid layout
const int ROWS = 6;
const int COLS = 12;


// Create the byte array to send in MQTT payload this stores all the colours 
// in memory so that they can be accessed in for example the rainbow function
byte RGBpayload[payload_size];

// Convert HSV to 8-bit RGB
void hsvToRgb(float h, float s, float v, uint8_t& r, uint8_t& g, uint8_t& b){
  while (h < 0)        h += 360.0f;
  while (h >= 360.0f)  h -= 360.0f;
  if (s <= 0.0f){ // gray
    uint8_t x = (uint8_t)(v * 255.0f);
    r = g = b = x;
    return;
  }
  float c = v * s;
  float x = c * (1 - fabsf(fmodf(h/60.0f, 2.0f) - 1));
  float m = v - c;
  float rp, gp, bp;
  if      (h < 60)  { rp=c; gp=x; bp=0; }
  else if (h < 120) { rp=x; gp=c; bp=0; }
  else if (h < 180) { rp=0; gp=c; bp=x; }
  else if (h < 240) { rp=0; gp=x; bp=c; }
  else if (h < 300) { rp=x; gp=0; bp=c; }
  else              { rp=c; gp=0; bp=x; }
  r = (uint8_t)((rp + m) * 255.0f);
  g = (uint8_t)((gp + m) * 255.0f);
  b = (uint8_t)((bp + m) * 255.0f);
}

float luxEMA = -1.0;                 // Exponential Moving Average
const uint8_t BASE_R = 255, BASE_G = 180, BASE_B = 60; // warm yellow light

// Button!!!
const int PIN_BTN = 7;  // pin7
enum Mode {
  MODE_A_LUX_RED,   // red light
  MODE_B_R_SWEEP,   // red sweep
  MODE_C_G_SWEEP,   // green sweep
  MODE_D_B_SWEEP,   // blue sweep
  MODE_E_LAYER,     // layer brightness
  MODE_G_LOOP,      // RGB color loop
  MODE_F_OFF        // turn off
};
Mode modeNow = MODE_A_LUX_RED;

bool btnLast = true;  // pressed = LOW
unsigned long btnTs = 0;
const unsigned long DEBOUNCE = 50; //ms

// Gradient animation (G)
const uint32_t HUE_CYCLE_MS = 10000;   // time for 1 coulor loop



void setup() {
  Serial.begin(115200);
  //while (!Serial); // Wait for serial port to connect (useful for debugging)
  Serial.println("Vespera");
  // start light sensor and wifi setup

  pinMode(PIN_BTN, INPUT_PULLUP); //Initialize button

  Wire.begin();
  if (!lightMeter.begin()) {
    Serial.println("BH1750 init failed, check wiring.");
  } else {
    lightMeter.configure(BH1750::CONTINUOUS_LOW_RES_MODE); // use faster light sensor mode
    Serial.println("BH1750 ready (LOW_RES).");
  }
  send_all_off(); //turn off all the light when start
  // make sure lights are clear before running



  

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
  // keep checking wifi and mqtt connection
  // if not connected then try to reconnect
  // Reconnect if necessary
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  
  if (WiFi.status() != WL_CONNECTED){
    startWifi();
  }
  // keep mqtt alive
  mqttClient.loop();

  //Press button to go through all modes
  bool now = (digitalRead(PIN_BTN) == LOW);        // pressed = LOW

  // change mode each time button is pressed
  // print the current mode to serial monitor
  if (now != btnLast && millis() - btnTs > DEBOUNCE) {
    btnTs = millis();
    if (now == false && btnLast == true) {         // detect button release
      modeNow = (Mode)((modeNow + 1) % 7);         // seven modes
      Serial.print("Mode -> ");
      switch (modeNow) {
        case MODE_A_LUX_RED: Serial.println("A (Lux Red)"); break;
        case MODE_B_R_SWEEP: Serial.println("B (Red Sweep)"); break;
        case MODE_C_G_SWEEP: Serial.println("C (Green Sweep)"); break;
        case MODE_D_B_SWEEP: Serial.println("D (Blue Sweep)"); break;
        case MODE_E_LAYER:   Serial.println("E (Layer)"); break;
        case MODE_G_LOOP:    Serial.println("G (RGB Loop)"); break;
        case MODE_F_OFF:     Serial.println("F (Off)"); break;
      }
    }
  }
  btnLast = now;


  //change brightness with the lightness
  static unsigned long lastFrame = 0;
  const unsigned long FRAME_MS = 80;  // send every 80ms

  if (millis() - lastFrame >= FRAME_MS) {
    lastFrame = millis();

    //EMA smoothing
    // read light value from BH1750 sensor
    // higher lux -> brighter environment
    float lux = lightMeter.readLightLevel();       // if <0, fail
    if (lux >= 0) {
      if (luxEMA < 0) luxEMA = lux;
      float delta = fabsf(lux - luxEMA);
      float a = (delta > 60.0f) ? 0.95f : 0.80f;   // react faster when big change
      luxEMA = a*lux + (1.0f - a)*luxEMA;
    }

    //mapping darker -> brighter 
    float Lmin=1.0f, Lmax=800.0f;
    float x = 1.0f - (constrain(luxEMA, Lmin, Lmax) - Lmin) / (Lmax - Lmin);
    float gamma = 0.7f;                         
    uint8_t br = (uint8_t)constrain((int)(255.0f * powf(x, gamma)), 0, 255);

    // Base color (red)
    uint8_t r = (uint16_t)BASE_R * br / 255;
    uint8_t g = (uint16_t)BASE_G * br / 255;
    uint8_t b = (uint16_t)BASE_B * br / 255;


    // Mode!!!
    if (modeNow == MODE_F_OFF) {
      // F: all off
      for (int p=0; p<num_leds; ++p) {
        RGBpayload[p*3+0] = 0;
        RGBpayload[p*3+1] = 0;
        RGBpayload[p*3+2] = 0;
      }
    }
    else if (modeNow == MODE_A_LUX_RED) {
      // A: warm yellow controlled by lightness
      for (int p=0; p<num_leds; ++p) {
        RGBpayload[p*3+0] = r;
        RGBpayload[p*3+1] = g;
        RGBpayload[p*3+2] = b;
      }
    }

    else if (modeNow == MODE_B_R_SWEEP) {
      // B: red gets brighter when it's darker, G and B fixed to 0
      // Darker environment → stronger red light.

      // Map luxEMA to R=0..255
      float Lmin = 1.0f, Lmax = 800.0f;  // adjust if sensor range differs
      float x = 1.0f - (constrain(luxEMA, Lmin, Lmax) - Lmin) / (Lmax - Lmin);
      float gammaR = 0.7f;                // nonlinear response
      uint8_t Rv = (uint8_t)constrain((int)(255.0f * powf(x, gammaR)), 0, 255);

      // Fill the strip with this red intensity
      for (int p = 0; p < num_leds; ++p) {
        RGBpayload[p*3 + 0] = Rv;   // Red follows ambient
        RGBpayload[p*3 + 1] = 0;    // Green off
        RGBpayload[p*3 + 2] = 0;    // Blue off
      }
    }
    else if (modeNow == MODE_C_G_SWEEP) {
      // C: Green controlled by lightness，R/B=0
      float Lmin=1.0f, Lmax=800.0f;
      float x = 1.0f - (constrain(luxEMA, Lmin, Lmax) - Lmin) / (Lmax - Lmin);
      uint8_t Gv = (uint8_t)constrain((int)(255.0f * powf(x, 0.7f)), 0, 255);
      for (int p=0; p<num_leds; ++p) {
        RGBpayload[p*3+0] = 0;
        RGBpayload[p*3+1] = Gv;
        RGBpayload[p*3+2] = 0;
      }
    }
    else if (modeNow == MODE_D_B_SWEEP) {
      // D: Blue controlled by lightness，R/G=0
      float Lmin=1.0f, Lmax=800.0f;
      float x = 1.0f - (constrain(luxEMA, Lmin, Lmax) - Lmin) / (Lmax - Lmin);
      uint8_t Bv = (uint8_t)constrain((int)(255.0f * powf(x, 0.7f)), 0, 255);
      for (int p=0; p<num_leds; ++p) {
        RGBpayload[p*3+0] = 0;
        RGBpayload[p*3+1] = 0;
        RGBpayload[p*3+2] = Bv;
      }
    }
    else if (modeNow == MODE_E_LAYER) {
      // E：Light up rows one by one
      float Lmin = 1.0f, Lmax = 800.0f;
      float x = 1.0f - (constrain(luxEMA, Lmin, Lmax) - Lmin) / (Lmax - Lmin);
      int lit = (int)constrain((int)roundf(x * num_leds), 0, num_leds); // 0..72

      const uint8_t WR = 255, WG = 180, WB = 60; // warm yellow light

      int k = 0; // numbers of already lighted
      for (int r = 0; r < ROWS; ++r) {
        for (int c = 0; c < COLS; ++c) {
          int idx = r + ROWS * c;         
          bool on = (k < lit);
          RGBpayload[idx*3 + 0] = on ? WR : 0;
          RGBpayload[idx*3 + 1] = on ? WG : 0;
          RGBpayload[idx*3 + 2] = on ? WB : 0;
          ++k;
        }
      }
    }
    else { // MODE_G_LOOP
      // colour loop
      float hue = fmodf((millis() % HUE_CYCLE_MS) * (360.0f / HUE_CYCLE_MS), 360.0f);

      // Convert HSV to RGB
      uint8_t R, G, B;
      hsvToRgb(hue, 1.0f, 1.0f, R, G, B);

      // Scale by environment brightness
      uint8_t rU = (uint8_t)((uint16_t)R * br / 255);
      uint8_t gU = (uint8_t)((uint16_t)G * br / 255);
      uint8_t bU = (uint8_t)((uint16_t)B * br / 255);

      // Fill  color
      for (int p = 0; p < num_leds; ++p) {
        RGBpayload[p*3 + 0] = rU;
        RGBpayload[p*3 + 1] = gU;
        RGBpayload[p*3 + 2] = bU;
      }
    }


    // Publish 
    if (mqttClient.connected()) { // send color data to vespera web display through MQTT
      mqttClient.publish(mqtt_topic.c_str(), RGBpayload, payload_size, true);
    }
  }

  delay(10);  



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

// print MAC address to serial monitor for debugging
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



