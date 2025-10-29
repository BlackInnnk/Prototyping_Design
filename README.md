# Vespera MQTT Light Controller

This Arduino project controls a Vespera LED panel through MQTT using the MKR WiFi 1010 board.  
A BH1750 light sensor measures ambient brightness, and the system changes lighting color or brightness automatically.  
A push button lets the user switch between different lighting modes.

---

## ⚙️ Hardware Setup

- **Arduino MKR WiFi 1010**
- **BH1750 Light Sensor (I2C)**
- **Push Button (Pin 7)**
- **Vespera LED Panel** – 72 pixels arranged in a 6×12 grid  

Wiring:
- BH1750 → SDA/SCL to MKR1010 I2C pins  
- Button → Pin 7 (digital input) + GND  
- Power via USB  

---

## 🌐 Network & Communication

The MKR1010 connects to Wi-Fi using **WiFiNINA** and publishes light data over **MQTT**  
to the broker `mqtt.cetools.org` on port `1884`.

All Wi-Fi and MQTT credentials are stored in a separate `arduino_secrets.h` file:
```cpp
#define SECRET_SSID "WiFi name"
#define SECRET_PASS "WiFi password"
#define SECRET_MQTTUSER "student"
#define SECRET_MQTTPASS "ce2021-mqtt-forget-whale"