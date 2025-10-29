# Vespera MQTT Light Controller

This Arduino project controls a Vespera LED panel through MQTT using the MKR WiFi 1010 board.  
A BH1750 light sensor measures ambient brightness, and the system changes lighting color or brightness automatically.  
A push button lets the user switch between different lighting modes.

---

## 💡 Hardware Setup

- **Arduino MKR WiFi 1010**  
- **BH1750 Light Sensor (I2C)**  
- **Push Button (Pin 7)**  
- **Vespera LED Panel** – 72 pixels arranged in a 6×12 grid  

**Wiring:**  
- BH1750 → SDA/SCL to MKR1010 I2C pins  
- Button → Pin 7 (digital input) + GND  
- Power via USB  

---

## 🌐 Network & Communication

The MKR1010 connects to Wi-Fi using **WiFiNINA** and publishes light data over **MQTT**  
to the broker `mqtt.cetools.org` on port **1884**.

All Wi-Fi and MQTT credentials are stored in a separate `arduino_secrets.h` file:

```cpp
#define SECRET_SSID "WiFi name"
#define SECRET_PASS "WiFi password"
#define SECRET_MQTTUSER "student"
#define SECRET_MQTTPASS "ce2021-mqtt-forget-whale"
```

---

## 🎨 Lighting Modes

The system provides multiple modes that can be cycled by pressing the button:

| Mode | Name | Description |
|------|------|--------------|
| A | **Warm Yellow Light** | Adjusts brightness based on ambient light (BH1750). |
| B | **Red Sweep** | Red intensity increases in darker environments. |
| C | **Green Sweep** | Green intensity varies with ambient brightness. |
| D | **Blue Sweep** | Blue intensity varies with ambient brightness. |
| E | **Layer Mode** | Lights up pixels sequentially based on brightness level. |
| G | **RGB Loop** | Smoothly cycles through the color spectrum. |
| F | **Off** | Turns off all pixels. |

Each press of the button switches to the next mode in sequence.

---

## ⚙️ System Logic

1. **Ambient Light Detection** – The BH1750 continuously reads lux values.  
2. **Exponential Moving Average (EMA)** – Applied to smooth sensor readings and prevent flicker.  
3. **Brightness Mapping** – Converts sensor readings into LED brightness (0–255).  
4. **Button Control** – Cycles through modes using a simple debounce mechanism.  
5. **MQTT Transmission** – Sends LED color payloads (`RGBpayload`) to the MQTT topic `student/CASA0014/luminaire/7`.  

---

## 🧩 Libraries Used

| Library | Function |
|----------|-----------|
| WiFiNINA | Provides Wi-Fi connectivity for MKR1010 |
| PubSubClient | Handles MQTT publish/subscribe |
| BH1750 | Communicates with the light sensor via I2C |
| Wire | Standard I2C communication library |
| Math.h | Used for smoothing, mapping, and gamma correction |

---

## 💻 Main Functions

| Function | Description |
|-----------|-------------|
| `setup()` | Initializes serial monitor, Wi-Fi, MQTT, and the BH1750 sensor. |
| `loop()` | Reads light levels, updates brightness, and handles button events. |
| `send_all_off()` | Publishes a payload of zeros to turn off all LEDs. |
| `send_RGB_to_pixel()` | Updates and publishes RGB data for a single pixel. |
| `hsvToRgb()` | Converts HSV color space to RGB values for smooth color transitions. |

---

## 🌈 Brightness & Color Mapping

- Ambient light values (`lux`) are smoothed using EMA:  
  ```
  luxEMA = α * lux + (1 - α) * luxEMA
  ```
- Then mapped to brightness using a nonlinear gamma function for perceptual balance.  
- Lower light → higher brightness (auto-dimming effect).  

---

## ▶️ How to Run

1. Open the `.ino` file in Arduino IDE.  
2. Install dependencies:  
   - **WiFiNINA**  
   - **PubSubClient**  
   - **BH1750**  
3. Add your Wi-Fi and MQTT credentials to `arduino_secrets.h`.  
4. Upload the sketch to MKR WiFi 1010.  
5. Open **Serial Monitor (115200 baud)** to view mode changes and status logs.  
6. Observe the lighting updates on the **Vespera LED Grid Visualizer** at  
   👉 [iot.io/projects/lumi](https://iot.io/projects/lumi)

---

## 🧠 Example Serial Output

```
Vespera
BH1750 ready (LOW_RES)
MAC address: 6C:4F:89:4A:33:40
This device is Vespera 7
Mode -> A (Lux Red)
Lux = 140.2 | Brightness = 120
MQTT Published: 72x RGB bytes
```

---

## 👤 Author

**Yewei Bian**  
MSc Connected Environments, UCL East  
Project: *Vespera MQTT Smart Luminaire*  
Supervisor: *Duncan Wilson*  
October 2025  

---
