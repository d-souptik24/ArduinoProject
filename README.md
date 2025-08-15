# Arduino Uno Temperature Monitor & Alert System

A compact, reliable temperature monitoring and alert system for the Arduino Uno using a DS18B20 digital temperature sensor, tri‑color status LEDs, a relay (fan / heater / exhaust), a buzzer, and an acknowledgment button. Designed to be easily extended (logging, displays, networking) while remaining well within the Uno’s resource limits.

---

## Table of Contents
1. Overview  
2. Features  
3. Hardware Requirements  
4. Wiring & Pin Map  
5. Temperature Logic  
6. Configuration  
7. Build & Run  
8. Provided Sketch  
9. Testing Procedure  
10. Improvement Opportunities  
11. Safety Notes  
12. License  

---

## 1. Overview
The system continuously samples temperature, displays status via LEDs, activates a relay and buzzer on threshold breaches, and enforces a cooldown to prevent alert spamming. A push button acknowledges (silences) an active alert. The default logic is suitable for enclosure, equipment bay, or ambient monitoring.

---

## 2. Features

| Feature | Description |
|---------|-------------|
| DS18B20 temperature sensing | Accurate digital Celsius readings (OneWire bus) |
| Status visualization | Green (normal), Yellow (warning: below min or above max), Red (critical) |
| Relay drive | Activates on alert (adjust policy as required) |
| Audible buzzer | Pattern on new alert (3 pulses) |
| Acknowledgment button | Silences buzzer; LEDs remain indicative |
| Alert cooldown | Prevents repetitive buzzer retriggers (default 5 min) |
| Serial diagnostics | Temperature + state logging @ 9600 baud |
| Clear separation of thresholds | Minimum, maximum, and critical |

---

## 3. Hardware Requirements (Minimum)

| Component | Notes |
|-----------|-------|
| Arduino Uno (ATmega328P) | Genuine or compatible |
| DS18B20 sensor | TO‑92 or waterproof probe; requires 4.7 kΩ pull‑up |
| 3 × LEDs + 3 × 220–330 Ω resistors | Green, Yellow, Red |
| Relay module (5V) | Prefer opto‑isolated module; HIGH = active (verify yours) |
| Active buzzer | Simple ON/OFF pattern |
| Momentary push button | With internal pull‑up |
| Jumper wires / breadboard | As needed |
| USB cable / 5V supply | Stable power source |

Optional expansions: I2C OLED, SD card module, Ethernet shield, ESP8266 bridge, enclosures, multiple DS18B20 sensors.

---

## 4. Wiring & Pin Map (Default)

| Function | Arduino Pin | Direction | Notes |
|----------|-------------|----------|-------|
| DS18B20 Data | D2 | OneWire | 4.7 kΩ resistor between D2 and +5V |
| Red LED (critical) | D3 | OUTPUT | Active HIGH |
| Yellow LED (warning) | D4 | OUTPUT | Active HIGH |
| Green LED (normal) | D5 | OUTPUT | Active HIGH |
| Buzzer | D6 | OUTPUT | Active HIGH |
| Relay | D7 | OUTPUT | Default LOW (inactive) |
| Acknowledge button | D8 | INPUT_PULLUP | Press = LOW |
| Serial (USB) | D0/D1 | UART | Do not repurpose |

---

## 5. Temperature Logic

| State | Condition | LED | Relay | Buzzer |
|-------|-----------|-----|-------|--------|
| Normal | min_temp < T < max_temp | Green | OFF | OFF |
| Warning (Low) | T ≤ min_temp | Yellow | OFF | Pulses on initial alert only |
| Warning (High) | max_temp ≤ T < critical_temp | Yellow | OFF | Pulses on initial alert only |
| Critical | T ≥ critical_temp | Red | ON | Pulses on initial alert only |
| Acknowledged | Button pressed during alert | (Depends on T) | Follows policy (e.g., ON if critical) | OFF |

Alert conditions: T ≤ min_temp OR T ≥ max_temp. Critical threshold provides distinct LED/relay behavior. A cooldown prevents the buzzer from repeating until the set interval elapses.

---

## 6. Configuration

Adjust these in the sketch:
- min_temp (default 18.0 °C)
- max_temp (default 25.0 °C)
- critical_temp (default 30.0 °C)
- ALERT_COOLDOWN (default 300000 ms = 5 min)

Relay behavior may be modified (e.g., only trigger on critical, or engage for both low and high conditions). For active‑LOW relay modules, invert logic in `triggerAlert()` and initialization.

---

## 7. Build & Run

1. Install Arduino IDE (or PlatformIO).
2. Install libraries via Library Manager:
   - OneWire
   - DallasTemperature
3. Wire hardware per the pin map.
4. Open the sketch, verify board = “Arduino Uno”.
5. Upload.
6. Open Serial Monitor @ 9600 baud.
7. Observe temperature and state transitions.

---

## 8. Provided Sketch

```cpp
#include <OneWire.h>              // Include library for OneWire communication (required for DS18B20)
#include <DallasTemperature.h>    // Include library for Dallas Temperature ICs (DS18B20)

// Pin definitions
#define ONE_WIRE_BUS 2            // Data pin for DS18B20 sensor connected to Arduino digital pin 2
const int red_led    = 3;         // Red LED connected to digital pin 3
const int yellow_led = 4;         // Yellow LED connected to digital pin 4
const int green_led  = 5;         // Green LED connected to digital pin 5
const int buzzer     = 6;         // Buzzer connected to digital pin 6
const int relay_pin  = 7;         // Relay (fan/heater/etc.) connected to digital pin 7
const int button_pin = 8;         // Button for alert acknowledgement connected to digital pin 8

// Temperature thresholds (°C)
float min_temp      = 18.0;
float max_temp      = 25.0;
float critical_temp = 30.0;

bool alert_active = false;
unsigned long last_alert_time = 0;
const unsigned long ALERT_COOLDOWN = 300000; // 5 minutes

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup() {
  Serial.begin(9600);
  pinMode(red_led, OUTPUT);
  pinMode(yellow_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(relay_pin, OUTPUT);
  pinMode(button_pin, INPUT_PULLUP);

  digitalWrite(relay_pin, LOW);   // Ensure relay off at boot
  sensors.begin();
}

void loop() {
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  updateLEDStatus(tempC);
  checkTemperatureAlerts(tempC);

  if (digitalRead(button_pin) == LOW) {
    acknowledgeAlert();
    delay(200); // Basic debounce
  }

  Serial.print("Temp: ");
  Serial.print(tempC, 2);
  Serial.println(" C");

  delay(1000);
}

void updateLEDStatus(float temp) {
  digitalWrite(red_led, LOW);
  digitalWrite(yellow_led, LOW);
  digitalWrite(green_led, LOW);

  if (temp >= critical_temp) {
    digitalWrite(red_led, HIGH);
  } else if (temp >= max_temp || temp <= min_temp) {
    digitalWrite(yellow_led, HIGH);
  } else {
    digitalWrite(green_led, HIGH);
  }
}

void checkTemperatureAlerts(float temp) {
  bool should_alert =
    (temp >= critical_temp) ||
    (temp >= max_temp) ||
    (temp <= min_temp);

  if (should_alert && !alert_active && (millis() - last_alert_time) > ALERT_COOLDOWN) {
    triggerAlert();
  } else if (!should_alert && alert_active) {
    clearAlert();
  }
}

void triggerAlert() {
  alert_active = true;
  last_alert_time = millis();

  for (int i = 0; i < 3; i++) {
    digitalWrite(buzzer, HIGH);
    delay(200);
    digitalWrite(buzzer, LOW);
    delay(200);
  }

  digitalWrite(relay_pin, HIGH); // Activate external cooling/heating device
}

void acknowledgeAlert() {
  alert_active = false;
  digitalWrite(buzzer, LOW);
}

void clearAlert() {
  alert_active = false;
  digitalWrite(buzzer, LOW);
  digitalWrite(relay_pin, LOW);
}
```

---

## 9. Testing Procedure

| Step | Action | Expected Result |
|------|--------|-----------------|
| 1 | Power system & open Serial Monitor | Temperature prints every second |
| 2 | Idle at nominal temperature | Green LED ON, relay OFF, no buzzer |
| 3 | Warm sensor above max_temp (e.g., touch) | Yellow LED; after cooldown window if newly triggered → buzzer pulses, relay engages |
| 4 | Continue heating to ≥ critical_temp | Red LED replaces Yellow; relay remains ON |
| 5 | Press button during alert | Buzzer silenced (no retrigger until conditions + cooldown) |
| 6 | Allow sensor to cool into normal band | Alert clears, relay OFF, Green LED resumes |
| 7 | Re‑trigger before cooldown ends | No buzzer until cooldown elapsed |

---

## 10. Improvement Opportunities

| Category | Enhancement | Benefit |
|----------|-------------|---------|
| Timing | Replace blocking delays with millis() scheduling | Enables multitasking (logging, UI) |
| Robustness | Detect sensor disconnect (`DEVICE_DISCONNECTED_C`) | Fail-safe behavior |
| Persistence | Store thresholds in EEPROM | Retain settings across resets |
| Precision | Add moving average / hysteresis | Reduces false oscillations |
| Hardware | Add fuse, enclosure, strain relief | Deployment readiness |
| UI | Add OLED / LCD readout | Local visibility |
| Data | SD card / CSV logging | Historical analysis |
| Networking | Offload to ESP8266/ESP32 (HTTP/MQTT) | Remote monitoring |
| Multi-sensor | Enumerate all DS18B20 devices | Multi-point thermal profiling |
| Reliability | Enable watchdog timer | Automatic recovery |

---

## 11. Safety Notes

- When switching mains voltage, use properly rated relay modules, enclosures, and observe creepage/clearance.  
- Never power high-voltage loads on an open breadboard.  
- For inductive loads (motors, solenoids), ensure the relay module includes proper flyback protection (diode or snubber).  
- Maintain solid ground reference and avoid long, noise-prone sensor wiring without shielding.

---

## 12. License (MIT)

```
MIT License

Copyright (c) 2025 d-souptik24

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
```

---

For production use, consider implementing non‑blocking timing, watchdog protection, and EEPROM-backed configuration as early enhancements.
