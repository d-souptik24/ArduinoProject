# Arduino Uno Smart NTC Temperature Monitor

A physics-based temperature monitoring system for the Arduino Uno. Instead of relying on digital libraries, this project implements the **Steinhart-Hart equation** to mathematically calculate temperature from an NTC thermistor. It features status LEDs, a relay control for automation (fan/heater), a buzzer alarm, and an acknowledgment system.

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
11. License

---

## 1. Overview
This system uses an analog voltage divider circuit to read the resistance of an NTC thermistor. It applies the **Steinhart-Hart logarithmic equation** to convert that resistance into a precise temperature reading in Celsius. It continuously monitors the environment and activates visual (LED) and audible (Buzzer) alerts if thresholds are breached, triggering a relay for external cooling or heating.

---

## 2. Features

| Feature | Description |
|---------|-------------|
| **Physics-Based Sensing** | Uses NTC Thermistor + Steinhart-Hart math (No heavy libraries) |
| **Status Visualization** | Green (Normal), Yellow (Warning), Red (Critical) |
| **Automated Control** | Relay activates on alert (Fan/Heater control) |
| **Audible Alarm** | 3-pulse buzzer pattern on new alerts |
| **Smart Acknowledgment** | Button press silences buzzer; system stays "Active" until safe |
| **Alert Cooldown** | Prevents buzzer "spamming" (Default: 5 minutes) |
| **Serial Diagnostics** | Real-time logging of Temperature (°C) and Resistance (Ω) |

---

## 3. Hardware Requirements

| Component | Specifications |
|-----------|----------------|
| Arduino Uno (ATmega328P) | Genuine or compatible |
| **NTC Thermistor** | 10kΩ or similar (Standard bead type) |
| **Fixed Resistor** | **100kΩ** (Critical for the voltage divider math) |
| 3 × LEDs | Red, Yellow, Green |
| 3 × Resistors | 220Ω - 330Ω (For LEDs) |
| Relay Module (5V) | Active HIGH configuration (Standard) |
| Active Buzzer | 5V DC |
| Push Button | Momentary switch |
| Jumper Wires | M-M, M-F as needed |

---

## 4. Wiring & Pin Map

**Crucial Circuit Note:** The thermistor requires a voltage divider circuit.
* **5V** -> Thermistor -> **A1** (Signal)
* **A1** -> 100kΩ Resistor -> **GND**

| Function | Arduino Pin | Type | Connection Notes |
|----------|-------------|------|------------------|
| **Thermistor Signal** | **A1** | Analog Input | Junction of Thermistor & 100k Resistor |
| Red LED (Critical) | D3 | OUTPUT | Active HIGH |
| Yellow LED (Warning) | D4 | OUTPUT | Active HIGH |
| Green LED (Normal) | D5 | OUTPUT | Active HIGH |
| Buzzer | D6 | OUTPUT | Active HIGH |
| Relay Control | D7 | OUTPUT | Active HIGH (Triggers fan/heater) |
| Acknowledge Button | D8 | INPUT_PULLUP | Connect other leg to GND |

---

## 5. Temperature Logic

| State | Condition | LED | Relay | Buzzer |
|-------|-----------|-----|-------|--------|
| **Normal** | 18°C < T < 25°C | Green | OFF | OFF |
| **Warning** | T ≤ 18°C OR T ≥ 25°C | Yellow | OFF | Pulses (Once) |
| **Critical** | T ≥ 30°C | Red | ON | Pulses (Once) |
| **Acknowledged** | User presses button | (Maintains Color) | ON (If Critical) | OFF (Silenced) |

*Note: The buzzer only sounds when a **new** alert event occurs or the cooldown timer expires.*

---

## 6. Configuration

You can adjust these values in the header of the sketch:

* **`min_temp`**: Lower bound of comfort zone (Default: 18.0°C)
* **`max_temp`**: Upper bound of comfort zone (Default: 25.0°C)
* **`critical_temp`**: Danger level, triggers relay (Default: 30.0°C)
* **`ALERT_COOLDOWN`**: Time in ms before buzzer can sound again (Default: 300,000ms = 5 mins)

---

## 7. Build & Run

1.  **Construct the Circuit:** Pay close attention to the Voltage Divider on Pin A1.
2.  **Open Arduino IDE:** Create a new sketch.
3.  **Paste Code:** Use the provided sketch below.
4.  **No Libraries Needed:** This code uses standard C math (`math.h`), so no external library installation is required.
5.  **Upload:** Connect Uno and upload.
6.  **Monitor:** Open Serial Monitor (9600 baud) to view real-time Resistance and Temperature data.

---

## 8. Provided Sketch

```cpp
#include <math.h>

// --- Hardware Pins ---
const int thermistor_output = A1;
const int red_led = 3;
const int yellow_led = 4;
const int green_led = 5;
const int buzzer = 6;
const int relay_pin = 7;
const int button_pin = 8;

// --- Temperature Thresholds ---
float min_temp = 18.0;
float max_temp = 25.0;
float critical_temp = 30.0;

// --- Alert Variables ---
bool alert_active = false;
unsigned long last_alert_time = 0;
const unsigned long ALERT_COOLDOWN = 300000; 

void setup() {
  Serial.begin(9600);
  
  pinMode(red_led, OUTPUT);
  pinMode(yellow_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(relay_pin, OUTPUT);
  pinMode(button_pin, INPUT_PULLUP);
}

void loop() {
  // Read analog data and convert to voltage
  int thermistor_adc_val = analogRead(thermistor_output);
  double output_voltage = ( (thermistor_adc_val * 5.0) / 1023.0 );
  
  // Calculate resistance based on 100k fixed resistor circuit
  double thermistor_resistance = ( ( 5 * ( 100.0 / output_voltage ) ) - 100 ); 
  thermistor_resistance = thermistor_resistance * 1000; 
  
  // Apply Steinhart-Hart equation to calculate temperature in Celsius
  double therm_res_ln = log(thermistor_resistance);
  double tempK = ( 1 / ( 0.001129148 + ( 0.000234125 * therm_res_ln ) + ( 0.0000000876741 * pow(therm_res_ln, 3) ) ) );
  float tempC = (float)tempK - 273.15;

  // Update hardware indicators and check for alert conditions
  updateLEDStatus(tempC);
  checkTemperatureAlerts(tempC);

  // Check for manual button press to mute the buzzer
  if (digitalRead(button_pin) == LOW) {
    acknowledgeAlert();
    delay(200); 
  }

  // Print data for serial monitoring
  Serial.print("Temp: ");
  Serial.print(tempC, 2);
  Serial.print(" C\tResistance: ");
  Serial.println(thermistor_resistance);

  delay(1000);
}

// --- Helper Functions ---

// Updates LED colors based on temperature thresholds
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

// Monitors temperature vs thresholds and handles the alert state machine
void checkTemperatureAlerts(float temp) {
  bool should_alert = (temp >= critical_temp || temp >= max_temp || temp <= min_temp);

  if (should_alert && !alert_active && (millis() - last_alert_time) > ALERT_COOLDOWN) {
    triggerAlert();
  } else if (!should_alert && alert_active) {
    clearAlert();
  }
}

// Triggers the audible alarm pattern and activates the relay
void triggerAlert() {
  alert_active = true;
  last_alert_time = millis();

  for (int i = 0; i < 3; i++) {
    digitalWrite(buzzer, HIGH);
    delay(200);
    digitalWrite(buzzer, LOW);
    delay(200);
  }
  digitalWrite(relay_pin, HIGH);
}

// Deactivates buzzer manually while keeping alert_active logic
void acknowledgeAlert() {
  alert_active = false;
  digitalWrite(buzzer, LOW);
}

// Resets alert status and turns off all warning hardware
void clearAlert() {
  alert_active = false;
  digitalWrite(buzzer, LOW);
  digitalWrite(relay_pin, LOW);
}
