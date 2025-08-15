#include <OneWire.h>              // Include library for OneWire communication (required for DS18B20)
#include <DallasTemperature.h>     // Include library for Dallas Temperature ICs (DS18B20)

// Pin definitions
#define ONE_WIRE_BUS 2            // Data pin for DS18B20 sensor connected to Arduino digital pin 2
const int red_led = 3;            // Red LED connected to digital pin 3
const int yellow_led = 4;         // Yellow LED connected to digital pin 4
const int green_led = 5;          // Green LED connected to digital pin 5
const int buzzer = 6;             // Buzzer connected to digital pin 6
const int relay_pin = 7;          // Relay (for fan/heater/etc.) connected to digital pin 7
const int button_pin = 8;         // Button for alert acknowledgement connected to digital pin 8

// Temperature thresholds
float min_temp = 18.0;            // Minimum temperature threshold in Celsius
float max_temp = 25.0;            // Maximum temperature threshold in Celsius
float critical_temp = 30.0;       // Critical temperature threshold in Celsius

bool alert_active = false;        // Track if an alert is currently active
unsigned long last_alert_time = 0;// Store the last alert time (for cooldown)
const unsigned long ALERT_COOLDOWN = 300000; // Minimum time between alerts (5 minutes in milliseconds)

// Initialize OneWire instance for DS18B20
OneWire oneWire(ONE_WIRE_BUS);    // Create a OneWire object using the specified pin
DallasTemperature sensors(&oneWire); // Create DallasTemperature object, passing the OneWire reference

void setup() {
  Serial.begin(9600);             // Start serial communication at 9600 baud for debugging
  pinMode(red_led, OUTPUT);       // Set red LED pin as output
  pinMode(yellow_led, OUTPUT);    // Set yellow LED pin as output
  pinMode(green_led, OUTPUT);     // Set green LED pin as output
  pinMode(buzzer, OUTPUT);        // Set buzzer pin as output
  pinMode(relay_pin, OUTPUT);     // Set relay pin as output
  pinMode(button_pin, INPUT_PULLUP); // Set button pin as input with pull-up resistor

  sensors.begin();                // Start the DS18B20 sensor library
}

void loop() {
  sensors.requestTemperatures();  // Request temperature from all DS18B20 sensors on the bus
  float tempC = sensors.getTempCByIndex(0); // Read temperature from the first sensor (index 0)

  updateLEDStatus(tempC);         // Update LED indicators based on temperature
  checkTemperatureAlerts(tempC);  // Check if temperature is outside thresholds and manage alerts

  if (digitalRead(button_pin) == LOW) { // If the button is pressed (LOW because of pull-up)
    acknowledgeAlert();           // Call the function to acknowledge and clear alert
    delay(200);                   // Debounce delay to prevent multiple triggers
  }

  Serial.print("Temp: ");         // Print label to Serial Monitor
  Serial.print(tempC, 2);         // Print temperature value with 2 decimal places
  Serial.println(" C");           // Print unit and newline

  delay(1000);                    // Wait 1 second before next reading
}

void updateLEDStatus(float temp) {
  digitalWrite(red_led, LOW);     // Turn off red LED
  digitalWrite(yellow_led, LOW);  // Turn off yellow LED
  digitalWrite(green_led, LOW);   // Turn off green LED

  if (temp >= critical_temp)      // If temperature is at or above critical threshold
    digitalWrite(red_led, HIGH);  // Turn on red LED
  else if (temp >= max_temp || temp <= min_temp) // If temp is above max OR below min
    digitalWrite(yellow_led, HIGH); // Turn on yellow LED
  else
    digitalWrite(green_led, HIGH); // Otherwise, temperature is normal, turn on green LED
}

void checkTemperatureAlerts(float temp) {
  bool should_alert = false;      // Initialize flag for alert status

  // If temp is at/above critical, above max, or below min, set should_alert true
  if (temp >= critical_temp || temp >= max_temp || temp <= min_temp) {
    should_alert = true;
  }

  // If alert should be triggered and not active and cooldown elapsed
  if (should_alert && !alert_active && (millis() - last_alert_time) > ALERT_COOLDOWN) {
    triggerAlert();               // Trigger alert sequence
  } else if (!should_alert && alert_active) { // If temperature normal and alert is active
    clearAlert();                 // Clear the alert
  }
}

void triggerAlert() {
  alert_active = true;            // Set alert_active flag
  last_alert_time = millis();     // Record the time alert was triggered

  for (int i = 0; i < 3; i++) {  // Repeat buzzer pattern 3 times
    digitalWrite(buzzer, HIGH);   // Turn on buzzer
    delay(200);                   // Wait 200ms
    digitalWrite(buzzer, LOW);    // Turn off buzzer
    delay(200);                   // Wait 200ms
  }
  digitalWrite(relay_pin, HIGH);  // Turn on relay to activate external device (e.g., fan)
}

void acknowledgeAlert() {
  alert_active = false;           // Reset alert flag
  digitalWrite(buzzer, LOW);      // Ensure buzzer is off
}

void clearAlert() {
  alert_active = false;           // Reset alert flag
  digitalWrite(buzzer, LOW);      // Ensure buzzer is off
  digitalWrite(relay_pin, LOW);   // Turn off relay (deactivate external device)
}