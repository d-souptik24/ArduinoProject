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
const unsigned long ALERT_COOLDOWN = 300000; // 5 minutes

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
  // 1. ORIGINAL NTC LOGIC
  int thermistor_adc_val = analogRead(thermistor_output);
  double output_voltage = ( (thermistor_adc_val * 5.0) / 1023.0 );
  
  // Using specific 100k fixed resistor formula
  double thermistor_resistance = ( ( 5 * ( 100.0 / output_voltage ) ) - 100 ); /* K-Ohms */
  thermistor_resistance = thermistor_resistance * 1000; /* Ohms */
  
  double therm_res_ln = log(thermistor_resistance);
  
  /* Steinhart-Hart: 1 / (A + B[ln(R)] + C[ln(R)]^3) */
  double tempK = ( 1 / ( 0.001129148 + ( 0.000234125 * therm_res_ln ) + ( 0.0000000876741 * pow(therm_res_ln, 3) ) ) );
  float tempC = (float)tempK - 273.15;

  
  updateLEDStatus(tempC);
  checkTemperatureAlerts(tempC);

  if (digitalRead(button_pin) == LOW) {
    acknowledgeAlert();
    delay(200); // Simple debounce
  }

  // Debugging
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
