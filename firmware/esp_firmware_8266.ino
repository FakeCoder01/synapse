/*
 * Synapse Robot ESP8266 Firmware
 * Complete motor control, sensor reading, and MQTT communication
 *
 * Hardware:
 * - ESP8266 (NodeMCU or similar)
 * - 4x TT Motors (differential drive)
 * - L298N Motor Driver
 * - 1x IR Distance Sensor (analog)
 *
 * Communication: MQTT over WiFi
 */

#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Ticker.h>

// ============================================================================
// CONFIGURATION - UPDATE THIS SECTION
// ============================================================================

// WiFi Configuration
const char *SSID = "YOUR_WIFI_SSID";
const char *PASSWORD = "YOUR_WIFI_PASSWORD";

// MQTT Configuration
const char *MQTT_SERVER = "192.168.1.100"; // IP of your ROS2 computer
const int MQTT_PORT = 1883;
const char *MQTT_CLIENT_ID = "robot_esp8266_01";

// Motor Configuration (GPIO Pins for L298N on NodeMCU)
// These are common pinouts, adjust to your wiring.
const uint8_t MOTOR_FL_IN1 = D1; // Front-Left
const uint8_t MOTOR_FL_IN2 = D2;
const uint8_t MOTOR_FR_IN1 = D5; // Front-Right
const uint8_t MOTOR_FR_IN2 = D6;

// L298N Enable Pins (must be PWM-capable)
const uint8_t MOTOR_LEFT_EN = D3;  // ENA for left side motors (FL, BL)
const uint8_t MOTOR_RIGHT_EN = D7; // ENB for right side motors (FR, BR)

// NOTE: This setup assumes front-left and back-left motors are wired in parallel
// to the same L298N output, and same for the right side.

// Sensor pins
const uint8_t IR_LEFT_PIN = A0; // Analog pin for left IR sensor

// Timing
const unsigned long SENSOR_PUBLISH_INTERVAL = 1000; // Publish sensors every 1s
const unsigned long TELEMETRY_INTERVAL = 2000;      // Send telemetry every 2s
const unsigned long MQTT_RECONNECT_INTERVAL = 5000; // Reconnect MQTT every 5s

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Motor speed state (-255 to 255)
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

// Sensor readings
uint16_t irLeftValue = 0;

// Timers
Ticker sensorPublishTimer;
Ticker telemetryTimer;

// ============================================================================
// MQTT TOPICS
// ============================================================================

// Subscribe to
const char *TOPIC_MOTOR_CMD = "robot/motor_cmd"; // Expects {"fl": pwm, "fr": pwm, "bl": pwm, "br": pwm}

// Publish to
const char *TOPIC_STATUS = "robot/status";
const char *TOPIC_SENSORS = "robot/sensors";
const char *TOPIC_TELEMETRY = "robot/telemetry";

// ============================================================================
// SETUP AND INITIALIZATION
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n[SETUP] AI Robot ESP8266 Firmware");

  // Initialize motor pins
  setupMotors();

  // Initialize sensor pins
  pinMode(IR_LEFT_PIN, INPUT);

  // Initialize WiFi
  setupWiFi();

  // Initialize MQTT
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  // Setup timers
  sensorPublishTimer.attach_ms(SENSOR_PUBLISH_INTERVAL, publishSensorData);
  telemetryTimer.attach_ms(TELEMETRY_INTERVAL, publishTelemetry);

  Serial.println("[SETUP] Initialization complete!");
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

void setupMotors() {
  pinMode(MOTOR_FL_IN1, OUTPUT);
  pinMode(MOTOR_FL_IN2, OUTPUT);
  pinMode(MOTOR_FR_IN1, OUTPUT);
  pinMode(MOTOR_FR_IN2, OUTPUT);
  pinMode(MOTOR_LEFT_EN, OUTPUT);
  pinMode(MOTOR_RIGHT_EN, OUTPUT);

  stopAllMotors();
  Serial.println("[MOTOR] Motor pins initialized");
}

/**
 * Set left-side motors speed
 * speed: -255 (full reverse) to 255 (full forward)
 */
void setLeftMotors(int speed) {
  speed = constrain(speed, -255, 255);
  leftMotorSpeed = speed;

  if (speed > 0) {
    digitalWrite(MOTOR_FL_IN1, HIGH);
    digitalWrite(MOTOR_FL_IN2, LOW);
  } else if (speed < 0) {
    digitalWrite(MOTOR_FL_IN1, LOW);
    digitalWrite(MOTOR_FL_IN2, HIGH);
    speed = -speed;
  } else {
    digitalWrite(MOTOR_FL_IN1, LOW);
    digitalWrite(MOTOR_FL_IN2, LOW);
  }
  analogWrite(MOTOR_LEFT_EN, speed);
}

/**
 * Set right-side motors speed
 * speed: -255 (full reverse) to 255 (full forward)
 */
void setRightMotors(int speed) {
  speed = constrain(speed, -255, 255);
  rightMotorSpeed = speed;

  if (speed > 0) {
    digitalWrite(MOTOR_FR_IN1, HIGH);
    digitalWrite(MOTOR_FR_IN2, LOW);
  } else if (speed < 0) {
    digitalWrite(MOTOR_FR_IN1, LOW);
    digitalWrite(MOTOR_FR_IN2, HIGH);
    speed = -speed;
  } else {
    digitalWrite(MOTOR_FR_IN1, LOW);
    digitalWrite(MOTOR_FR_IN2, LOW);
  }
  analogWrite(MOTOR_RIGHT_EN, speed);
}

void stopAllMotors() {
  setLeftMotors(0);
  setRightMotors(0);
  Serial.println("[MOTOR] All motors stopped");
}

// ============================================================================
// SENSOR READING
// ============================================================================

void publishSensorData() {
  irLeftValue = analogRead(IR_LEFT_PIN);

  // Convert ADC value to distance estimate (varies by sensor)
  // This is for a Sharp GP2Y0A21YK0F. Adjust if using a different sensor.
  float voltage_left = (irLeftValue / 1023.0) * 3.3;
  float distance_cm = (voltage_left > 0.3) ? 27.86 * pow(voltage_left, -1.15) : 80.0;

  DynamicJsonDocument doc(256);
  doc["ir_left_cm"] = distance_cm;

  String payload;
  serializeJson(doc, payload);
  mqttPublish(TOPIC_SENSORS, payload.c_str());
}

// ============================================================================
// MQTT COMMUNICATION
// ============================================================================

void setupWiFi() {
  Serial.printf("[WiFi] Connecting to %s\n", SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\n[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
}

void ensureMQTTConnected() {
  if (!mqttClient.connected()) {
    Serial.println("[MQTT] Attempting connection...");
    if (mqttClient.connect(MQTT_CLIENT_ID)) {
      Serial.println("[MQTT] Connected!");
      mqttClient.subscribe(TOPIC_MOTOR_CMD);
      mqttPublish(TOPIC_STATUS, "{\"status\":\"online\"}");
    } else {
      Serial.printf("[MQTT] Connection failed, rc=%d. Retrying in 5s.\n", mqttClient.state());
    }
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.printf("[MQTT] RX on %s: %s\n", topic, message.c_str());

  if (strcmp(topic, TOPIC_MOTOR_CMD) == 0) {
    DynamicJsonDocument doc(256);
    DeserializationError error = deserializeJson(doc, payload, length);

    if (error) {
      Serial.printf("[MQTT] JSON parse error: %s\n", error.c_str());
      return;
    }

    // The ROS node sends commands for all 4 wheels. We use the left and right values.
    int left_pwm = doc["fl"] | 0;
    int right_pwm = doc["fr"] | 0;

    setLeftMotors(left_pwm);
    setRightMotors(right_pwm);
  }
}

void mqttPublish(const char *topic, const char *payload) {
  if (mqttClient.connected()) {
    mqttClient.publish(topic, payload);
  }
}

// ============================================================================
// TELEMETRY
// ============================================================================

void publishTelemetry() {
  DynamicJsonDocument doc(512);
  doc["uptime_ms"] = millis();
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["motor_left_pwm"] = leftMotorSpeed;
  doc["motor_right_pwm"] = rightMotorSpeed;
  doc["ir_left_raw"] = irLeftValue;
  doc["free_heap"] = ESP.getFreeHeap();

  String payload;
  serializeJson(doc, payload);
  mqttPublish(TOPIC_TELEMETRY, payload.c_str());
}

// ============================================================================
//  MAIN LOOP
// ============================================================================

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    setupWiFi(); // Simple reconnect logic
  }
  
  if (!mqttClient.connected()) {
    ensureMQTTConnected();
  }
  
  mqttClient.loop(); // Process MQTT messages
  
  // Tickers handle periodic tasks in the background
  delay(10);
}
