/*
 * Synapse Robot ESP8266 Firmware
 * Complete motor control, sensor reading, and MQTT communication
 *
 * Hardware:
 * - ESP8266 (NodeMCU or similar)
 * - 4x TT Motors (differential drive)
 * - L298N Motor Driver
 * - 2x IR Distance Sensors (analog)
 *
 * Communication: MQTT over WiFi
 */

#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Ticker.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

// WiFi Configuration
const char *SSID = "WiFi SSID";
const char *PASSWORD = "WiFi PASSWORD";
const char *MQTT_SERVER = "192.168.1.100"; // Raspberry Pi/ROS2 System IP
const int MQTT_PORT = 1883;
const char *MQTT_CLIENT_ID = "robot_esp8266_01";

// Motor Configuration (GPIO Pins)
struct MotorPin {
  uint8_t in1;
  uint8_t in2;
  uint8_t en;
};

// Motor pins for L298N (can be adjusted as per wiring)
const MotorPin MOTOR_FL = {D1, D2, D3};   // Front-Left (FL) = 0
const MotorPin MOTOR_FR = {D4, D5, D6};   // Front-Right (FR) = 1
const MotorPin MOTOR_BL = {D7, D8, D0};   // Back-Left (BL) = 2
const MotorPin MOTOR_BR = {D9, D10, D11}; // Back-Right (BR) = 3

// Sensor pins
const uint8_t IR_LEFT = A0;     // Analog pin for left IR sensor
const uint8_t IR_RIGHT = A1;    // Analog pin for right IR sensor (if available)
const uint8_t SPEAKER_PIN = D4; // PWM pin for speaker

// Timing
const unsigned long SENSOR_PUBLISH_INTERVAL = 1000; // Publish sensors every 1s
const unsigned long WIFI_CHECK_INTERVAL = 5000;     // Check WiFi every 5s
const unsigned long MQTT_RECONNECT_INTERVAL = 5000; // Reconnect MQTT every 5s
const unsigned long TELEMETRY_INTERVAL = 2000;      // Send telemetry every 2s

// Motor PWM frequency and range
const uint16_t PWM_FREQUENCY = 1000; // 1kHz PWM
const uint16_t PWM_RANGE = 255;      // 0-255 PWM values
const float MAX_SPEED = 1.0;         // Max speed (normalized)

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Motor speed state (-255 to 255)
int motorSpeeds[4] = {0, 0, 0, 0}; // FL, FR, BL, BR

// Sensor readings
uint16_t irLeftValue = 0;
uint16_t irRightValue = 0;
float batteryVoltage = 0.0;

// Timers
Ticker sensorPublishTimer;
Ticker wifiCheckTimer;
Ticker mqttReconnectTimer;
Ticker telemetryTimer;

// Connection flags
volatile bool wifiConnected = false;
volatile bool mqttConnected = false;

// ============================================================================
// MQTT TOPICS
// ============================================================================

// Please check ROS2 and configure

// Subscribe to
const char *TOPIC_CMD_VEL = "robot/cmd_vel";
const char *TOPIC_MOTOR_CMD = "robot/motor_cmd";
const char *TOPIC_AUDIO_CMD = "robot/audio";
const char *TOPIC_CONFIG = "robot/config";

// Publish to
const char *TOPIC_STATUS = "robot/status";
const char *TOPIC_SENSORS = "robot/sensors";
const char *TOPIC_ODOMETRY = "robot/odometry";
const char *TOPIC_TELEMETRY = "robot/telemetry";

// ============================================================================
// SETUP AND INITIALIZATION
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n\n");
  Serial.println("========================================");
  Serial.println("AI Robot ESP8266 Firmware v1.0");
  Serial.println("========================================");

  // Initialize motor pins
  setupMotors();

  // Initialize sensor pins
  setupSensors();

  // Initialize PWM
  analogWriteFreq(PWM_FREQUENCY);
  analogWriteRange(PWM_RANGE);

  // Initialize WiFi
  Serial.println("[SETUP] Initializing WiFi...");
  setupWiFi();

  // Initialize MQTT
  Serial.println("[SETUP] Initializing MQTT...");
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(1024);

  // Setup timers
  Serial.println("[SETUP] Setting up timers...");
  sensorPublishTimer.attach_ms(SENSOR_PUBLISH_INTERVAL, publishSensorData);
  wifiCheckTimer.attach_ms(WIFI_CHECK_INTERVAL, checkWiFiConnection);
  mqttReconnectTimer.attach_ms(MQTT_RECONNECT_INTERVAL, ensureMQTTConnected);
  telemetryTimer.attach_ms(TELEMETRY_INTERVAL, publishTelemetry);

  Serial.println("[SETUP] Initialization complete!");
  Serial.println("========================================\n");

  // Publish startup message
  mqttPublish(TOPIC_STATUS, "{\"status\":\"online\",\"version\":\"1.0\"}");
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

void setupMotors() {
  // Front-Left motor
  pinMode(MOTOR_FL.in1, OUTPUT);
  pinMode(MOTOR_FL.in2, OUTPUT);
  pinMode(MOTOR_FL.en, OUTPUT);
  digitalWrite(MOTOR_FL.in1, LOW);
  digitalWrite(MOTOR_FL.in2, LOW);
  analogWrite(MOTOR_FL.en, 0);

  // Front-Right motor
  pinMode(MOTOR_FR.in1, OUTPUT);
  pinMode(MOTOR_FR.in2, OUTPUT);
  pinMode(MOTOR_FR.en, OUTPUT);
  digitalWrite(MOTOR_FR.in1, LOW);
  digitalWrite(MOTOR_FR.in2, LOW);
  analogWrite(MOTOR_FR.en, 0);

  // Back-Left motor
  pinMode(MOTOR_BL.in1, OUTPUT);
  pinMode(MOTOR_BL.in2, OUTPUT);
  pinMode(MOTOR_BL.en, OUTPUT);
  digitalWrite(MOTOR_BL.in1, LOW);
  digitalWrite(MOTOR_BL.in2, LOW);
  analogWrite(MOTOR_BL.en, 0);

  // Back-Right motor
  pinMode(MOTOR_BR.in1, OUTPUT);
  pinMode(MOTOR_BR.in2, OUTPUT);
  pinMode(MOTOR_BR.en, OUTPUT);
  digitalWrite(MOTOR_BR.in1, LOW);
  digitalWrite(MOTOR_BR.in2, LOW);
  analogWrite(MOTOR_BR.en, 0);

  Serial.println("[MOTOR] Motor pins initialized");
}

/**
 * Set motor speed
 * motor: 0=FL, 1=FR, 2=BL, 3=BR
 * speed: -255 (full reverse) to 255 (full forward)
 */
void setMotor(uint8_t motor, int speed) {
  if (motor > 3)
    return;

  // Clamp speed
  speed = constrain(speed, -255, 255);

  MotorPin pin;
  switch (motor) {
  case 0:
    pin = MOTOR_FL;
    break;
  case 1:
    pin = MOTOR_FR;
    break;
  case 2:
    pin = MOTOR_BL;
    break;
  case 3:
    pin = MOTOR_BR;
    break;
  default:
    return;
  }

  if (speed > 0) {
    // Forward
    digitalWrite(pin.in1, HIGH);
    digitalWrite(pin.in2, LOW);
  } else if (speed < 0) {
    // Reverse
    digitalWrite(pin.in1, LOW);
    digitalWrite(pin.in2, HIGH);
    speed = -speed;
  } else {
    // Stop
    digitalWrite(pin.in1, LOW);
    digitalWrite(pin.in2, LOW);
  }

  analogWrite(pin.en, speed);
  motorSpeeds[motor] = speed;
}

/**
 * Stop all motors
 */
void stopAllMotors() {
  for (int i = 0; i < 4; i++) {
    setMotor(i, 0);
  }
  Serial.println("[MOTOR] All motors stopped");
}

/**
 * Handle differential drive velocity commands (cmd_vel style)
 * linear_x: forward/backward speed (-1.0 to 1.0)
 * angular_z: rotation speed (-1.0 to 1.0)
 */
void cmdVelToDifferentialDrive(float linear_x, float angular_z) {
  const float WHEEL_SEPARATION = 0.2; // 20cm between wheels
  const float WHEEL_RADIUS = 0.03;    // 3cm wheel radius

  // Differential drive kinematics
  float left_speed =
      (linear_x - angular_z * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS;
  float right_speed =
      (linear_x + angular_z * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS;

  // Normalize to -1.0 to 1.0 range
  float max_speed = max(fabs(left_speed), fabs(right_speed));
  if (max_speed > 1.0) {
    left_speed /= max_speed;
    right_speed /= max_speed;
  }

  // Convert to motor speeds (-255 to 255)
  int left_pwm = (int)(left_speed * 255);
  int right_pwm = (int)(right_speed * 255);

  // Set motors (front and back wheels move together)
  setMotor(0, left_pwm);  // Front-Left
  setMotor(2, left_pwm);  // Back-Left
  setMotor(1, right_pwm); // Front-Right
  setMotor(3, right_pwm); // Back-Right

  Serial.printf("[MOTOR] Cmd_vel: linear=%.2f, angular=%.2f | L=%d, R=%d\n",
                linear_x, angular_z, left_pwm, right_pwm);
}

// ============================================================================
// SENSOR READING
// ============================================================================

void setupSensors() {
  pinMode(IR_LEFT, INPUT);
  pinMode(SPEAKER_PIN, OUTPUT);
  digitalWrite(SPEAKER_PIN, LOW);
  Serial.println("[SENSOR] Sensor pins initialized");
}

/**
 * Read IR sensors and publish data
 */
void publishSensorData() {
  irLeftValue = analogRead(IR_LEFT);

  // Convert ADC value to distance estimate (varies by sensor)
  // GP2Y0A21YK0F: ~30cm to 80cm range
  // Distance (cm) ≈ 27.86 * V^-1.15 where V is voltage
  float voltage_left = (irLeftValue / 1023.0) * 3.3;
  float distance_left =
      voltage_left > 0 ? 27.86 * pow(voltage_left, -1.15) : 255.0;

  // Create JSON payload
  DynamicJsonDocument doc(256);
  doc["ir_left_raw"] = irLeftValue;
  doc["ir_left_cm"] = distance_left;
  doc["timestamp_ms"] = millis();

  // Publish
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

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.printf("\n[WiFi] Connected! IP: %s\n",
                  WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[WiFi] Failed to connect");
  }
}

void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiConnected) {
      wifiConnected = false;
      Serial.println("[WiFi] Lost connection");
    }
    // Attempt reconnection
    WiFi.begin(SSID, PASSWORD);
  } else if (!wifiConnected) {
    wifiConnected = true;
    Serial.println("[WiFi] Reconnected");
  }
}

void ensureMQTTConnected() {
  if (!wifiConnected)
    return;

  if (!mqttClient.connected()) {
    Serial.println("[MQTT] Attempting connection...");

    if (mqttClient.connect(MQTT_CLIENT_ID)) {
      mqttConnected = true;
      Serial.println("[MQTT] Connected!");

      // Subscribe to topics
      mqttClient.subscribe(TOPIC_CMD_VEL);
      mqttClient.subscribe(TOPIC_MOTOR_CMD);
      mqttClient.subscribe(TOPIC_AUDIO_CMD);
      mqttClient.subscribe(TOPIC_CONFIG);

      // Publish online status
      mqttPublish(TOPIC_STATUS, "{\"status\":\"connected\"}");
    } else {
      Serial.printf("[MQTT] Connection failed, rc=%d\n", mqttClient.state());
    }
  } else {
    mqttClient.loop();
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  // Convert payload to string
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.printf("[MQTT] Received topic: %s, payload: %s\n", topic,
                message.c_str());

  // Route to appropriate handler
  if (strcmp(topic, TOPIC_CMD_VEL) == 0) {
    handleCmdVel(message);
  } else if (strcmp(topic, TOPIC_MOTOR_CMD) == 0) {
    handleMotorCommand(message);
  } else if (strcmp(topic, TOPIC_AUDIO_CMD) == 0) {
    handleAudioCommand(message);
  } else if (strcmp(topic, TOPIC_CONFIG) == 0) {
    handleConfigCommand(message);
  }
}

/**
 * Handle cmd_vel messages (geometry_msgs/Twist style)
 * Expected format: {"linear":{"x":0.5},"angular":{"z":0.1}}
 */
void handleCmdVel(const String &payload) {
  DynamicJsonDocument doc(256);
  DeserializationError error = deserializeJson(doc, payload);

  if (error) {
    Serial.printf("[CMD_VEL] JSON parse error: %s\n", error.c_str());
    return;
  }

  float linear_x = doc["linear"]["x"] | 0.0;
  float angular_z = doc["angular"]["z"] | 0.0;

  cmdVelToDifferentialDrive(linear_x, angular_z);
}

/**
 * Handle direct motor commands
 * Expected format: {"fl":100,"fr":100,"bl":100,"br":100}
 * Or: {"fl":100,"fr":100,"bl":100,"br":100} for individual control
 */
void handleMotorCommand(const String &payload) {
  DynamicJsonDocument doc(256);
  DeserializationError error = deserializeJson(doc, payload);

  if (error) {
    Serial.printf("[MOTOR_CMD] JSON parse error: %s\n", error.c_str());
    return;
  }

  // Get motor speeds (default to 0 if not present)
  int fl = doc["fl"] | 0;
  int fr = doc["fr"] | 0;
  int bl = doc["bl"] | 0;
  int br = doc["br"] | 0;

  // Set motors
  setMotor(0, fl);
  setMotor(1, fr);
  setMotor(2, bl);
  setMotor(3, br);

  Serial.printf("[MOTOR_CMD] FL=%d, FR=%d, BL=%d, BR=%d\n", fl, fr, bl, br);
}

/**
 * Handle audio commands (speaker buzzer, etc.)
 * Expected format: {"buzz":1,"freq":1000,"duration":100}
 */
void handleAudioCommand(const String &payload) {
  DynamicJsonDocument doc(256);
  DeserializationError error = deserializeJson(doc, payload);

  if (error) {
    Serial.printf("[AUDIO_CMD] JSON parse error: %s\n", error.c_str());
    return;
  }

  if (doc["buzz"] == 1) {
    int freq = doc["freq"] | 1000;
    int duration = doc["duration"] | 100;

    Serial.printf("[AUDIO_CMD] Buzzing: freq=%d Hz, duration=%d ms\n", freq,
                  duration);

    // Simple buzzer using tone (PWM)
    tone(SPEAKER_PIN, freq, duration);
  }
}

/**
 * Handle configuration commands
 * Expected format: {"ssid":"new_ssid","password":"new_pass"}
 */
void handleConfigCommand(const String &payload) {
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, payload);

  if (error) {
    Serial.printf("[CONFIG] JSON parse error: %s\n", error.c_str());
    return;
  }

  Serial.println(
      "[CONFIG] Configuration received (not implemented for runtime update)");
}

void mqttPublish(const char *topic, const char *payload) {
  if (mqttConnected) {
    if (mqttClient.publish(topic, payload)) {
      Serial.printf("[MQTT] Published to %s: %s\n", topic, payload);
    } else {
      Serial.printf("[MQTT] Failed to publish to %s\n", topic);
    }
  }
}

// ============================================================================
// TELEMETRY
// ============================================================================

void publishTelemetry() {
  if (!mqttConnected)
    return;

  // Read battery voltage (example: using ADC)
  uint16_t adcValue = analogRead(A0);
  batteryVoltage = (adcValue / 1023.0) * 3.3 * 5; // Assuming voltage divider

  // Create telemetry JSON
  DynamicJsonDocument doc(512);
  doc["uptime_ms"] = millis();
  doc["battery_voltage"] = batteryVoltage;
  doc["wifi_signal_strength"] = WiFi.RSSI();
  doc["mqtt_connected"] = mqttConnected;
  doc["motor_fl"] = motorSpeeds[0];
  doc["motor_fr"] = motorSpeeds[1];
  doc["motor_bl"] = motorSpeeds[2];
  doc["motor_br"] = motorSpeeds[3];
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
  // MQTT client loop (processes incoming messages)
  if (mqttConnected) {
    mqttClient.loop();
  }

  // Small delay to prevent watchdog timeout
  delay(10);
}
