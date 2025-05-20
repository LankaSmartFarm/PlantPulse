#include <WiFi.h>
#include <ModbusMaster.h>
#include <PubSubClient.h>

// -------------------------------
// RS485 and Modbus Setup
// -------------------------------
#define RX_PIN 16                  // ESP32 UART RX2 pin connected to RO of MAX485
#define TX_PIN 17                  // ESP32 UART TX2 pin connected to DI of MAX485
#define MAX485_CONTROL_PIN 4       // DE and RE of MAX485 tied together and controlled via GPIO
#define SLAVE_ID 0x01              // Modbus slave address of the soil sensor

ModbusMaster node;                 // Modbus master object

// -------------------------------
// Wi-Fi Credentials
// -------------------------------
const char* ssid = "ssid";
const char* password = "password";

// -------------------------------
// MQTT Broker Configuration
// -------------------------------
const char* mqtt_server = "mqtt_host.example.com";
const int mqtt_port = 1883;
const char* mqtt_username = "dsdsdasda";      // Device access token or MQTT credentials
const char* mqtt_password = "asrwgsdgdsg";      // Password if required (optional in ThingsBoard)
const char* mqtt_client_id = "sdsokdfds";     // Client ID for MQTT session
const char* mqtt_topic = "v1/devices/me/telemetry";      // MQTT topic for sending telemetry data

WiFiClient espClient;
PubSubClient client(espClient);

// -------------------------------
// Timers and Polling Interval
// -------------------------------
unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 10000; // Sensor read interval in milliseconds (10 seconds)

// -------------------------------
// RS485 Transmit/Receive Control
// -------------------------------
void preTransmission() {
  digitalWrite(MAX485_CONTROL_PIN, HIGH); // Enable transmission
}
void postTransmission() {
  digitalWrite(MAX485_CONTROL_PIN, LOW);  // Enable reception
}

// -------------------------------
// Connect to Wi-Fi
// -------------------------------
void connectWiFi() {
  Serial.print("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected.");
}

// -------------------------------
// Connect to MQTT Broker
// -------------------------------
void connectMQTT() {
  client.setServer(mqtt_server, mqtt_port);
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect(mqtt_client_id, mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT broker.");
    } else {
      Serial.print("Failed, rc=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

// -------------------------------
// Publish Sensor Data to MQTT
// Units: 
//  - temperature (°C)
//  - moisture (%)
//  - pH (0–14)
//  - conductivity (µS/cm)
//  - nitrogen, phosphorus, potassium (mg/kg)
//  - salinity (mg/kg)
//  - tds (mg/L)
// -------------------------------
void publishSensorData(float temp, float moisture, float pH, uint16_t cond,
                       uint16_t N, uint16_t P, uint16_t K, uint16_t salinity, uint16_t tds) {
  char payload[256];
  snprintf(payload, sizeof(payload),
           "{\"temperature\":%.1f,\"moisture\":%.1f,\"pH\":%.1f,"
           "\"conductivity\":%u,\"nitrogen\":%u,\"phosphorus\":%u,"
           "\"potassium\":%u,\"salinity\":%u,\"tds\":%u}",
           temp, moisture, pH, cond, N, P, K, salinity, tds);

  Serial.print("Publishing: ");
  Serial.println(payload);
  client.publish(mqtt_topic, payload, true); // Send retained telemetry message
}

// -------------------------------
// Setup Function
// -------------------------------
void setup() {
  Serial.begin(115200);

  // Initialize RS485 serial port (UART2)
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  // Setup control pin for MAX485
  pinMode(MAX485_CONTROL_PIN, OUTPUT);
  digitalWrite(MAX485_CONTROL_PIN, LOW); // Default to receive mode

  // Initialize Modbus communication
  node.begin(SLAVE_ID, Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  connectWiFi();
  connectMQTT();

  Serial.println("System ready.");
}

// -------------------------------
// Main Loop
// -------------------------------
void loop() {
  client.loop(); // Maintain MQTT connection

  if (millis() - lastReadTime >= READ_INTERVAL) {
    // Read 9 holding registers starting from 0x00
    uint8_t result = node.readHoldingRegisters(0x00, 9);
    if (result == node.ku8MBSuccess) {
      // Each register response based on 7-in-1 sensor datasheet
      float moisture    = node.getResponseBuffer(0) / 10.0;     // % soil moisture
      float temperature = node.getResponseBuffer(1) / 10.0;     // °C
      uint16_t cond     = node.getResponseBuffer(2);            // µS/cm conductivity
      float pH          = node.getResponseBuffer(3) / 10.0;     // pH level
      uint16_t nitrogen = node.getResponseBuffer(4);            // mg/kg
      uint16_t phosphorus = node.getResponseBuffer(5);          // mg/kg
      uint16_t potassium  = node.getResponseBuffer(6);          // mg/kg
      uint16_t salinity   = node.getResponseBuffer(7);          // mg/kg
      uint16_t tds        = node.getResponseBuffer(8);          // mg/L total dissolved solids

      publishSensorData(temperature, moisture, pH, cond, nitrogen, phosphorus, potassium, salinity, tds);
    } else {
      Serial.print("Modbus read failed: ");
      Serial.println(result);
    }

    lastReadTime = millis(); // Reset timer
  }
}
