#include <ModbusMaster.h>

// Modbus slave ID (default from datasheet is 0x01)
#define SLAVE_ID 0x01

// RS485 TX/RX pins for ESP32 UART2
#define RX_PIN 16
#define TX_PIN 17

// MAX485 DE+RE control pin
#define MAX485_CONTROL_PIN 4

// Instantiate ModbusMaster
ModbusMaster node;

unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 5000; // 5 seconds
const unsigned long SENSOR_DELAY = 100;

// Control RS485 TX/RX direction
void preTransmission() {
  digitalWrite(MAX485_CONTROL_PIN, HIGH);  // Transmit mode
}

void postTransmission() {
  digitalWrite(MAX485_CONTROL_PIN, LOW);   // Receive mode
}

void printModbusError(uint8_t result) {
  Serial.print("Modbus error: ");
  switch (result) {
    case ModbusMaster::ku8MBIllegalFunction: Serial.println("Illegal Function"); break;
    case ModbusMaster::ku8MBIllegalDataAddress: Serial.println("Illegal Data Address"); break;
    case ModbusMaster::ku8MBIllegalDataValue: Serial.println("Illegal Data Value"); break;
    case ModbusMaster::ku8MBSlaveDeviceFailure: Serial.println("Slave Device Failure"); break;
    default: Serial.println("Unknown Error"); break;
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // RS485 on UART2

  pinMode(MAX485_CONTROL_PIN, OUTPUT);
  digitalWrite(MAX485_CONTROL_PIN, LOW);  // Default to receive mode

  // Initialize Modbus communication
  node.begin(SLAVE_ID, Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  Serial.println("Soil Sensor RS485 Reader Started");
  delay(1000);
}

void loop() {
  if (millis() - lastReadTime >= READ_INTERVAL) {
    Serial.println("\n--- Reading Sensor Values ---");

    uint8_t result = node.readHoldingRegisters(0x00, 9);  // Read registers 0x00 to 0x08
    if (result == node.ku8MBSuccess) {
      float moisture    = node.getResponseBuffer(0) / 10.0;
      float temperature = node.getResponseBuffer(1) / 10.0;
      uint16_t cond     = node.getResponseBuffer(2);
      float pH          = node.getResponseBuffer(3) / 10.0;
      uint16_t nitrogen = node.getResponseBuffer(4);
      uint16_t phosphor = node.getResponseBuffer(5);
      uint16_t potass   = node.getResponseBuffer(6);
      uint16_t salinity = node.getResponseBuffer(7);
      uint16_t tds      = node.getResponseBuffer(8);

      Serial.printf("Moisture:        %.1f %%\n", moisture);
      Serial.printf("Temperature:     %.1f °C\n", temperature);
      Serial.printf("Conductivity:    %u us/cm\n", cond);
      Serial.printf("pH:              %.1f\n", pH);
      Serial.printf("Nitrogen (N):    %u mg/kg\n", nitrogen);
      Serial.printf("Phosphorus (P):  %u mg/kg\n", phosphor);
      Serial.printf("Potassium (K):   %u mg/kg\n", potass);
      Serial.printf("Salinity:        %u mg/kg\n", salinity);
      Serial.printf("TDS:             %u mg/kg\n", tds);

    } else {
      Serial.println("Failed to read sensor values!");
      printModbusError(result);
    }

    Serial.println("-----------------------------");

    lastReadTime = millis();
  }
}
