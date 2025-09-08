# 🌱 PlantPulse – Smart Agriculture Monitoring System  

PlantPulse is an **ESP32-based IoT device** for precision agriculture.  
It collects data from **20 Modbus soil sensors** (pH, moisture, temperature, EC, N, P, K), logs it into **FATFS**, synchronizes time with a **DS3231 RTC** and GSM network, and transmits data to the cloud over **SIM800L MQTT**.

---

## 🚀 Features  
- 📡 **MQTT over SIM800L GSM** (custom lightweight MQTT client)  
- 🧪 **20 Soil Sensors** via RS485 Modbus  
- 🕒 **Timekeeping with DS3231 RTC** (synced with GSM time if drift >10s)  
- 💾 **Data Logging with FATFS** (binary files, timestamped filenames)  
- 🔄 **Auto Reconnect** for GSM & MQTT  
- 🗑️ **Reliable Storage** – if publish fails, payload is stored in FATFS and re-sent later  
- 🔌 **Custom ESP-IDF Components** (MQTT, RTC, RS485, FATFS)  

---

## 🔌 Hardware Pinout  

| Peripheral    | ESP32 Pin | Notes                              |
|---------------|-----------|------------------------------------|
| **SIM800L GSM** |           |                                    |
| TX (SIM800L → ESP32 RX) | GPIO15   | UART1 RX (ESP32 receives)       |
| RX (SIM800L ← ESP32 TX) | GPIO16   | UART1 TX (ESP32 transmits)      |
| RESET         | GPIO3    | SIM800L reset control               |
| **RTC DS3231** |           |                                    |
| I2C SDA       | GPIO4    | I2C data line                       |
| I2C SCL       | GPIO5    | I2C clock line                      |
| INT/SQW       | GPIO6    | RTC interrupt / square wave output  |
| **RS485 Modbus** |           |                                    |
| RX            | GPIO17   | UART RX for Modbus slave            |
| TX            | GPIO18   | UART TX for Modbus master           |

*(Update pins as per your final design)*  

---

## 📡 Protocols  

- **MQTT (Custom Lightweight Client over GSM AT commands)**  
- **Modbus RTU** (RS485 multi-drop for soil sensors)  
- **FATFS** (SPI Flash storage for logs & credentials)  
- **I2C** (RTC DS3231 time sync)  

---

## 📂 File Structure  

├── components/ # Custom ESP-IDF components
│ ├── mqtt/ # SIM800L MQTT client
│ ├── myRtc/ # DS3231 RTC driver
│ └── RS_485/ # Modbus RS485 driver
├── main/ # Application code
│ ├── storage/ # FATFS storage handling
│ │ ├── FS.c
│ │ └── FS.h
│ ├── dataLogging.c # Logging logic
│ ├── dataLogging.h
│ ├── main.c # Main application entry point
│ └── CMakeLists.txt
├── partitions.csv # Flash partition table
├── CMakeLists.txt # Project CMake



---

## ⚡ Build & Flash Instructions  

```bash
# Clone repo
git clone https://github.com/yourusername/PlantPulse.git
cd PlantPulse

# Set ESP-IDF environment
. $HOME/esp/esp-idf/export.sh

# Build
idf.py build

# Flash
idf.py -p /dev/ttyUSB0 flash

# Monitor
idf.py monitor
