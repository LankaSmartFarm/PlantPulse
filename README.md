# 🌿 PlantPulse – Universal IoT Device for Smart Agriculture

<img src="https://raw.githubusercontent.com/LankaSmartFarm/PlantPulse/develop/PlantPulse.png" width="128" height="128" alt="PlantPulse Logo" />

**PlantPulse** is an open-source, rugged, and intelligent IoT hardware platform designed as part of the LankaSmartFarm DPI initiative. It enables precision farming through sensor-driven insights and remote control, built specifically for the tropical agriculture challenges of Sri Lanka and similar regions.

This universal device combines sensing, communication, and control in a modular, scalable way—empowering farmers and AgriTech developers with real-time actionable data and automation.

---

## 🚀 Key Features

### 🌱 Sensor Suite (7-in-1 Soil Health Monitoring)
- Monitors:
  - **pH**
  - **Soil Moisture**
  - **Soil Temperature**
  - **Electrical Conductivity (EC)**
  - **Nitrogen (N)**
  - **Phosphorus (P)**
  - **Potassium (K)**  
- Periodic sensing powered by a separate 12V replaceable battery to preserve energy for critical reads.

### 🔊 Acoustic Monitoring & Alerting
- **Speaker + Microphone** for:
  - Insect repellent sound pulses
  - Audible alarms
  - Acoustic anomaly detection (under research)

### 🎯 Visual & Motion Sensing *(Research Stage)*
- Bug detection via visual traps
- Animal trespass monitoring using computer vision models or IR sensors

### 💧 Smart Irrigation & Fertigation Control
- Remotely control **drip irrigation** systems
- Activate **nutrient delivery** only when required

### 🔌 Controllable AUX & GPIO Ports
- Support for up to **10 auxiliary IOs**
- Configurable for lighting, spraying, fans, or custom actuators

---

## 🧠 Architecture

### 🌐 Central IoT Controller
- Acts as the **brain** of the PlantPulse network
- Connects to the cloud for telemetry reporting and receiving commands
- Commands child sensors dynamically—no need for hardcoded logic on each device
- Manages sensor behaviour and firmware updates centrally

### 📡 Child Sensor Nodes
- Connected via **RF or LoRa** for long-range, low-power communication
- Powered by:
  - **MCU battery** (rechargeable via compact solar panel array)
  - **12V sensing battery** for heavy sensor reads
- Contain **RGB status LEDs** for local feedback (alarm, status, fault, etc.)
- Built-in **self-check and debugging** capabilities to detect sensor connectivity or hardware faults

---

## 🛡️ Rugged Design

- **IP67-rated** enclosure
- Engineered to survive:
  - Tropical humidity
  - Heavy rainfall
  - High UV exposure
  - Impacts from falling fruits like coconuts or mangoes
- Can be securely fixed to the ground

---

## 🔧 Open Hardware Goals

PlantPulse aims to be:
- Fully **open-source** (hardware schematics + firmware)
- Modular and extensible
- Built using off-the-shelf components wherever possible
- Maintainable and upgradeable in the field with **no need for full reprogramming**

---

## 🌍 Why It Matters

- **Reduces manual labour and guesswork** in plantation and field farming
- **Supports climate-resilient agriculture** by providing precise micro-level insights
- **Scales easily** from backyard farms to large estates
- Aligns with **Digital Public Infrastructure (DPI)** and **DPG** principles

---

## 🧩 Project Status

- ✅ Hardware architecture and component design – In Progress  
- ⚙️ Firmware development – Early Stage  
- 📶 Communication protocol – RF/LoRa planned  
- 🔬 Bug and trespass detection – Research stage  
- 🔋 Solar & battery power testing – Prototyping  

Stay tuned for schematics, BOM, and reference firmware.

---

## 🤝 Contributing

We welcome hardware tinkerers, firmware developers, Agri researchers, and field testers.

Check our [CONTRIBUTING.md](CONTRIBUTING.md) to get started.

---

## 📜 License

This project will be released under an open hardware-compliant license (e.g., CERN-OHL or Solderpad License). Final terms TBA.

---

## 📬 Contact & Updates

- 🌐 [LankaSmartFarm GitHub](https://github.com/LankaSmartFarm)


---

> “Data grows when rooted in the soil. PlantPulse makes the invisible visible — one sensor at a time.”
