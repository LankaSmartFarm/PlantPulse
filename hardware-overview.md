# 🔩 PlantPulse Hardware Overview

This document outlines the hardware architecture and component design of the **PlantPulse** universal smart farming IoT device.

---

## 📦 System Architecture

### 🧠 Central IoT Controller

- **Role**: Central command unit for all child sensor nodes
- **Responsibilities**:
  - Connects to the cloud (Wi-Fi/4G/LTE)
  - Receives commands from cloud services
  - Sends telemetry data (sensor readings, device health)
  - Dispatches control signals to child sensor nodes
- **Features**:
  - Local command queuing and scheduling
  - Minimal power consumption with sleep cycles
  - Modular firmware upgradable remotely

### 🌿 Child Sensor Nodes (Field Units)

- **Communication**: RF/LoRa modules (up to 1–2 km LOS)
- **Power System**:
  - Rechargeable battery (MCU operation)
  - 12V replaceable battery (for 7-in-1 sensing only)
  - Solar charging system (4x calculator-sized panels)
- **Indicators**: RGB LED for:
  - Alarms
  - Sensor health
  - Activity indication
- **Self-diagnostics**: Sensor connection and health checks

---

## 🧪 Sensor Suite

### 🔬 7-in-1 Soil Quality Sensor

- pH
- Soil Moisture
- Soil Temperature
- Electrical Conductivity (EC)
- Nitrogen (N)
- Phosphorus (P)
- Potassium (K)

### 🔊 Acoustic Subsystem

- **Microphone**: Insect detection, environmental sound capture
- **Speaker**: Alarm output, sound-based repellent (experimental)

### 👀 Visual Detection *(Under Research)*

- Bug trap imaging and recognition
- Animal trespass detection using camera module + CV

---

## 🔌 AUX & GPIO Expansion

- **Up to 10 GPIOs** for:
  - Relays
  - Pumps
  - Lights
  - External devices
- Fully configurable via central controller commands

---

## 🛡️ Enclosure & Protection

- **IP67-rated** shell
- Designed to resist:
  - Rain, humidity, and heat
  - Impact from falling fruits (e.g., coconuts, mangoes)
  - Ground vibrations and animals
- Ground-anchorable

---

## 🧩 Component Sourcing

- Off-the-shelf modules where possible
- Open schematics and BOM to be published

---

## 🗺️ Deployment Topologies

- **Standalone Node**: Single unit operation for isolated crops
- **Cluster Mode**: One central controller managing up to 50+ nodes
- **Mesh Expansion**: Planned support for mesh networking in Phase 2

---

> Want to help with circuit design or PCB layout? Start a discussion or open an issue!
