# 🧠 PlantPulse Firmware Roadmap

This document outlines the development plan for firmware modules across the PlantPulse IoT system.

---

## 🏗️ Architecture

### Firmware Layers:

1. **Device Abstraction Layer (DAL)**  
   - Interfaces directly with sensors, IOs, and power modules

2. **Control Logic Layer**  
   - Executes scheduled actions
   - Interprets central controller commands
   - Handles local events and diagnostics

3. **Communication Layer**  
   - RF/LoRa communication protocol
   - Data encryption & compression
   - OTA firmware update protocol (planned)

4. **Telemetry Layer**  
   - Formats and sends telemetry to central controller
   - Includes timestamps, battery, sensor data, and faults

---

## 📅 Roadmap

### 🔹 Phase 1 – MVP (Q2 2025)
- [x] Basic GPIO toggling from central controller
- [x] Periodic sensor reading
- [x] RGB LED status indicators
- [ ] Basic telemetry reporting
- [ ] Manual reset + debug mode

### 🔸 Phase 2 – Core Features (Q3 2025)
- [ ] LoRa mesh node discovery
- [ ] Event-driven triggers (e.g., moisture too low → activate pump)
- [ ] OTA firmware updates
- [ ] Self-diagnostics on sensor health

### 🔹 Phase 3 – Advanced Intelligence (Q4 2025)
- [ ] AI-assisted edge logic (soil condition learning)
- [ ] Audio event pattern detection
- [ ] Vision-based bug/animal detection (if module available)

---

## 🛠 Development Notes

- Written in C/C++ (Arduino/PlatformIO base)
- Open to MicroPython and Rust prototypes
- GitHub Actions CI for unit testing
- Separate branches for stable vs. experimental firmware

---

> Contributions welcome for modular drivers, LoRa protocol enhancements, or data compression techniques.
