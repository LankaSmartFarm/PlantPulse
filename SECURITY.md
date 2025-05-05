# 🔐 Security Policy – PlantPulse

## 📅 Supported Versions

We are actively developing the project and fixing bugs and vulnerabilities on the `main` and `develop` branches. Please use the latest version whenever possible.

| Version        | Supported          |
|----------------|--------------------|
| `main` (latest) | ✅ Yes              |
| `develop`       | ✅ Yes (dev builds) |
| Older versions  | ❌ No               |

---

## 🛡️ Reporting a Vulnerability

If you discover a security vulnerability in **PlantPulse**:

1. **Do not** open a public issue.
2. Instead, report it privately via:
   - 📧 **Email**: plantpulse@lankasmartfarm.org
   - 📜 Include:
     - Description of the vulnerability
     - Steps to reproduce
     - Impact assessment (if possible)

We will:
- Acknowledge your report within **48 hours**
- Investigate and address the issue within **7 working days**
- Credit you as the reporter (optional)

---

## 🔐 Security Best Practices

- Avoid hardcoded credentials or device identifiers
- Validate all data received via RF/LoRa/telemetry
- Isolate insecure sensor modules using GPIO filters
- Report even minor anomalies that could lead to abuse

---

> Security is a shared responsibility. Let’s keep PlantPulse safe for all its users.
