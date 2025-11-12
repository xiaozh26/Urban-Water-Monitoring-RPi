# Raspberry Pi Environmental Monitoring System

This repository contains the Python prototype code developed for the 2023 research paper "Upgrading Urban Water Storage System."

This script is designed to run on a Raspberry Pi, connecting to a suite of environmental sensors. Its primary function is to collect real-time atmospheric and water-level data, which it then uses to calculate the estimated water evaporation rate using the FAO Penman-Monteith formula.

*Note: This code was developed in 2023. It is being uploaded now following a GitHub account migration.*

## 💡 Real-World Application & Purpose

This code was built to solve a critical, real-world problem: **massive water loss from evaporation in urban reservoirs.**

Megacities (like Beijing, as studied in the paper) face extreme water scarcity and high energy demands. Open-surface reservoirs lose enormous amounts of water to the atmosphere.

* **The Problem:** It's difficult to justify the high cost of solutions (like covering reservoirs with solar panels) without first knowing exactly *how much* water is being lost.
* **This Solution:** This script is the **measurement and validation tool** to solve that. By accurately measuring all the environmental variables (temp, humidity, wind, etc.) on-site, it calculates a **real-time evaporation rate**. This baseline data is the essential "proof" needed to quantify the problem and verify the effectiveness of the proposed solution (i.e., "We can save X million m³ of water"), justifying the large-scale engineering project.

## 📄 Associated Research

This code serves as the working prototype and data-collection tool for the following paper:

> **Upgrading Urban Water Storage System: Achieving Water Conservation, Power Generation, Carbon Reduction, and Water Quality Enhancement**
>
> Xiao Zhang
>
> *arXiv (2023)*
>
> **DOI:** `10.48550/arXiv.2308.08553`

## ⚙️ Core Functionality

* **Multi-Sensor Integration:** Interfaces with a variety of hardware:
    * **GPIO:** Reads an **HC-SR04** ultrasonic sensor for water level.
    * **I2C:** Reads an **AHT20** (temp/humidity) and **BMP280** (pressure).
    * **Modbus-RTU:** Communicates via RS485 with a **ZTS-3000-FSJT-N01** anemometer.
* **Evaporation Calculation:** Implements the **FAO Penman-Monteith formula** (`evaporation()`) to calculate an estimated evaporation rate (mm/day) based on the live sensor data.
* **Live Data Dashboard:** Uses `matplotlib` to generate a real-time, multi-plot dashboard showing all incoming sensor values and calculated results.
* **Modbus/CRC Handling:** Includes functions (`crc16`, `checkcrc`, `sendingmod`) for correctly formatting and validating Modbus-RTU command frames for the wind sensor.

## 🛠️ Hardware Requirements

* **Controller:** Raspberry Pi
* **Sensors:**
    * HC-SR04 (Ultrasonic)
    * AHT20 (Temp/Humidity)
    * BMP280 (Pressure)
    * ZTS-3000-FSJT-N01 (Wind Speed)
* **Interface:**
    * RS485-to-USB adapter (e.g., `/dev/ttyUSB0`)

## 🚀 Installation & Usage

1.  **Enable Interfaces:**
    On your Raspberry Pi, use `sudo raspi-config` to enable `I2C`.

2.  **Install Dependencies:**
    ```bash
    pip install RPi.GPIO smbus crcmod pyserial matplotlib
    ```

3.  **Run the Script:**
    The script requires `sudo` for GPIO access.
    ```bash
    sudo python your_script_name.py
    ```
    The `matplotlib` dashboard will open and begin updating. Press `CTRL + C` in the terminal to stop the script and trigger the `GPIO.cleanup()` function.

## 📁 Code Structure

* `water_level(...)`: Handles the GPIO trigger/echo sequence for the HC-SR04.
* `class AHT10`: A helper class that contains methods to read and process data from the I2C sensors (`THP_get()` for AHT20, `THP_cal()` for BMP280).
* `crc16(...)`, `checkcrc(...)`, `sendingmod(...)`: Utility functions for the Modbus-RTU communication.
* `evaporation(...)`: Contains the core scientific calculation (Penman-Monteith).
* `if __name__ == '__main__':`: The main application loop.
    1.  Initializes `matplotlib` plots.
    2.  Enters a `while True:` loop.
    3.  Polls each sensor function.
    4.  Calculates the evaporation rate.
    5.  Updates the `matplotlib` plots and table.
    6.  Pauses briefly before repeating.

## 📄 License

MIT License. See the `LICENSE` file for details.

## 👤 Author

* **Xiao Zhang**
    * GitHub: `[@xiaozh26](https://github.com/xiaozh26)`
