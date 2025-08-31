# IP8 - Roboter Energy Management System

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![MATLAB](https://img.shields.io/badge/MATLAB-2024b-orange)](https://www.mathworks.com/)

> Battery Management System (BMS) for Project Work IP8

This repository contains the source code for the Battery Management System (BMS) for the Project Work IP8.

---

## Project Structure

### HPPC - Hardware Processing

This folder contains the microcontroller source code for processing the HPPC-Test (Hybrid Pulse Power Characterization) for battery characterization.

### MATLAB - Simulations

Simulations used to develop the BMS are located in this folder.

**Requirements:**
- MATLAB Version: `2024b`
- Required Extensions:
  - Simulink
  - Simulink Coder
  - Simscape

### ROS2 - Robot Integration

To integrate the BMS into the existing robot system, ROS2 Humble is used.

#### Implemented Nodes

| Node | Description |
|------|-------------|
| **BMS** | SOC and SOH Estimation |
| **battery_monitor** | Simple GUI for BMS results |
| **battery_state_publisher** | Reads current and voltage values and publishes for further usage |

#### BMS Testing

To run the BMS on CSV data, a `sim.py` script is located in the test folder.
The data to load are defined in the script itself.

```bash
source install/setup.zsh
python -m src.bms.test.sim
```


### Current Measurement - Proof of Concept

Proof of concept to read values from a PZEM 0-17 sensor.

**Used Libraries:**
- [libmodbus](https://libmodbus.org/reference/#rtu-context)

**Datasheet:** [PZEM-003/017 User Manual](https://thesunpays.com/downloads/files/Battery%20SOC%20meters/PZEM-003%20017User%20Manual(MEDC300V).pdf)

#### Build and Run

```bash
g++ -std=c++17 client.cpp -lmodbus -o client
./client
```

### Bag Analysis - Data Visualization

Visualizes ROS bag data.

To record a ROS2 bag, use:

```bash
ros2 bag record <TOPICS>
```

BMS data are available on `/bms/state`.

#### Run the Script

Navigate to bag_analysis: `cd bag_analysis`

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python3 bag_analysis <BAG_FILE>
```

