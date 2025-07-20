# room-heater

Heater Control System
Overview
This project implements a robust heater control system using an Arduino-compatible microcontroller, simulated within the Wokwi online development environment. The system utilizes a DS18B20 temperature sensor to monitor environmental conditions, controls a simulated heater (represented by an LED), and provides visual feedback through an indicator LED. The system features a state machine with five distinct states—Idle, Heating, Stabilizing, Target Reached, and Overheat—ensuring precise temperature management and safety. All operational data is logged via the Serial Monitor for real-time monitoring and debugging.
Project Details

Author: Bhushan Irukulla
Date: July 20, 2025
Platform: Wokwi Simulator
Microcontroller: Arduino Uno
Submission Deadline: July 22, 2025, 09:07 AM IST

Hardware Requirements

Microcontroller: Arduino Uno or compatible board.
Temperature Sensor: DS18B20 (connected to digital pin 2) with a 4.7kΩ pull-up resistor to 5V.
Heater: Simulated using an LED connected to digital pin 3.
Indicator LED: Connected to digital pin 4 for visual status feedback.
Additional Components: Basic wiring and a Wokwi-compatible simulation setup.
Power Supply: 5V provided by the Arduino Uno (simulated in Wokwi).

Software Requirements

Development Environment: Arduino IDE or Wokwi online simulator.
Libraries:
OneWire (for DS18B20 communication over a single data line).
DallasTemperature (for reading temperature data from the DS18B20).


Code File: heater_control.ino (included in this repository).

Setup Instructions
1. Wokwi Simulation Setup

Access the project simulation at: https://wokwi.com/projects/436968190657623041.
Verify the following components in the Wokwi diagram:
Arduino Uno as the central microcontroller.
DS18B20 temperature sensor with data pin connected to pin 2, VCC to 5V, GND to GND, and a 4.7kΩ pull-up resistor between pin 2 and 5V.
Heater LED (anode to pin 3, cathode through a 220Ω resistor to GND).
Indicator LED (anode to pin 4, cathode through a 220Ω resistor to GND).


Ensure all connections are intact and match the schematic.

2. Code Installation

Open heater_control.ino in the Arduino IDE or Wokwi code editor.
Install the required libraries (OneWire and DallasTemperature) via the Arduino Library Manager if using the IDE, or confirm their availability in Wokwi.
Upload the code to the simulated Arduino Uno in Wokwi.

3. Running the Simulation

Click the “Play” button in Wokwi to start the simulation.
Open the Serial Monitor (set to 9600 baud) to view real-time logs of temperature and system state.
Adjust the DS18B20 temperature in the Wokwi properties panel (e.g., 20°C, 25°C, 35°C) to test state transitions and observe LED behavior.

Functionality
State Machine
The system employs a finite state machine to manage heating based on temperature thresholds:



State
Temperature Range
Behavior



Idle
< 24.0°C
Heater and indicator LEDs remain off.


Heating
24.0°C to < 25.0°C
Heater LED turns on to increase temperature; indicator LED mirrors this state.


Stabilizing
25.0°C to < 26.0°C
Heater may toggle to maintain temperature near the target; indicator LED active.


Target Reached
26.0°C to < 35.0°C
Heater LED turns off; indicator LED off; target temperature achieved.


Overheat
≥ 35.0°C
Heater and indicator LEDs turn off for safety.


Key Features

Temperature Monitoring: Continuously reads temperature from the DS18B20 sensor.
Heater Control: Simulates heater activation via an LED on pin 3, turning on when heating is required.
Indicator Feedback: An LED on pin 4 provides visual confirmation of the heater’s state.
Logging: Logs temperature (in °C) and current state to the Serial Monitor every second.
Error Handling: Detects and reports a disconnected or invalid temperature sensor.

Testing and Validation

Test Cases:
20°C: System enters "Heating" state, both LEDs on.
25°C: Transitions to "Stabilizing" state.
30°C: Reaches "Target Reached" state, LEDs off.
35°C: Enters "Overheat" state, LEDs off.


Verification: Confirmed via Serial Monitor logs and LED status in Wokwi simulation.

Future Improvements

Overheat Protection: Integrate a buzzer or additional LED to alert users of critical temperatures (e.g., >50°C).
Multiple Heating Profiles: Add support for user-selectable temperature settings (e.g., 20°C, 25°C, 30°C) via serial input or buttons.
Advanced Connectivity: Implement Bluetooth Low Energy (BLE) for remote monitoring and control via a mobile app.
Precision Control: Incorporate a PID (Proportional-Integral-Derivative) algorithm for finer temperature regulation.
Real-Time Scheduling: Use a real-time clock (RTC) or FreeRTOS to schedule heating periods.

Known Limitations

The Wokwi simulation requires manual temperature adjustments, unlike a real-world sensor that responds to environmental changes.
Current implementation lacks persistent storage for settings (e.g., EEPROM usage).

License
This project is for educational purposes only. Feel free to modify and use it under the MIT License.
Contact
For questions or feedback, contact Bhushan Irukulla.
