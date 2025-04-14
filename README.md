Welcome to the official repository of the International Robotic Club of Beihang University! This repository houses code, documentation, and resources for our robotics projects, reflecting our journey from our first competition in 2024 to the present with an advanced embedded development with the ESP32 microcontroller and FreeRTOS for real-time multitasking.

Key Features
ESP32 & FreeRTOS Integration: Modern codebase leveraging FreeRTOS for task scheduling, multi-threading, and efficient resource management on ESP32.
Wireless Capabilities: Built-in Wi-Fi/Bluetooth support for IoT-enabled robotics applications
Sensor Fusion: Integration of IMUs, LiDAR, and environmental sensors for autonomous navigation and decision-making(pending).
Arduino Legacy: Archived examples of past Arduino projects (motor control, PID tuning, sensor interfacing) for historical reference and learning.


Past Projects (Arduino Based):
Autonomous line-following robots with PID control.
Bluetooth-controlled robotic arms using HC-05 modules.
Obstacle-avoidance systems with ultrasonic sensors.
Data logging and telemetry for competition robots.


Current Focus (ESP32 + FreeRTOS):
Real-Time Robotic Control: Multi-threaded tasks for motor control, sensor polling, and communication.
IoT Robotics: Cloud-connected systems for remote monitoring/control via MQTT/HTTP.
Smart Robotics: AI-driven edge inference (e.g., TinyML) for object detection and path planning (In conjunction with openMV h7 plus).
Modular Code Architecture: Reusable drivers and libraries for rapid prototyping and connection with other MCU

Prerequisites: ESP32 development environment (Arduino IDE/PlatformIO/VSCode + ESP-IDF),  FreeRTOS basics (tasks, queues, semaphores), Familiarity with Arduino libraries (for legacy code reference).
