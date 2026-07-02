
# CortexFlightController
High-performance, low-latency flight control firmware for the STM32H723ZGT6. Optimized for 1.66 kHz synchronized sensor-to-motor loops, hard-mounted LSM6DS3 IMU configurations, and precision PWM output for RC aviation.
<img width="1440" height="1122" alt="drone" src="https://github.com/user-attachments/assets/f20a5a24-29fa-4123-818e-4be97c2f6214" />

<img width="250" height="350" alt="IMG_4452" src="https://github.com/user-attachments/assets/acec6f28-e0d4-4274-8429-9a22ec3163e7" />

CortexFlightController
Cortex is a custom, high-performance drone flight controller firmware written from scratch for ARM Cortex-M microcontrollers. Designed with modularity and precision in mind, this project implements low-level hardware abstraction, robust sensor fusion, and real-time flight control loops without relying on heavy external libraries.

🚀 Features


Custom Flight Control Loop: Highly optimized bare-metal / RTOS-friendly control loops for stable flight dynamics.

Advanced Sensor Fusion: Integrated IMU processing using the LSM6DS3 (via SPI/I2C) for precise attitude (roll, pitch, yaw) estimation.

Modularity: Clean separation between sensor drivers, state estimation, and PID control logic.

🛠️ Tech Stack & Hardware

Language: C 

Target Architecture: ARM Cortex-M7 Series (e.g., STM32)

Primary IMU: LSM6DS3 (6-axis Inertial Measurement Unit)

Build Tools: [STM32CUBEMX,STM32CubeIDE,STM32CUBEPROGRAMMER]

📈 Roadmap & Ongoing Development


1) Implement full PID tuning interface via GUI.
2)  Add support for multiple receiver protocols (PWM / SBUS / IBUS / PPM).
3)  Optimize Mahony filter coefficients
4)  Blackbox logging functionality over SPI Flash or SD Card.
5)  altitude hold using ms5611 + velocity control on vertical direction
