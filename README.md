# ESP32 Line Follower & Robotic Arm 🤖

This project is a multifunction autonomous robot based on the **ESP32** microcontroller. It features line-following capabilities, obstacle detection, and a 4-DOF robotic arm for picking up objects (ping-pong balls). It also supports **Manual Mode** via Bluetooth using a smartphone app.

## 🚀 Features

* **Auto Mode:** Follows a black line using 5 IR sensors. When an object is detected by the Ultrasonic sensor (<15cm), the robot stops, picks it up, and resumes.
* **Manual Mode:** Toggle via a physical Push Button. Allows full remote control (Drive + Arm) via Bluetooth Classic.
* **Power Management:** Uses a high-current Li-Ion setup with a Buck Converter to prevent brownouts.
* **Safety Features:** "Non-blocking" button logic and "Stop on Lift" detection.

## 🛠️ Hardware List

* **Microcontroller:** ESP32 Dev Module
* **Motor Driver:** L298N
* **Power:** 7.4V Li-Ion Battery (2x 18650)
* **Regulator:** LM2596 Buck Converter (Set to 5.5V for Servos)
* **Motors:** 2x DC Motors + Wheels
* **Arm:** 4-DOF Acrylic Arm (using 3x SG90 Servos)
* **Sensors:**
    * 5-Channel IR Line Sensor Module
    * HC-SR04 Ultrasonic Sensor
* **Input:** Push Button (Mode Toggle)

## 🔌 Pin Mapping

| Component | ESP32 Pin | Note |
| :--- | :--- | :--- |
| **Right Motor (Speed/EN)** | GPIO 21 | ENA |
| **Right Motor (Dir)** | GPIO 15, 19 | IN1, IN2 |
| **Left Motor (Speed/EN)** | GPIO 17 | ENB |
| **Left Motor (Dir)** | GPIO 5, 4 | IN3, IN4 |
| **Shoulder Servo** | GPIO 26 | |
| **Elbow Servo** | GPIO 14 | |
| **Claw Servo** | GPIO 27 | |
| **Ultrasonic Trig** | GPIO 22 | |
| **Ultrasonic Echo** | GPIO 23 | |
| **IR Sensors** | 25, 33, 32, 35, 34 | Left to Right |
| **Push Button** | GPIO 13 | Connect to GND |
| **Status LED** | GPIO 2 | Onboard LED |

## ⚙️ Installation & Setup

1.  **Install Libraries:**
    * Open Arduino IDE.
    * Go to **Sketch** -> **Include Library** -> **Manage Libraries**.
    * Search for and install: `ESP32Servo` (by Kevin Harrington).
    * *Note: `BluetoothSerial` is built-in to the ESP32 board definitions.*

2.  **Calibrate Servos:**
    * Before attaching the arm horns, run the code and ensure `shoulderHome` and `elbowHome` are set to 90.
    * Adjust the `Reach` variables in the code if the arm hits the floor.

3.  **Bluetooth App:**
    * Download **"Serial Bluetooth Terminal"** (Android) or any Bluetooth Classic terminal.
    * Pair with **"ESP32_Robot"**.
    * **Controls:**
        * `F` = Forward
        * `B` = Backward
        * `L` = Left
        * `R` = Right
        * `S` = Stop
        * `X` = Trigger Pickup Sequence

## 🔋 Power Wiring (Crucial)

* **7.4V Battery** connects directly to **L298N (12V)** and **LM2596 Input**.
* **LM2596 Output** (calibrated to 5.5V) connects to **ESP32 VIN** and **Servo VCC**.
* **ALL Grounds** (Battery, L298N, ESP32, Buck Converter) must be connected together.

## 📝 License

This project is open-source. Feel free to modify and use it!
