# 🤖 Project CHISPA: Components & Sensor Integration

Welcome to the documentation for **Project CHISPA**, our robot designed for the WRO 2025 Future Engineers Challenge! This README provides an in-depth look at the hardware that powers our autonomous system, detailing each component's role, integration logic, and the engineering challenges we overcame during development.

Our robot leverages a **hybrid approach**, combining robust modular components from an existing kit with custom-designed 3D-printed parts. This strategy allowed us to harness the reliability of pre-tested hardware while creating tailored solutions for the mission's unique demands.

## 🗺️ Table of Contents

1. [🔋 Power Supply]

2. [💻 Main Controllers]

3. [👁️ Object Detection & Tracking]

4. [🚫 Removed Component: Lidar]

5. [🔄 Motor Control]

6. [🔊 Sensors for Navigation and Detection]

7. [🔌 Power & Regulation]

8. [🧠 How It All Works Together]

## 🔋 Power Supply

The backbone of our robot's operation, ensuring consistent power delivery to all systems.

* **LiPo Battery 1200mAh 12V**

  * **Role:** Powers the entire robot, supporting both mobility and computational processing throughout the mission.

  * **Integration Logic:** Provides stable power output for motors, sensors, Raspberry Pi, and Arduino.

  * **Key Feature:** We Integrated a **voltage regulator** to prevent current spikes and protect delicate components, it is the LM2596 which provides us with stable 12V to 5V for the raspberry pi, arduino, servo and sensors.
 
  * **Issue & Solution:** Mitigated early issues of sudden shutdowns during intensive operations. A **battery monitor** was incorporated for real-time voltage measurement and low-level alerts.

## 💻 Main Controllers

The brains and brawn, orchestrating high-level decision-making and low-level hardware control.

* **Raspberry Pi 4**

  * **Role:** Central processing hub. Responsible for **image processing**, **decision-making logic**, and **sensor data management**.

  * **Integration Logic:** Runs Python scripts to interpret input from the Limelight and communicates with the Arduino via a serial interface.

  * **Capabilities:** High-speed processing for simultaneous tasks like pathfinding, obstacle avoidance, and turn correction.

  * **Enhancements:** Includes an **external cooling fan** and **heat sinks** for optimal temperature maintenance during extended operations.

* **Arduino MEGA**

  * **Role:** Controls low-level tasks such as **PWM motor control** and **real-time sensor readings** (e.g., ultrasonic distance, servo movements).

  * **Integration Logic:** Acts as a reliable bridge for deterministic behavior in the hardware layer such as the drivetrain itself (servomotor and engine).

  * **Benefits:** Division of responsibilities with the Pi balances computational load and reduces latency. Simplifies hardware debugging and guarantees fast, responsive control signals.

## 👁️ Object Detection & Tracking

Our robot's eyes, enabling autonomous navigation through visual target acquisition.

* **Limelight 3A**

  * **Role:** Powerful camera system adapted from FRC robots to detect and lock onto specific colored or shaped objects.

  * **Usage:** Enables autonomous navigation by marking visual targets and aligning the robot accordingly.

  * **Challenges:**

    * Required a well-lit and consistent environment for reliable data.

    * **Solution:** Adjusted the robot’s onboard lighting and modified thresholds in the Limelight software to improve reliability.

    * **Solution:** Integrated a **protective mount** and **anti-vibration dampers** to stabilize image input.

## 🚫 Removed Component: LDrobot Lidar LD19

An initial consideration for environmental mapping that was ultimately replaced.

* **LDrobot Lidar LD19**

  * **Initial Purpose:** Selected for real-time SLAM (Simultaneous Localization and Mapping) and environment scanning.

  * **Why We Stopped Using It:**

    * Provided unstable and inconsistent results in small, confined spaces (the LIDAR worked better in way more open fields up to 12m).

    * Difficulty detecting nearby walls and objects due to reflection noise (plots randomly appeared and then stopped appearing after a second or two, making it impossible to do multiple sampling).

    * Calibration proved time-consuming, and performance did not justify the complexity and time required to get it working.

  * **Our Solution:** Replaced with strategically placed **ultrasonic sensors** and a reliable **gyroscope** to manage spatial awareness with much greater efficiency. This switch significantly improved consistency and reduced computational load.

## 🔄 Motor Control

Driving the robot's movement with precision and power.

* **L298N Motor Driver**

  * **Role:** Dual H-Bridge driver allowing for **direction control** and **PWM-based speed adjustment**.

  * **Issue & Solution:** Initial controller suffered overheating; the L298N offered a robust and thermally safer alternative. Equipped with a **built-in heat sink** to improve heat dissipation.

* **JGA25-370 Gear Motors**

  * **Role:** Provide strong torque and reliable performance, especially during turns and ramp climbs (**yes, this is a problem at regionals, theres quite literally speedbumps**).

  * **Features:** Compatible with our power source and robust enough for repeated high-load operations. High RPM ensures a lot of speed and the integrated encoder ensures accurate control in precision-based movement challenges such as the obstacle round.

* **Servo MG945**

  * **Role:** Handles the system that allows the car to turn left/right.

  * **Features:** Durable metal gears ensure precision and a long operational lifespan.

  * **Integration:** Integrated with custom **3D-printed brackets** to prevent misalignment and slippage.

## 🔊 Sensors for Navigation and Detection

The robot's senses, crucial for understanding its environment and navigating the challenge course.

* **HC-SR04 Ultrasonic Sensors**

  * **Function:** Detect distance to nearby objects and walls.

  * **Technical Accuracy:** \~3mm, max range 4m.

  * **Usage:** Placed on both the front and side of the robot to measure proximity and assist in avoiding wall collisions. Critical for maintaining correct lane alignment and determining turn timings.

  * **Enhancement:** Added **foam dampers** around the sensors to reduce interference from ground reflections.

* **GY-251 Accelerometer + Gyroscope (MPU6050-based)**

  * **Function:** Detects angular velocity and tilt.

  * **Usage:** Guides the robot’s heading during turns and confirms straight-line travel. Essential for consistent 90° turns and recalibration during complex movements.

  * **Placement:** Mounted at the **center of gravity** to reduce noise and increase accuracy.

* **TCS3200 Color Sensor**

  * **Function:** Reads floor tile colors to guide the robot’s path.

  * **Issues and Solutions:**

    * One unit had significant lag and failed to detect color transitions in motion.

    * Another sensor misidentified multiple colors and only read orange with high confidence.

    * **Solution:** Performed extensive **calibration trials** under consistent lighting; replaced the malfunctioning unit with a higher-quality clone.

  **Accuracy Reference (sourced from online comparative tests):**

  | **Color** | **Accuracy (%)** | 
  | Red | 92% | 
  | Green | 89% | 
  | Blue | 94% | 
  | Yellow | 87% | 
  | Orange | 96% | 
  | White | 91% | 
  | Black | 88% | 


## 🔌 Power & Regulation

Ensuring stable and safe power distribution throughout the system.

* **LM2596 Voltage Regulator**

  * **Role:** Converts 12V from the LiPo battery to a stable 5V required by sensors and controllers.

  * **Features:** Fine-tunable and heat-resistant, preventing power-related failures. Easy to replace and adjust for different output requirements during testing phases.

* **3-pin Toggle Switch**

  * **Role:** Allows safe, manual powering of the entire robot without direct battery disconnection.

  * **Placement:** Placed on the back panel for convenient access and safety during handling.

## 🧠 How It All Works Together

Our robot was developed for the WRO 2025 Future Engineers Challenge, which requires navigating a color-coded maze, obstacle avoidance, and accurate maneuvering. We implemented a **layered control strategy** to achieve these objectives:

* **Color Sensing for Path Following:**

  * The robot follows a line/path system on the floor, detecting color patterns using the TCS3200.

  * Decisions (turns, stops, or interactions) are triggered based on detected colors.

  * Logic is reinforced with timeouts and counters to avoid misreads from transient reflections.

* **Dynamic Obstacle Avoidance:**

  * HC-SR04 sensors measure distances in real-time.

  * Upon detecting an object or wall within a predefined threshold, the robot adjusts its trajectory accordingly.

  * In edge cases, the gyroscope is consulted to determine alternative safe paths.

* **Precise Turning & Direction Control:**

  * Using GY-251 gyroscope data, we ensure consistent 90° turns.

  * This helps correct drift and ensures the robot remains aligned with mapped paths.

  * Combined with encoder data for enhanced positional awareness.

* **Centralized Coordination (Pi-Arduino Synergy):**

  * The **Raspberry Pi** processes high-level logic and forwards simplified instructions to the **Arduino**.

  * This modular separation allows for real-time reaction to environmental data while executing long-term movement plans.

  * Logs are stored locally for later debugging and performance analysis.

* **Structural Customization & Optimization:**

  * We modified kit-based chassis with **3D-printed mounts** for better sensor placement and airflow.

  * This helped avoid overheating and mechanical misalignments during long test sessions.

  * The design is compact yet expandable, making it ideal for future missions.

* **Rigorous Component Testing and Iteration:**

  * Every part of the robot underwent unit testing and stress evaluation.

  * We documented all observed limitations and iteratively improved sensor placement and code logic.

  * Testing environments included both simulated and real-field trials.

### 🌟 Conclusion

By meticulously overcoming major hurdles like sensor inconsistency, voltage fluctuations, and Lidar misreadings, we developed a system that is **adaptive, stable, and highly capable** of completing the mission. Our documentation, design, and testing reflect a clear engineering vision with flexibility and resilience at its core.

Project CHISPA embodies a balance between innovation and practicality — leveraging existing resources while smartly adapting to meet the demands of a complex and evolving challenge. Our engineering choices were made with precision, and the modular approach ensures the robot is easy to repair, extend, and reconfigure if necessary. We’re proud of its development and look forward to its performance on the competition field!
