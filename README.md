# Chispas team – WRO 2025 Project

Welcome to the official repository of the **Chispas team**!  
This space contains all the resources, schematics, models, photos, and code developed for our participation in the **Future Engineers** challenge at the **World Robot Olympiad 2025**.

Here you’ll find detailed information about the design, functionality, and technical documentation of our autonomous vehicle.

---

## 📸 Team Photos

Below are two images that reflect our team’s identity:

| 📷 Official Photo | 😄 Fun Photo |
|------------------|--------------|
| ![Official team photo](t-photos/foto_oficial.jpeg) | ![Fun team photo](t-photos/foto_divertida.jpeg) |

---

## 🚗 Vehicle View

Here you can see the full design of the vehicle from every angle. This detailed view helps to understand the layout of components and overall structure.

| Front | Back | Left |
|--------|---------|-----------|
| ![](v-photos/enfrente.jpeg) | ![](v-photos/trasera.jpeg) | ![](v-photos/izquierda.jpeg) |

| Right | Top | Bottom |
|--------|----------|----------|
| ![](v-photos/derecha.jpeg) | ![](v-photos/arriba.jpeg) | ![](v-photos/abajo.jpeg) |


---

<div align="center" style="display: flex; flex-wrap: wrap; justify-content: center; gap: 20px;">

  <div style="text-align: center;">
    <img src="schemes/component1.jpeg" alt="LiPo Battery" width="200" style="border: 1px solid #ccc; border-radius: 10px; padding: 5px;">
    <p>LiPo Battery 1200mAh 12V</p>
  </div>

  <div style="text-align: center;">
    <img src="schemes/component2.jpeg" alt="Raspberry Pi 4" width="200" style="border: 1px solid #ccc; border-radius: 10px; padding: 5px;">
    <p>Raspberry Pi 4</p>
  </div>

  <div style="text-align: center;">
    <img src="schemes/component3.jpeg" alt="Limelight 3A" width="200" style="border: 1px solid #ccc; border-radius: 10px; padding: 5px;">
    <p>Limelight 3A</p>
  </div>

  <div style="text-align: center;">
    <img src="schemes/component4.jpeg" alt="Servo MG945" width="200" style="border: 1px solid #ccc; border-radius: 10px; padding: 5px;">
    <p>Servo MG945</p>
  </div>

  <div style="text-align: center;">
    <img src="schemes/component5.jpeg" alt="JGA25-370 Motor" width="200" style="border: 1px solid #ccc; border-radius: 10px; padding: 5px;">
    <p>JGA25-370 Motor</p>
  </div>

  <div style="text-align: center;">
    <img src="schemes/component6.jpeg" alt="LM2596 Regulator" width="200" style="border: 1px solid #ccc; border-radius: 10px; padding: 5px;">
    <p>LM2596 Voltage Regulator</p>
  </div>

  <div style="text-align: center;">
    <img src="schemes/component7.jpeg" alt="HC-SR04 Ultrasonic Sensor" width="200" style="border: 1px solid #ccc; border-radius: 10px; padding: 5px;">
    <p>HC-SR04 Ultrasonic Sensor</p>
  </div>

  <div style="text-align: center;">
    <img src="schemes/component8.jpeg" alt="GY-251 Accelerometer" width="200" style="border: 1px solid #ccc; border-radius: 10px; padding: 5px;">
    <p>GY-251 Gyroscope</p>
  </div>

  <div style="text-align: center;">
    <img src="schemes/component10.jpg" alt="TCS3200 Color Sensor" width="200" style="border: 1px solid #ccc; border-radius: 10px; padding: 5px;">
    <p>TCS3200 Color Sensor</p>
  </div>

  <div style="text-align: center;">
    <img src="schemes/component11.webp" alt="L298N Motor Driver" width="200" style="border: 1px solid #ccc; border-radius: 10px; padding: 5px;">
    <p>L298N Motor Driver</p>
  </div>

  <div style="text-align: center;">
    <img src="schemes/component12.webp" alt="3-Pin Switch" width="200" style="border: 1px solid #ccc; border-radius: 10px; padding: 5px;">
    <p>3-Pin Switch</p>
  </div>

</div>


</div>

---

### 📊 Wiring Diagram

Below is the full electromechanical schematic that shows how everything is connected:

![Wiring Diagram](schemes/electromechanical_diagram.png)

> You can find this image and other versions in the [`schemes`](schemes/) folder.

---

## ⚠️ Challenges During Development

Like any real-world project, we faced several challenges that pushed us to adapt, learn quickly, and find creative solutions. Here are some of the main issues we had to overcome:

- **LiDAR connection problems:** At the beginning, we couldn’t get the LiDAR to connect properly with our system. It took us a while to understand the communication protocol and configure it correctly so it would respond as expected.

- **Inaccurate data:** Even after establishing the connection, the data we received from the LiDAR was imprecise and unreliable. We had to tweak the reading filters and run several tests until we got stable and useful measurements.

- **Track border size issues:** We noticed that the track borders were 1 cm smaller than expected. This affected how our robot detected edges and navigated the course, so we decided to increase the borders by 1 cm to match the official measurements and avoid errors.

- **3D part adjustments:** We had to redesign a 3D-printed part that holds the LiDAR sensor. It needed to have just the right height so the LiDAR could rotate freely without hitting other components. This took some trial and error until we got it just right.

- **Camera configuration and lighting:** Object detection didn’t work well at first. We had to adjust settings like exposure and white balance, since lighting conditions were messing up the color and shape recognition. Proper calibration made a big difference.

