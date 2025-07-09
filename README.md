# Chispas team – Proyecto WRO 2025

¡Bienvenidos al repositorio oficial del equipo **[Chispas team]**!  
Este espacio contiene todos los recursos, esquemas, modelos, fotografías y código desarrollados para nuestra participación en el desafío **Future Engineers** de la **World Robot Olympiad 2025**.

Aquí podrás encontrar información detallada sobre el diseño, funcionamiento y documentación técnica de nuestro vehículo autónomo.

---

## 📸 Fotos del Equipo

A continuación, compartimos dos imágenes que reflejan nuestra identidad como equipo:

| 📷 Foto Oficial | 😄 Foto Divertida |
|----------------|-------------------|
| ![Foto oficial del equipo](t-photos/foto_oficial.jpg) | ![Foto divertida del equipo](t-photos/foto_divertida.jpg) |


## Content

* `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
* `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
* `video` contains the video.md file with the link to a video where driving demonstration exists
* `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
* `src` contains code of control software for all components which were programmed to participate in the competition
* `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
* `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.

## Introduction

_This part must be filled by participants with the technical clarifications about the code: which modules the code consists of, how they are related to the electromechanical components of the vehicle, and what is the process to build/compile/upload the code to the vehicle’s controllers._

## How to prepare the repo based on the template

_Remove this section before the first commit to the repository_

1. Clone this repo by using the `git clone` functionality.
2. Remove `.git` directory
3. [Initialize a new public repository on GitHub](https://github.com/new) by following instructions from "create a new repository on the command line" section (appeared after pressing "Create repository" button).

<h2 align="center">Componentes</h2>

<table>
  <tr>
    <td align="center">
      <img src="IMG/lipo%201200mah%2012v.jpeg" width="120"/><br/>
      <b>Lipo 1200mAh 12V</b>
    </td>
    <td align="center">
      <img src="IMG/raspberrypi4.jpeg" width="120"/><br/>
      <b>Raspberry Pi 4</b>
    </td>
    <td align="center">
      <img src="IMG/limelight%203A%20.jpeg" width="120"/><br/>
      <b>Limelight 3A</b>
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="IMG/servo%20MG945.jpeg" width="120"/><br/>
      <b>Servo MG945</b>
    </td>
    <td align="center">
      <img src="IMG/LDrobot%20lidar%20LD19.jpeg" width="120"/><br/>
      <b>Lidar LD19</b>
    </td>
    <td align="center">
      <img src="IMG/LM%202596.jpeg" width="120"/><br/>
      <b>LM2596</b>
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="IMG/PWM%20MOSFET%20controller.jpeg" width="120"/><br/>
      <b>PWM MOSFET Controller</b>
    </td>
    <td align="center">
      <img src="IMG/motor%20jga25-370.jpeg" width="120"/><br/>
      <b>Motor JGA25-370</b>
    </td>
    <td align="center">
      <img src="IMG/switch%20random%203%20pines.jpeg" width="120"/><br/>
      <b>Switch 3 Pines</b>
    </td>
  </tr>
</table>
