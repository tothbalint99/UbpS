# UbpS: Real-time Continuous Blood Pressure Sensing on an Ultrasound IoT

## Table of Contents
- [Project Description](#project-description)
- [Folder Structure](#folder-structure)

## Project description
This repository opensources the codes and data related to the paper: UbpS: Real-time Continuous Blood Pressure Sensing on an Ultrasound IoT.

## Folder Structure
The project repository is structured as follows:

### Client application
This folder contains the graphical client application and the corresponding test data.

### Code
This folder contains all the desktop implementations of the algorithm tailored for the phantom, the in-vivo algorithm without optimization using a 2 GHz sampling frequency, and the in-vivo algorithm that uses a 2 MHz sampling frequency for the A-mode ultrasound scans.

### Data
The ultrasound recordings related to this project can be found in this folder.

This folder contains the following subfolders:
- `Phantom_measurement/`: Contains the ultrasound recordings on the phantom and the corresponding ground truth pressure value.
- `Subject_measurement/`: Contains the ultrasound recordings made on a real subject.

> **Note:** Due to GitHub's file size limits, the data is **not hosted in this repository**.  
> You can access and download the full dataset from the following Google Drive link:  
> [https://drive.google.com/drive/folders/1IrROS9DMPvO2uPypbxCCy9VnxXq31iUe?usp=sharing](https://drive.google.com/drive/folders/1IrROS9DMPvO2uPypbxCCy9VnxXq31iUe?usp=sharing)

### Embedded code
This folder contains two embedded implementations of the in-vivo algorithm, both using a 2 MHz sampling frequency for the A-mode ultrasound scans. One works with the Client application, the other one is used for profiling and contains the algorithm with minimum overhead, which is the data reading from a header file.