# AVDC_GDP_TeamC
This repository was created in order to manage the code for our Group Design Project (GDP) in MSc AVDC 2022-2023.

**MSc Autonomous Vehicle Dynamics and Control (AVDC) 2022 - 2023, Cranfield Univeristy**

# Contributers
1. Mucahit TASDEMIR
2. Abhay THAJ
3. Aykut SIRMA
4. Fatima ALHASHMI
5. Komsun TAMANAKIJPRASART
6. Mansoor ALMANSOORI
7. Mar VILLALONGA TORRES
8. Mehmet ERKESIM

# Abstract
This group project report presents the design, development, and testing of a multi-agent search 
mission using autonomous quadcopters. The quadcopters were equipped with onboard Jetson 
Nano systems for target identification and classification, utilizing trained datasets for accurate 
object detection. Two drones were built from scratch using standard components, with some
parts 3D printed for customization. The flight controller and state estimators were also designed 
and implemented on the BeagleBone Blue platform. Guidance algorithms were implemented 
to actively follow the target once it was detected. Communication links, developed from 
scratch, allowed for interaction between the drones, the ground station, and the onboard 
JetsonNano. Several indoor tests were conducted using the VICON system to assist in 
localizing the drone's position and attitude, validating the drone's ability to successfully track
desired targets. The situational awareness algorithm for identification and tracking was 
thoroughly tested and verified. The results demonstrate the effectiveness of the multi-agent 
autonomous quadcopter system in real-world search and identification missions.

# Showcase
https://cranfield-my.sharepoint.com/:v:/r/personal/mar_villalongatorres_580_cranfield_ac_uk/Documents/Sequence%20%231(4).mp4?csf=1&web=1&e=luTpeJ

# Introduction
The Group Design Project is a hands-on project for students of AVDC to gain invaluable experience about the autonomous systems and the project design phase cycle for a project. The GDP is completed by 8 students between the dates of 15th November 2022 – 24th March 2023 which is approximately 4 months of time period.

Autonomous systems are inseparable elements of a daily life of today’s world. The key technologies are being developed and evolved in an accelerated manner. The trend in aerial autonomous vehicles is increasing reliable autonomy while not reducing the performance. This requires understanding the trend and enabling technologies. Object detection, identification and tracking using autonomous UAVs can be utilized in many real-life cases such as rescue missions, wildlife protection duties and military applications etc.

The main purpose of GDP is to design, build and demonstrate for an autonomous aerial multi agent system which is capable of object detection, identification and tracking for a given scenario. Through GDP, it is expected to apply the teachings learned during the taught courses and experience an engineering design cycle from system requirement review till final flight demonstration while optimizing the design as the process proceeds. The scenario is provided to students to derive the design requirements. The scenario details are given in the following sections. The team is divided into subteams which are Guidance, Navigation & Control Subteam, Communication & Network Subteam, Situational Awareness Subteam, Task & Mission Control Subteam and Hardware & Integration Subteam.

The report explains the design and integration process, validation and verifications methods and demonstration flight for each subsystem and integrated end product. At the end, a post flight analysis discussion is conducted to assess the achieved objectives. To conclude the report, possible solutions and future work is discussed.  It should be noted that, due to time limitations and in parallel on-going courses, the focus is given onto building a working system before the end of GDP.

# Project Description and Mission Statement
The GDP given is a hardware-based project. The objectives of the GDP can be listed as below:
- To define requirements and performance metrics
- To design 2-3 UAVs
- To design and develop a MAS solution based on 2-3 UAVs. It is required to utilize cooperation strategies and enabling technologies
- To develop methods to verify and validate the proposed solution
- To properly analyse the performance of the proposed solution and come up with meaningful discussions

|![image](https://github.com/Fatma52125/AVDC_GDP_TeamC/assets/133139057/5e139a94-1b77-403a-acbb-6704ad603560)|
|:--:|
|*Mission Scenario*|

It should be noted that the whole scenario should be performed autonomously. The ground control station should be developed by the team.
Into the CDR, the flight area is changed from outdoor flight area to indoor flying arena thus the updated mission schematic is as follows:

|![image](https://github.com/Fatma52125/AVDC_GDP_TeamC/assets/133139057/26ec1fa9-beab-4d8c-b5d6-33d9859692ce)|
|:--:|
|*The indoor flying arena*|



