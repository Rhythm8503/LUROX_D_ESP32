# LUROX D Prosthetic Arm VMCA Early Stage Research

In this repository it contains the firmware for the ESP32-S3 and the MaixBit K210. The ESP32-S3-N16R8 utilizes the Arduino Core and Arduino IDE, the Maix Bit K210 utilizes the SiSpeed MaixPy Micropython platform.

# What is a VMCA?
The VMCA is a Visual Modular Contextualization Action algorithm, which is comprised of many sections. This being the Semantics Clustering, Decision Backbone, Action and Vision block. The VMCA is similar to a VLA or Visual Language Action algorithm however utilizes a decoupled network architecture approach than a typical mono-lithic pipeline found in open-source VLA models. 


## Semantics Clusters & Modular Context
This architecture is special in many ways, the first being the Semantics clustering. The Semantics clustering focus is to grab the user's intent and this would be fed through the decision backbone. In addition, depending on the instruction, it may also be directly fed into the Vision or Action block. In the LUROX D Implementation we have four specific clusters, Request, Intent, Objective and Specification. The LUROX D implementation is quite basic compared to the full VMCA architecture, but the general goal is to break down a sentence/command into a pure instruction set for the system to process. Some commands are direct, while others may need additional processing and this step in the algorithm helps get the picture.

The VMCA is designed for input modality, with this type of semantic cluster approach we can create a uniform instruction set that can allow different devices to effortlessly control the LUROX D prosthetic arm. This allows for asynchronous development of different algorithms, instead of requiring a full stack development, this modular approach allows for the VMCA to be designed seperately from a communication algorithm such as Speech Recognition or EEG Recogniton. 

## Decoupled Architecture & Direct Mapping
In this algorithm, the Action and Vision block operate independently from the Decision backbone. Instead of requiring total guidance from the Decision backbone, the Action and Vision block rather just get instructions and move forward. This allows for the introduction of directly mapped commands, where some commands simply just move to the action or vision block rather than needing total reasoning from the Decision backbone. 

# The Maix-Bit Microcontroller
Inside the Maix-Bit microcontroller is 8MB of RAM, 6MB allocated for the system memory and 2MB allocated for AI. The primary processor the Kendryte K210 which operates at 550MHz. The vision system utilizes the OV5640 camera connected to a 24 pin FFC wire. The vision block is operated on the Maix-Bit where it is comprised of a MobileNet0.75 backbone with a TinyYolOV2 Algorithm. In order to improve the Lock-On performance an Approximate Nearest Neighbor algorithm is used to prevent "fake-outs" and help the robot maintain tracking especially with vibrations/wobble. 

# The ESP32 Microcontroller
The ESP32-S3-N16-R8 is the primary microcontroller for this system, managing the motor control systems, action and decision blocks. The action block utilizes a gradient descent inverse kinematics systems coupled with a parametric bezier curve trajectory plotter. The goal for the action block is to simply move the arm, either completing gesture functions for finding the trajectory of where to go. 

