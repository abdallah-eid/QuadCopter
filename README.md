Overview
This project provides a comprehensive drone simulation using MATLAB's Simscape and Simulink. The simulation models the physical dynamics of a drone, including its wings, body, and control systems, allowing for detailed analysis and testing of drone behavior under various conditions.

Files and Structure
To run the simulation, you need to download the following files:

Wing Files: Contains the aerodynamic properties of the drone's wings.
Body Files: Defines the structural characteristics of the drone's body.
Assem1 Files: Includes the assembly data integrating the wings and body into a complete drone model.

Simulation Components
1. Physical Modeling with Simscape
The simulation uses Simscape to model the physical environment and dynamics of the drone. This includes:

-Structural properties of the body
-Integration of all components in the Assem1 files

2. Thrust and Motor Torques
Thrust and motor torques are added to simulate the drone's propulsion and maneuvering capabilities:

-Thrust Functions: Simulate the thrust generated by each propeller.
-Motor Torque Functions: Simulate the torques for roll, pitch, and yaw movements, essential for drone stability and control.

3. Control System with PID Tuning
The control system is implemented using PID controllers in Simulink to manage the drone's flight:

-PID Controllers: Used to maintain desired flight characteristics by adjusting motor speeds based on feedback.
-Simulink Integration: The PID controllers are integrated into the Simulink model to interact with the drone’s dynamics and provide real-time control.

Getting Started Prerequisites:
-MATLAB with Simulink and Simscape installed.
-Basic understanding of control systems and physical modeling.
