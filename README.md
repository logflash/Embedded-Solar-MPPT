This repository contains my final project submission for ECE464 (Embedded Computing) at Princeton. The goal was to develop a working buck converter and sensor configuration for running the MPPT (Maximum Power Point Tracking) algorithm on a solar cell. By controlling the duty cycle (percentage of on-time) of the buck converter, the system changes the load on the solar cell, allowing it to operate at voltages where it generates more power. 

After designing and implementing the hardware, the aim of my project was to test two algorithms (Perturb-and-Observe and Gradient Descent) to find the strengths and weaknesses of each in real, non-simulated setting.