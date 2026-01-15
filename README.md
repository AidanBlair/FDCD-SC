# FDCD-SC
Fully Distributed Coordinate Descent Multi-Sensor Control

## Overview
This repository contains the code for the paper "Distributed Multi-Sensor Control for Multi-Target Tracking Using Adaptive Complementary Fusion for LMB Densities" by Aidan Blair et al.

## Requirements
. MATLAB 2023a
. Statistics and Machine Learning Toolbox
The code most likely will work with more recent versions of MATLAB, however it hasn't been tested.

## Usage
The main script is "main.m". Input prompts will let the user choose between which of the two scenarios and four multi-sensor control methods are used. In the paper, scenario 1 used 2 consensus iterations and scenario 2 used 3 consensus iterations.

"src/plot_6cams_figures_final.m" and "src/plot_8cams_figures_final.m" compare the results in "results" and produce some of the plots used in the paper.
