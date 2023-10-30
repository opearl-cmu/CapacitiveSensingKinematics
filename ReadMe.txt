CapacitiveSensingKinematics

Owen Douglas Pearl | Carnegie Mellon University

Micro Robotics Lab | Musculoskeletal Biomechanics Lab 

----------------------------------------------------------------------------------------------------------

Introduction and Proper Usage:

This repository contains code that illustrates how to use sparse capacitive sensing data to predict
a related joint angle. First a pretrained LSTM based neural network is used to predict normalized 
muscle lengths at the shank from capacitive sensing data for a test subject not included during
nueral network training. Next, the muscle lengths are unnormalized while performing inverse kinematics
to determine the ankle angle required to produce the unnormalized muscle lengths. 

By default, the script will run without requiring the MATLAB DeepLearning Toolbox, however the net variable
which contains the pretrained neural network can be adapted or packaged as a stand alone cpp file if the 
user has the DeepLearning Toolbox.

Be sure to add all files to the MATLAB path in order to run all code from the "Main" script. This
repository also requires installation of opensim matlab sripting libraries, which can be done using the
following instructions: https://simtk-confluence.stanford.edu:8443/display/OpenSim/Scripting+with+Matlab

Please see the attached License.txt file for rules on use and redistribution 
(Copyright (c) 2022 Owen Douglas Pearl via MIT License)

When using this work, we ask that you please cite the relevant literature to properly credit the authors:
Pearl, O., Rokhmanova, N., Dankovich, L., Faille, S., Bergbreiter, S., & Halilaj, E. (2022). 
Capacitive Sensing for Natural Environment Rehabilitation Monitoring [Preprint]. 
In Review. https://doi.org/10.21203/rs.3.rs-1902381/v1