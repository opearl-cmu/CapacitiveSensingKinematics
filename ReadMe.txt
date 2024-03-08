CapacitiveSensingKinematics
----------------------------------------------------------------------------------------------------------

Overview: This codebase includes code to show how to use capacitive sensing data within two different 
wearable kinematics algorithms, CSInverseKinematics (folder) and CSOptimalControl (folder). It illustrates
how to load raw CS signals, process them, analyze them, learn from them, and predict kinematics with them
on their own or in combination with other wearables.

----------------------------------------------------------------------------------------------------------

Crediting and Licensing 

Please see the attached License.txt file for rules on use and redistribution 
(Copyright (c) 2024 Owen Douglas Pearl via MIT License)

When using this work, we ask that you please cite the relevant literature to properly credit the authors:
Pearl, O., Rokhmanova, N., Dankovich, L., Faille, S., Bergbreiter, S., & Halilaj, E. (2022). 
Capacitive Sensing for Natural Environment Rehabilitation Monitoring [Preprint]. 
In Review. https://doi.org/10.21203/rs.3.rs-1902381/v1

----------------------------------------------------------------------------------------------------------

CSInverseKinematics:

This repository contains code that illustrates how to use sparse capacitive sensing data to predict
a related joint angle. First a pretrained LSTM based neural network is used to predict normalized 
muscle lengths at the shank from capacitive sensing data for a test subject not included during
nueral network training. Next, the muscle lengths are unnormalized while performing inverse kinematics
to determine the ankle angle required to produce the unnormalized muscle lengths. 

By default, the script will run without requiring the MATLAB DeepLearning Toolbox, however the net variable
which contains the pretrained neural network can be adapted or packaged as a stand alone cpp file if the 
user has the DeepLearning Toolbox.

This code should run quite fast (within 10 seconds).

Be sure to add all files to the MATLAB path in order to run all code from the "Main" script. This
repository also requires installation of opensim matlab sripting libraries, which can be done using the
following instructions: https://simtk-confluence.stanford.edu:8443/display/OpenSim/Scripting+with+Matlab

----------------------------------------------------------------------------------------------------------

CSOptimalControl:

This repository contains code that illustrates how to fuse fiber lengths predicted with capacitive sensing 
data and inertial sensing data to enable accurate wearable's based optimal control simulation. The
Main script is setup to run three types of simulations, (1) sensor-fusion filter state-estimation with 
inertial sensors only, (2) IMU+EMG fusion via optimal control, (3) and IMU+CS optimal fusion via optimal
control. The IMU+CS method uses a custom cost function loaded into OpenSim moco, so be sure to set all 
of the path variables properly. The custom cost function is provided both as a .dll file and as source
with cmake scripts for editing and recompiling as needed. 

This code base enables users to automatically run biomechanics data through the entire OpenSim 
pipeline, including scaling, inverse kinematics, inverse dynamics residual reduction algorithm, 
static optimization, virtual IMU placement and calibration, IMU inverse kinematics, and a variety of 
potential MOCO simulations (torque driven, muscle driven, with and without contact forces, etc.)

This code also shows how to process CS data and compare it to fiber length measures from marker based 
motion capture, and then train predictive models from CS data to fiber lengths for predicting fiber 
lengths and tracking with optimal control (or inverse kinematics in the other folder's code).

Warning: Many operations in this code base will run for a few hours, even with powerful multi-core
CPUs. Optimal control tracking of multiple fiber lengths and joint angles is a computational 
expensive simulation. (If it is too expensive, try changing the number of knot points and simulating for
shorter time intervals).

Be sure to add all files to the MATLAB path in order to run all code from the "Main" script. This
repository also requires installation of opensim matlab sripting libraries, which can be done using the
following instructions: https://simtk-confluence.stanford.edu:8443/display/OpenSim/Scripting+with+Matlab

The simulations take only the starter data from the "InputData" folder (need to be copied to the 
"OutputData" folder) and populates the "OutputData" folder with all processed results and figures.

If you wish to edit the plugin source files directly, please reference the available tutorials on plugin 
development: https://simtk-confluence.stanford.edu:8443/display/OpenSim/Developer%27s+Guide