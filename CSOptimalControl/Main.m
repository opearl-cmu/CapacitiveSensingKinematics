%% CSOptimalControl
% Author: Owen Pearl

% This script shows an example of use the OpenSim API workflow and MOCO to
% compare various optimal control techniques with example wearables data.
% It also shows how to create and load a custom MOCO plugin for tracking
% the CS-predicted fiber lengths along with IMU-predicted orientations.
% It also shows how to load raw CS data, process it, analyze it, learn from
% it, make inferences, and track inferences with optimal control simulation

% When using this work, please cite the following literature:
% Pearl, O., Rokhmanova, N., Dankovich, L., Faille, S., Bergbreiter, S., & Halilaj, E. (2022). 
% Capacitive Sensing for Natural Environment Rehabilitation Monitoring [Preprint]. 
% In Review. https://doi.org/10.21203/rs.3.rs-1902381/v1

%% Main 
% 0. Setup Workflow and Select Options
clear all; close all; clc

% Import libraries
import org.opensim.modeling.*;
myMatlabLog = JavaLogSink();
Logger.addSink(myMatlabLog);

% Pathing
currentDirectory = pwd;
setupDirectory = fullfile(currentDirectory,'Setup');
dataDirectory = fullfile(currentDirectory,'OutputData');

% Choose Simulation
% Settings(1) Contact Model (1 for yes; 0 for no), 
% Settings(2) IMU Tracking (2 for tracking OpenSense Kins, 1 for tracking accels and gyro; 0 for marker-based kinematics);
% Setting(3) EMG tracking (1 for yes, 0 for no); 
% Settings(4) CS Tracking (2 for yes (analyze CS and build models), 1 for yes (uses pre-printed models), 0 for no); 
% Settings(5) Experimental Wearables Data or Synthetic (1 for yes, 0 for no)

% settings = [1, 0, 0, 0, 0]; % Generate initial guess with kins tracking (also already pre-generated)
settings = [1, 2, 0, 1, 1]; % Compare IMU-Only and IMU+CS fusion (see RMSEOpenSenseStruct and RMSEMocoStruct for comparison; uses prepracked CS-driven FL predictions)
% settings = [1, 2, 0, 2, 1]; % Compare IMU-Only and IMU+CS fusion (this one does the CS data unpacking and analysis and building of CS predictive models of FLs; requires deep learning toolbox)

% Explore other options by following through the logic and providing
% proper experimental or synthetic data inputs as needed, etc.

% OpenSim Model/Settings Inputs
if settings(1) == 1
        osimModelFile = 'OpenSimMOCOModelWithContactGeometry.osim';
elseif settings(1) == 0
    osimModelFile = 'OpenSimMOCOModel.osim';
end
   
markerModelFile = 'MarkerSet.xml';
scaleModelFile = 'ScaleSettings.xml'; startScale = 0; endScale = 5; % second marks
ikModelFile = 'IKSettings.xml'; startIK = 20; endIK = 65; % second marks
imuCalculatorFile = 'AnalyzeToolSettings_IMUDataReporter.xml';

subjectCalibrationFile = 'OpenSim_Sub02_Subject_Calibration.mat';
load(fullfile(dataDirectory,subjectCalibrationFile));
scalingTrialFile = 'OpenSim_static_markers.trc';
markerTrialFile = 'OpenSim_Sub02_CSBaseline_markers.trc';
grfDataFile = 'OpenSim_Sub02_CSBaseline_GRF.mot';
ExternalLoadsFile = 'OpenSim_Sub02_CSBaseline_ExternalLoads.xml';

% Experimental Files if using exp. data
if settings(5) == 1
    accelFile = 'OpenSim_Sub02_CSBaseline_Accel.trc';
    gyroFile = 'OpenSim_Sub02_CSBaseline_Gyro.trc';
    magnetFile = 'OpenSim_Sub02_CSBaseline_Magnet.trc';
    orientationFile = 'OpenSim_Sub02_CSBaseline_Orientations.sto';
    lcsFile = 'OpenSim_Sub02_CSBaseline_LCS.mot';
    rcsFile = 'OpenSim_Sub02_CSBaseline_RCS.mot';
end

% Add OpenSim Geometry to path to avoid warnings
geomPath = 'Setup\Geometry';
ModelVisualizer.addDirToGeometrySearchPaths(geomPath);

% Start saving the MATLAB outputs if you would like another copy
% diary(fullfile(dataDirectory,'commandwindow.txt'))

%% 1. Model Scaling
import org.opensim.modeling.*;

% Create names specific to scaling tool
opensimModelFilename = fullfile(setupDirectory,osimModelFile);
markerSetFilename = fullfile(setupDirectory,markerModelFile);
scaleSettingsFilename = fullfile(setupDirectory,scaleModelFile);
trialFilename = fullfile(dataDirectory,markerTrialFile);

% Create tool and set properties
scaleTool = ScaleTool(scaleSettingsFilename);
scaleTool.setPathToSubject('');
[~, scaleFilename, scaleExt] = fileparts(scaleSettingsFilename);

% Intermediate/Output Pathing
[~, opensimModelName, osimModelExt] = fileparts(opensimModelFilename);
outputInitModelFilename = fullfile(dataDirectory, [opensimModelName, '_init', osimModelExt]);
outputScaledModelFilename = fullfile(dataDirectory, [opensimModelName, '_scaled', osimModelExt]);
outputPlacedModelFilename = fullfile(dataDirectory, [opensimModelName, '_placed', osimModelExt]);

% Add IMUs at this point so they move downstream through the entire process
if settings(5) == 0

    if settings(2) == 1 || settings(2) == 2
        % Load the model
        baseModel = Model(opensimModelFilename);
        baseModel.setName(opensimModelName);
        markerSet = MarkerSet(markerSetFilename);
        baseModel.updateMarkerSet(markerSet);

        % Add IMUs to model if using synthetic IMU data
        addIMUFrame(baseModel,'torso', Vec3(0.078485655249427938, 0.42727180873301879, -0.0015073311787994498), Vec3(0, 0, 0.52359999999999995));
        addIMUFrame(baseModel, 'pelvis', Vec3(-0.24262253112242196, 0.031145714558142319, 0), Vec3(0, 0, 0));
        addIMUFrame(baseModel, 'femur_r', Vec3(-0.0094999999999999998, -0.21786619166774404, 0.070586648118892703), Vec3(0, -1.5708, 0));
        addIMUFrame(baseModel, 'tibia_r', Vec3(0.00042809626861462923, -0.31757661857261293, 0.068802474692082713), Vec3(0, -1.5708, 0));
        addIMUFrame(baseModel, 'toes_r', Vec3(-0.03770960005819491, 0.053106709183529832, 0.0068471829146961672), Vec3(0, 0, 0.78539999999999999));
        addIMUFrame(baseModel, 'femur_l', Vec3(-0.0094999999999999998, -0.21765259736218745, -0.070586648118892703), Vec3(0, 1.5708, 0));
        addIMUFrame(baseModel, 'tibia_l', Vec3(-0.0039965320610509227, -0.31055002154075884, -0.065775795305983709), Vec3(0, 1.5708, 0));
        addIMUFrame(baseModel, 'toes_l', Vec3(-0.040452877861573776, 0.039258010849339511, -0.010276848567916523), Vec3(0, 0, 0.78539999999999999));

        % Add IMU components to the model using the PhysicalOffsetFrames
        % we just added to the model. We'll use the helper function addModelIMUs()
        % included with OpenSenseUtilities
        imuFramePaths = StdVectorString();
        imuFramePaths.add('/bodyset/torso/torso_imu_offset');
        imuFramePaths.add('/bodyset/pelvis/pelvis_imu_offset');
        imuFramePaths.add('/bodyset/femur_r/femur_r_imu_offset');
        imuFramePaths.add('/bodyset/tibia_r/tibia_r_imu_offset');
        imuFramePaths.add('/bodyset/toes_r/toes_r_imu_offset');
        imuFramePaths.add('/bodyset/femur_l/femur_l_imu_offset');
        imuFramePaths.add('/bodyset/tibia_l/tibia_l_imu_offset');
        imuFramePaths.add('/bodyset/toes_l/toes_l_imu_offset');
        OpenSenseUtilities().addModelIMUs(baseModel, imuFramePaths);
        baseModel.initSystem();

        % Print the new model for referencing in the scale tool and beyond
        baseModel.print(outputInitModelFilename);

        imuColumnsPaths = imuFramePaths;

    elseif settings(2) == 0
        % Load the model add markerset - resave
        baseModel = Model(opensimModelFilename);
        baseModel.setName(opensimModelName);
        markerSet = MarkerSet(markerSetFilename);
        baseModel.updateMarkerSet(markerSet);
        baseModel.initSystem();
        baseModel.print(outputInitModelFilename);
    end

elseif settings(5) == 1
    baseModel = Model(opensimModelFilename);
    baseModel.setName(opensimModelName);
    markerSet = MarkerSet(markerSetFilename);
    baseModel.updateMarkerSet(markerSet);
    addIMUFrame(baseModel, 'torso', Vec3(0.05, 0.43, 0), Vec3(0, 1.5708, 0));
    addIMUFrame(baseModel, 'toes_r', Vec3(-0.0426463, 0.0531067, 0.0150312), Vec3(-1.308, 0.4, -0.2));
    addIMUFrame(baseModel, 'toes_l', Vec3(-0.0457487, 0.051258, -0.0170539), Vec3(-1.908, 0.4, 0.3));
    imuFramePaths = StdVectorString();
    imuFramePaths.add('/bodyset/torso/torso_imu_offset');
    imuFramePaths.add('/bodyset/toes_r/toes_r_imu_offset');
    imuFramePaths.add('/bodyset/toes_l/toes_l_imu_offset');
    OpenSenseUtilities().addModelIMUs(baseModel, imuFramePaths);
    baseModel.initSystem();
    baseModel.print(outputInitModelFilename);
end

% Set time range
timeRange = ArrayDouble(0, 2);
timeRange.set(0, startScale);
timeRange.set(1, endScale);

% Set subject values
scaleTool.setSubjectMass(subjectInfoStruct.mass);
scaleTool.setSubjectHeight(subjectInfoStruct.height);
scaleTool.setSubjectAge(subjectInfoStruct.age);

% Set Generic Model/Marker File Information
modelMaker = scaleTool.getGenericModelMaker();
modelMaker.setModelFileName(outputInitModelFilename);
modelMaker.setMarkerSetFileName(markerSetFilename);

% Scaling
scaler = scaleTool.getModelScaler();
scaler.setMarkerFileName(trialFilename);
scaler.setTimeRange(timeRange);
scaler.setPreserveMassDist(1);
scaler.setOutputModelFileName(outputScaledModelFilename);
scaler.setPrintResultFiles(1);

% Adjust markers
placer = scaleTool.getMarkerPlacer();
placer.setStaticPoseFileName(trialFilename);
placer.setTimeRange(timeRange);
placer.setOutputModelFileName(outputPlacedModelFilename);
placer.setMoveModelMarkers(1);
placer.setMaxMarkerMovement(0.001)

% Print Settings File name
outputScaleToolFilename = fullfile(dataDirectory, [scaleFilename, scaleExt]);
scaleReload = ScaleTool(scaleTool);
scaleReload.print(outputScaleToolFilename);
scaleReload.run(); % Loads generic model, scales, and places markers

% Add IMUs for after scaling here using the marker locations to calibrate
% when using the experimental IMU data
if (settings(2) == 1 || settings(2) == 2) && settings(5) == 1
   
    % Load the scaled model and get the default pose
    baseModel = Model(outputScaledModelFilename);
    staticState = baseModel.initSystem();
% 
    % Calculate position of IMU in each local body frame using clusters to
    % ensure best chance of good tracking
    pelvis = baseModel.updBodySet().get('pelvis');
    LPS1 = baseModel.getMarkerSet().get('LPS1');
    RPS2 = baseModel.getMarkerSet().get('RPS2');
    LPS1_Position = LPS1.findLocationInFrame(staticState, pelvis).getAsMat';
    RPS2_Position = RPS2.findLocationInFrame(staticState, pelvis).getAsMat';
    sacrumIMUPosition = (LPS1_Position - RPS2_Position)/2 + RPS2_Position;

    femur_r = baseModel.updBodySet().get('femur_r');
    RTH1 = baseModel.getMarkerSet().get('RTH1');
    RTH3 = baseModel.getMarkerSet().get('RTH3');
    RTH1_Position = RTH1.findLocationInFrame(staticState, femur_r).getAsMat';
    RTH3_Position = RTH3.findLocationInFrame(staticState, femur_r).getAsMat';
    rthighIMUPosition = (RTH1_Position - RTH3_Position)/2 + RTH3_Position;

    femur_l = baseModel.updBodySet().get('femur_l');
    LTH1 = baseModel.getMarkerSet().get('LTH1');
    LTH3 = baseModel.getMarkerSet().get('LTH3');
    LTH1_Position = LTH1.findLocationInFrame(staticState, femur_l).getAsMat';
    LTH3_Position = LTH3.findLocationInFrame(staticState, femur_l).getAsMat';
    lthighIMUPosition = (LTH1_Position - LTH3_Position)/2 + LTH3_Position;

    tibia_r = baseModel.updBodySet().get('tibia_r');
    RSH1 = baseModel.getMarkerSet().get('RSH1');
    RSH3 = baseModel.getMarkerSet().get('RSH3');
    RSH1_Position = RSH1.findLocationInFrame(staticState, tibia_r).getAsMat';
    RSH3_Position = RSH3.findLocationInFrame(staticState, tibia_r).getAsMat';
    rshankIMUPosition = (RSH1_Position - RSH3_Position)/2 + RSH3_Position;

    tibia_l = baseModel.updBodySet().get('tibia_l');
    LSH1 = baseModel.getMarkerSet().get('LSH1');
    LSH3 = baseModel.getMarkerSet().get('LSH3');
    LSH1_Position = LSH1.findLocationInFrame(staticState, tibia_l).getAsMat';
    LSH3_Position = LSH3.findLocationInFrame(staticState, tibia_l).getAsMat';
    lshankIMUPosition = (LSH1_Position - LSH3_Position)/2 + LSH3_Position;

    % Put the new frames and IMUs on the model
    addIMUFrame(baseModel, 'pelvis', Vec3(sacrumIMUPosition(1), sacrumIMUPosition(2), sacrumIMUPosition(3)), Vec3(-1.58359, -1.57079, 0));
    addIMUFrame(baseModel, 'femur_r', Vec3(rthighIMUPosition(1), rthighIMUPosition(2), rthighIMUPosition(3)), Vec3(0, 0, -1.5708));
    addIMUFrame(baseModel, 'femur_l', Vec3(lthighIMUPosition(1), lthighIMUPosition(2), lthighIMUPosition(3)), Vec3(0, -3.14159, -1.57079));
    addIMUFrame(baseModel, 'tibia_r', Vec3(rshankIMUPosition(1), rshankIMUPosition(2), rshankIMUPosition(3)), Vec3(0, 0, -1.5708));
    addIMUFrame(baseModel, 'tibia_l', Vec3(lshankIMUPosition(1), lshankIMUPosition(2), lshankIMUPosition(3)), Vec3(0, -3.14159, -1.57079));
    
    imuFramePaths = StdVectorString();
    imuFramePaths.add('/bodyset/pelvis/pelvis_imu_offset');
    imuFramePaths.add('/bodyset/femur_r/femur_r_imu_offset');
    imuFramePaths.add('/bodyset/femur_l/femur_l_imu_offset');
    imuFramePaths.add('/bodyset/tibia_r/tibia_r_imu_offset');
    imuFramePaths.add('/bodyset/tibia_l/tibia_l_imu_offset');

    OpenSenseUtilities().addModelIMUs(baseModel, imuFramePaths);
    baseModel.initSystem();

    % Print the new model as the updated scaled model
    baseModel.print(outputPlacedModelFilename);

    % Store column labels for below
    imuColumnsPaths = StdVectorString();
    imuColumnsPaths.add('/bodyset/torso/torso_imu_offset');
    imuColumnsPaths.add('/bodyset/toes_r/toes_r_imu_offset');
    imuColumnsPaths.add('/bodyset/toes_l/toes_l_imu_offset');
    imuColumnsPaths.add('/bodyset/pelvis/pelvis_imu_offset');
    imuColumnsPaths.add('/bodyset/femur_r/femur_r_imu_offset');
    imuColumnsPaths.add('/bodyset/femur_l/femur_l_imu_offset');
    imuColumnsPaths.add('/bodyset/tibia_r/tibia_r_imu_offset');
    imuColumnsPaths.add('/bodyset/tibia_l/tibia_l_imu_offset');
end

%% 2a. Inverse Kinematics
import org.opensim.modeling.*;
% Create names specific to IK Tool
ikSettingsFilename = fullfile(setupDirectory,ikModelFile);
iktrialFilename = fullfile(dataDirectory,markerTrialFile);

% Load Model and Initialize
baseModel = Model(outputPlacedModelFilename);
baseModel.initSystem();

% Load template file into tool - Easiest way to load marker tasks
ikTool = InverseKinematicsTool(ikSettingsFilename);
[~, ikFilename, ikExt] = fileparts(ikSettingsFilename);

% Set Start and End Times
ikTool.setStartTime(startIK);
ikTool.setEndTime(endIK);

% Set the model and marker files for the tool
ikTool.setModel(baseModel);
ikTool.setMarkerDataFileName(iktrialFilename);

% Set the output .mot file and print the tool settings
outputIKResultsFilename = fullfile(dataDirectory, ['IKoutput_', num2str(startIK), 'to', num2str(endIK), 'secs.mot']);
ikTool.setOutputMotionFileName(outputIKResultsFilename);

% Run the tool
ikTool.run();

% Print Settings File name
outputIKToolFilename = fullfile(dataDirectory, [ikFilename, ikExt]);
ikTool.print(outputIKToolFilename);

%% 2b. Post IK ErrorCheck and Filtering (convert treadmill to overground for overground with contact model)
% IK Error Checker Parameters/Options
doIKErrorCheck = 1;
IKErrorParameters = struct;
% Accuracy level for IK Solver
IKErrorParameters.accuracy = 1E-05;
% Add constraint weight from the ikSettingsFile here (def. = inf)
IKErrorParameters.constraintWeight = inf;
% Enter debug mode? bool for on or off (def. = false)
IKErrorParameters.doDebugPrint = true;
% Maximum error theshold for each marker
IKErrorParameters.maxAllowedError = 0.8;   % meters
IKErrorParameters.maxAllowedRMSE = 0.6;    % meters

% Savename for Error Suffix
IKErrorStructSuffix = '_IKErrorData.mat';

% IK parameters for Post Filtering
% Filtfilt - Total Order 2x listed.
IKPostFilterParameters.filterFreq = 10;
IKPostFilterParameters.filterOrder = 2;

% Run IK Error Checker
if(doIKErrorCheck)
    ikErrorDataStruct = CheckErrorOpenSimIK(outputPlacedModelFilename, fullfile(dataDirectory, markerTrialFile), outputIKToolFilename, IKErrorParameters, 30, 35);
    if(~isempty(ikErrorDataStruct))
        outputWorkspaceFilename = fullfile(dataDirectory, ['IKoutput_', num2str(startIK), 'to', num2str(endIK), 'secs', IKErrorStructSuffix]);
        ParforSave_SingleStructure(outputWorkspaceFilename, ikErrorDataStruct);
    end
end

numJoints = baseModel.getNumJoints;
jointSet = baseModel.getJointSet();
mocoCoordLabels = {};
counter = 0;
for i = 0:numJoints-1
    currJoint = jointSet.get(i);
    numCoords = currJoint.numCoordinates;
    for j = 0:numCoords-1
        counter = counter + 1;
        %         disp( ['/jointset/', char(currJoint), '/', char(currJoint.get_coordinates(j)), '/value'] )
        mocoCoordLabels{counter} = ['/jointset/', char(currJoint), '/', char(currJoint.get_coordinates(j)), '/value'];
    end
end

% Filter the IK data and Create New File
% Load Data
ikRawData = ReadOpenSimData(outputIKResultsFilename);
dataRate = 1/(ikRawData.time(2)-ikRawData.time(1));

% Filter IK Data
ratio = IKPostFilterParameters.filterFreq/(dataRate/2);
[b,a] = butter(IKPostFilterParameters.filterOrder, ratio);
ikRawData.data = filtfilt(b, a, ikRawData.data);
ikRawData.dataRate = dataRate;
ikRawData.labels = mocoCoordLabels;

% Toggle External Forces
% At this point, if using a contact model, we convert this data to
% overground walking using the known walking speed
if settings(1) == 1 % If using contact model, make pelvis_tx adjustment
    treadSpeed = subjectInfoStruct.gaitspeed; % m/s
    time = ikRawData.time - min(ikRawData.time);
    overgroundAdjust = treadSpeed * time;
    ikRawData.data(:, find(strcmp(ikRawData.labels, '/jointset/ground_pelvis/pelvis_tx/value'))) = ikRawData.data(:, find(strcmp(ikRawData.labels, '/jointset/ground_pelvis/pelvis_tx/value'))) + overgroundAdjust;
end

% Resave for tracking
outputIKFilteredFilename = fullfile(dataDirectory, ['IKoutput_', num2str(startIK), 'to', num2str(endIK), 'secs_', num2str(IKPostFilterParameters.filterFreq), 'hz.mot']);
ConvertStructDataToOpenSimMotionStorage(ikRawData, outputIKFilteredFilename) % Overwrites existing file with filtered data .mot

%% 3a. Run RRA (Optional)
doRRA = 0;
if(doRRA)
    import org.opensim.modeling.*;
    % START RRA
    % Create names specific to RRA Tool
    rraSettingsFilename = fullfile(setupDirectory,'RRASettings.xml');
    
    % Build tool, edit and print settings file, reload, and run
    rraToolsetup = RRATool(rraSettingsFilename,0); 

    % Set the name of the settings file: This actually changes the prefix
    % of the output files from RRA. This must be set as the trial name to
    % be unique for each one
    rraToolsetup.setName(extractBefore(markerTrialFile, '_markers'));
     
    % Load model file 
    rraToolsetup.setModelFilename(outputPlacedModelFilename);

    % Set Start and End Times
    rraToolsetup.setStartTime(startIK);
    rraToolsetup.setFinalTime(endIK);

    % Set the kinematics file .mot
    rraToolsetup.setDesiredKinematicsFileName(outputIKResultsFilename);
    
    % Set the outputs adjusted model.osim name
    outputAdjustedModelFileName = fullfile(dataDirectory, [opensimModelName, '_adjusted', osimModelExt]);
    rraToolsetup.setOutputModelFileName(outputAdjustedModelFileName);
  
    % Set the external loads.xml from exp. data
    rraToolsetup.setExternalLoadsFileName(fullfile(dataDirectory, ExternalLoadsFile));
    
    % Set output directory
    rraToolsetup.setResultsDir(dataDirectory);

    % Set ForceSetFiles
    force_set = ArrayStr;
    % Loads actuator file into first indice of force_set list of files
    force_set.setitem(0,fullfile(setupDirectory,'RRA_Actuators.xml'));
    rraToolsetup.setForceSetFiles(force_set);
    
    % Set task list
    rraToolsetup.setTaskSetFileName(fullfile(setupDirectory,'RRA_Tasks.xml'));
    
    % Print the xml for the setup of RRA
    outputRRAToolFilename = fullfile(dataDirectory, [extractBefore(markerTrialFile, '_markers'), '_RRASettings.xml']);
    rraToolsetup.print(outputRRAToolFilename);
    
    % Reload and run
    rraTool = RRATool(outputRRAToolFilename);
    rraTool.run();
end

%% 3b. Run Static Opt (Optional)
doStaticOptimization = 0;
if(doStaticOptimization)
    import org.opensim.modeling.*;
    % START STATIC OPTIMIZATION
    % Create names specific to Static Opt Tool
    analyzeSettingsFilename = fullfile(setupDirectory,'AnalyzeToolSettings_StaticOptimization.xml');

    % Build tool, edit and print settings file, reload, and run
    analyzeToolsetup = AnalyzeTool(analyzeSettingsFilename, 0); % Loads default setting file

    % Set the name of the settings file: This actually changes the prefix
    % of the output files from StaticOpt. This must be set as the trial name to
    % be unique for each one
    analyzeToolsetup.setName(extractBefore(markerTrialFile, '_markers'));
    
    % Load model file 
   analyzeToolsetup.setModelFilename(outputAdjustedModelFileName);

    % Set Start and End Times
    analyzeToolsetup.setStartTime(startIK);
    analyzeToolsetup.setFinalTime(endIK);

    % Set the kinematics file .mot
    inputRRAq = fullfile(dataDirectory, [extractBefore(markerTrialFile, '_markers'), '_Kinematics_q.sto']);
    analyzeToolsetup.setCoordinatesFileName(inputRRAq);
    
    % Set output directory
    analyzeToolsetup.setResultsDir(dataDirectory);
    
    % Set the external loads.xml from exp. data
    analyzeToolsetup.setExternalLoadsFileName(fullfile(dataDirectory, ExternalLoadsFile));
    
    % Set ForceSetFiles
    force_set = ArrayStr;
    % Loads actuator file into first indice of force_set list of files
    force_set.setitem(0,fullfile(setupDirectory,'RRA_Actuators.xml'));
    analyzeToolsetup.setForceSetFiles(force_set);
   
    % Print the xml for the setup of RRA (can load into GUI easily!!!)
    outputAnalyzeToolFilename = fullfile(dataDirectory, [extractBefore(markerTrialFile, '_markers'), '_StaticOptSettings.xml']);
    analyzeToolsetup.print(outputAnalyzeToolFilename);
    
    % Set lowpass frequency on coordinate data
    analyzeToolsetup.setLowpassCutoffFrequency(IKPostFilterParameters.filterFreq)
    
    % Reload and run
    analyzeTool = AnalyzeTool(outputAnalyzeToolFilename);
    analyzeTool.run();
end

%% 4a Calculate IMU Profiles From Filtered Kinematics (for virtual data)
if (settings(2) == 1 || settings(2) == 2)
    % Parameters for Post Filtering Filtfilt - Total Order is 2x listed
    doIMUFilt = 1;
    AccelPostFilterParameters.filterFreq = 10;
    AccelPostFilterParameters.filterOrder = 2;
    GyroPostFilterParameters.filterFreq = 10;
    GyroPostFilterParameters.filterOrder = 2;

    import org.opensim.modeling.*;
    imuSettingsFilename = fullfile(setupDirectory, imuCalculatorFile);

    % Build tool, edit and print settings file, reload, and run
    analyzeToolsetup = AnalyzeTool(imuSettingsFilename, 0); % Loads default setting file

    % Set the name of the settings file: This actually changes the prefix
    % of the output files. This must be set as the trial name to
    % be unique for each one
    analyzeToolsetup.setName('VirtualIMUProfiles');

    % Load model file
    analyzeToolsetup.setModelFilename(outputPlacedModelFilename);

    % Set Start and End Times
    analyzeToolsetup.setStartTime(startIK);
    analyzeToolsetup.setFinalTime(endIK);

    % Set the kinematics file .mot
    analyzeToolsetup.setCoordinatesFileName(outputIKFilteredFilename);

    % Set output directory
    analyzeToolsetup.setResultsDir(dataDirectory);

    % Print the xml for the setup of RRA (can load into GUI easily)
    analyzeToolsetup.print(fullfile(dataDirectory, imuCalculatorFile));

    % Reload with custom built settings and run with API
    analyzeTool = AnalyzeTool(fullfile(dataDirectory, imuCalculatorFile));
    analyzeTool.run();

    % Convert to TimeSeriesTableVec3 (and filter) for tracking
    % Toggle on or off using the IK driven IMU profiles vs exp. IMUs
    if settings(5) == 0
        accelTimeSeriesTableVec3 = TimeSeriesTableVec3(fullfile(dataDirectory, 'VirtualIMUProfiles_linear_accelerations.sto'));
        gyroTimeSeriesTableVec3 = TimeSeriesTableVec3(fullfile(dataDirectory, 'VirtualIMUProfiles_angular_velocity.sto'));
        imuFramePaths = imuColumnsPaths;
    elseif settings(5) == 1
        accelTimeSeriesTableVec3 = TimeSeriesTableVec3(fullfile(dataDirectory, accelFile));
        gyroTimeSeriesTableVec3 = TimeSeriesTableVec3(fullfile(dataDirectory, gyroFile));
        experimentalColumns = accelTimeSeriesTableVec3.getColumnLabels();
        imuFramePaths = StdVectorString();
        for iterate = 1:experimentalColumns.capacity
            for iterateagain = 1:experimentalColumns.capacity
                % Note: If one of the following lines causes an error, switch to
                % the other line, sometimes depending on the loading of the
                % libraries you need to use 'toCharArray()' and sometimes
                % you don't

                if contains(imuColumnsPaths.get(iterateagain-1).toCharArray()', extractBefore(experimentalColumns.get(iterate-1).toCharArray()','_imu'))
                % if contains(imuColumnsPaths.get(iterateagain-1), extractBefore(experimentalColumns.get(iterate-1),'_imu'))
                    imuFramePaths.add(imuColumnsPaths.get(iterateagain-1));
                end
            end
        end
        assert(imuFramePaths.capacity == imuColumnsPaths.capacity)
    end

    if doIMUFilt == 1
        accelStruct = osimTableToStruct(accelTimeSeriesTableVec3);
        gyroStruct = osimTableToStruct(gyroTimeSeriesTableVec3);

        time = accelStruct.time;
        samplingFrequency = 1/(time(2)-time(1));
        accelRatio = AccelPostFilterParameters.filterFreq/(samplingFrequency/2);
        [bAccel,aAccel] = butter(AccelPostFilterParameters.filterOrder, accelRatio);
        gyroRatio = GyroPostFilterParameters.filterFreq/(samplingFrequency/2);
        [bGyro,aGyro] = butter(GyroPostFilterParameters.filterOrder, gyroRatio);

        fields = fieldnames(accelStruct);
        fields(strcmp(fields,'time')) = [];
        for i = 1:length(fields)
            accelStruct.(fields{i}) = filtfilt(bAccel, aAccel, accelStruct.(fields{i}));
            gyroStruct.(fields{i}) = filtfilt(bGyro, aGyro, gyroStruct.(fields{i}));
        end

        % Print
        accelTimeSeriesTableVec3 = osimTableFromStruct(accelStruct);
        gyroTimeSeriesTableVec3 = osimTableFromStruct(gyroStruct);
    end

    % Change the column labels to match IMU full paths, not just frame
    accelTimeSeriesTableVec3.setColumnLabels(imuFramePaths)
    gyroTimeSeriesTableVec3.setColumnLabels(imuFramePaths)

    % Write filtered final version to see what is actually tracked
    accelTimeSeriesTableVec3.addTableMetaDataString('DataRate',string(samplingFrequency))
    accelTimeSeriesTableVec3.addTableMetaDataString('Units','m')
    gyroTimeSeriesTableVec3.addTableMetaDataString('DataRate',string(samplingFrequency))
    gyroTimeSeriesTableVec3.addTableMetaDataString('Units','m')
    TRCFileAdapter().write(accelTimeSeriesTableVec3, fullfile(dataDirectory, 'FilteredIMUAccels.trc'))
    TRCFileAdapter().write(gyroTimeSeriesTableVec3, fullfile(dataDirectory, 'FilteredlIMUGyros.trc'))
end

%% 4b Calculate Experimental Kinematics from XSens IMUs using OpenSense
% This the IMU Only simulation in the cited study
if(settings(2) == 2)
    import org.opensim.modeling.*;

    % Setup and run the IMUPlacer tool
    sensor_to_opensim_rotations = Vec3(-pi/2, 0, 0); % From NED to X-Forward, Y-Up, Z-Right of OSIM

    % First frame of model must match default pose of model
    myIMUPlacer = IMUPlacer();
    myIMUPlacer.set_model_file(outputPlacedModelFilename);
    myIMUPlacer.set_orientation_file_for_calibration(fullfile(dataDirectory, orientationFile))
    myIMUPlacer.set_sensor_to_opensim_rotations(sensor_to_opensim_rotations) % From NED to X-Forward, Y-Up, Z-Right of OSIM
    myIMUPlacer.set_base_imu_label('pelvis_imu_offset_imu')
    myIMUPlacer.set_base_heading_axis('-z')
    myIMUPlacer.run(false); % Can visualize with true but it stops the code

    % Write the calibrated model to file
    outputIMUPlacedModelFilename = fullfile(dataDirectory, [opensimModelName, '_IMUCalibrated', osimModelExt]);
    myIMUPlacer.getCalibratedModel().print(outputIMUPlacedModelFilename);

    % Calculate Kinematics using OpenSense (IMUInverseKinematicsTool)
    % Create and fill settings file
    imuIK = IMUInverseKinematicsTool();
    imuIK.set_model_file(outputIMUPlacedModelFilename);
    imuIK.set_results_directory(dataDirectory);
    imuIK.set_orientations_file(fullfile(dataDirectory, orientationFile));
    imuIK.set_sensor_to_opensim_rotations(sensor_to_opensim_rotations)
    imuIK.setStartTime(startIK);
    imuIK.setEndTime(endIK);
    imuIK.print(fullfile(dataDirectory,'Setup_IMUIK.xml'));

    % Reload and run
    runIMUIK = IMUInverseKinematicsTool(fullfile(dataDirectory,'Setup_IMUIK.xml'));
    runIMUIK.run(false); % bool here determines whether to visualize

    %% Check IMU-Only Results Against Ground Truth Kinematics (Mocap)
    resultsDirectory = dataDirectory;
    OpenSenseResults = calculateOpenSenseIKRMSEsfromReferenceKinematics(outputIKFilteredFilename, fullfile(dataDirectory, ['ik_', extractBefore(orientationFile,'.sto'), '.mot']), startIK, endIK);
    
    % Plot them
    grabInds = [10 13 15];
    figure
    X = 1:length(grabInds);
    Y = OpenSenseResults.KINrmse(grabInds)
    h = bar(X,Y);
    xticklabels({'Hip Flexion', 'Knee Flexion', 'Ankle Flexion'})
    set(gca, 'box', 'off')
    ylabel('RMSE (deg)')
    h.FaceColor = [0 0 .7];
    title('IMU IK RMSEs')

    save(fullfile(resultsDirectory, 'RMSEOpenSenseStruct.mat'), 'OpenSenseResults')
end

%% 5. Load CS Data, Calculate FLs from kinematics, Predict FLs from CS
% This section requires you to have already printed out an edited model
% file (_trajopt) with the model processer (see section 6) to work on.
% This should be the case because you need to run a first pass tracking
% just the ground truth kinematics anyways to generate a blank inital
% trajectory to initialize the optimization problem

if(settings(4) == 2 && settings(5) == 1)
    % Import libraries
    import org.opensim.modeling.*;
    
    % Create Trajopt model (must match how it is defined in trajopt
    % section, otherwise the tracking won't be tracking with the same model
    % convention)

    % Construct a ModelProcessor and set it on the tool. The default
    % muscles in the model are replaced with optimization-friendly
    % DeGrooteFregly2016Muscles, and adjustments are made to the default muscle
    % parameters.
    baseModel = Model(outputPlacedModelFilename);
    
    % Start editing model here
    modelProcessor = ModelProcessor(baseModel);
    modelProcessor.append(ModOpIgnoreTendonCompliance());
    
    % https://simtk.org/plugins/phpBB/viewtopicPhpbb.php?f=1815&t=12104&p=34251&start=0&view=
    % modelProcessor.append(ModOpTendonComplianceDynamicsModeDGF('implicit'));
    modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
    % Only valid for DeGrooteFregly2016Muscles.
    modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
    
    % https://simtk.org/plugins/phpBB/viewtopicPhpbb.php?f=1815&t=14291&p=41189&start=0&view=
    % modelProcessor.append(ModOpUseImplicitTendonComplianceDynamicsDGF());
    modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
    
    % Remove all the muscles in the model's ForceSet (torque drive)
    % modelProcessor.append(ModOpRemoveMuscles());

    % Add CoordinateActuators to the model degrees-of-freedom. By default
    % ignores if they already have an actuator...
    modelProcessor.append(ModOpAddReserves(1000));
    % modelProcessor.append(ModOpReplaceJointsWithWelds(jointNames));

    % Toggle on External Forces
    if settings(1) == 0 % If no contact model add the external loads to model
        externalLoadsFilename = fullfile(dataDirectory,ExternalLoadsFile);
        modelProcessor.append(ModOpAddExternalLoads(externalLoadsFilename));
    end
    modelobj = modelProcessor.process();
    trajOptModelfile = fullfile(dataDirectory, [opensimModelName, '_trajopt', osimModelExt]);
    modelobj.print(trajOptModelfile);

    % Inputs, Pathing, & Settings
    trajOptModelfile = fullfile(dataDirectory, [opensimModelName, '_trajopt', osimModelExt]);
    trainIKfile = outputIKFilteredFilename;
    trainLCSfile = fullfile(dataDirectory,lcsFile);
    trainRCSfile = fullfile(dataDirectory,rcsFile);
    trainStart = 30;
    trainEnd = 65;

    % For this example, predict within-subject across steps in the trial,
    % you can also predict across trials within-subject as well as predict
    % across-subjects (so long as you normalize with standard scaler first)
    % (for more on predicting across-subjects, see CSInverseKinematics
    % example, as this requires optimizing unnormalizing parameters)
    predictIKfile = trainIKfile; 
    predictLCSfile = trainLCSfile;
    predictRCSfile = trainRCSfile;
    predictStart = 20;
    predictEnd = 30;
    
    % Settings for Muscle Strings - choose CS data labels and predicted muscles
    csShankLabels = {'Bot_Shank', '2nd_Bot_Shank', '2nd_Top_Shank', 'Top_Shank'};
    csThighLabels = {'Bot_Thigh', 'Med_Thigh', 'Top_Thigh'};
    shankLMuscleStrings = {'tibant_l', 'tibpost_l', 'gasmed_l', 'gaslat_l', 'soleus_l', 'edl_l', 'ehl_l', 'fdl_l', 'fhl_l', 'perbrev_l', 'perlong_l'};
    shankRMuscleStrings = {'tibant_r', 'tibpost_r', 'gasmed_r', 'gaslat_r', 'soleus_r', 'edl_r', 'ehl_r', 'fdl_r', 'fhl_r', 'perbrev_r', 'perlong_r'};
    thighLMuscleStrings = {'bflh_l', 'bfsh_l', 'recfem_l', 'semimem_l', 'semiten_l', 'vasint_l', 'vaslat_l', 'vasmed_l'};
    thighRMuscleStrings = {'bflh_r', 'bfsh_r', 'recfem_r', 'semimem_r', 'semiten_r', 'vasint_r', 'vaslat_r', 'vasmed_r'};
    muscleStrings = [shankLMuscleStrings,shankRMuscleStrings,thighLMuscleStrings,thighRMuscleStrings];

    % Load Training IK Data and Convert to Radian to Calc Fiber Lengths
    trainIKData = ReadOpenSimData(trainIKfile);
    origtraintime = trainIKData.time;
    trainIKData.time =  trainIKData.time(origtraintime >= trainStart & origtraintime <= trainEnd);
    trimmedTrainTime = trainIKData.time;
    trainIKData.data =  trainIKData.data(origtraintime >= trainStart & origtraintime <= trainEnd, :);
    
    numFrames = length(trainIKData.time);
    for i = 1:size(trainIKData.data,2)
        if ~contains(trainIKData.labels(i), {'tx','ty','tz'})
            trainIKData.data(:,i)  = trainIKData.data(:,i)*pi/180;
        end
    end
    
    % Load the osim model and get access to desired muscles
    osimModel = Model(trajOptModelfile);
    state =  osimModel.initSystem();
    numQ = state.getQ().size();
    for i = 1:length(muscleStrings)
        muscleOjbStruct.(muscleStrings{i}) = osimModel.getMuscles().get(muscleStrings{i});
        muscleFLStruct.(muscleStrings{i}) = zeros(1,numFrames);
    end
    
    % Update model state and get each muscles fiber length for given IK
    for i = 1:numFrames
        clc
        fprintf('Processing frame %d out of %d...', i, numFrames)
        for j = 1:numQ
            osimModel.updCoordinateSet().get(j-1).setValue(state, trainIKData.data(i,j))
        end

        % Update model with kinematics and get muscles
        osimModel.realizePosition(state);
    
        % Get the muscle states (full MTLs)
        for k = 1:length(muscleStrings)
            muscleFLStruct.(muscleStrings{k})(i) = muscleOjbStruct.(muscleStrings{k}).getLength(state);
        end
    end
    
    %% Load the CS Profiles for analysis and making predictive models
    lcsTrainDataStruct = ReadOpenSimData(trainLCSfile);
    lcsTrainShankData = lcsTrainDataStruct.data(:,contains(lcsTrainDataStruct.labels, csShankLabels));
    lcsTrainThighData = lcsTrainDataStruct.data(:,contains(lcsTrainDataStruct.labels, csThighLabels));
    lcsTrainIMUData = lcsTrainDataStruct.data(:,contains(lcsTrainDataStruct.labels, 'BNO'));
    lcsTrainTime = lcsTrainDataStruct.time;

    rcsTrainDataStruct = ReadOpenSimData(trainRCSfile);
    rcsTrainShankData = rcsTrainDataStruct.data(:,contains(rcsTrainDataStruct.labels, csShankLabels));
    rcsTrainThighData = rcsTrainDataStruct.data(:,contains(rcsTrainDataStruct.labels, csThighLabels));
    rcsTrainIMUData = rcsTrainDataStruct.data(:,contains(rcsTrainDataStruct.labels, 'BNO'));
    rcsTrainTime = rcsTrainDataStruct.time;

    lcsTrainShankData =  lcsTrainShankData(lcsTrainTime >= trainStart & lcsTrainTime <= trainEnd, :);
    lcsTrainThighData =  lcsTrainThighData(lcsTrainTime >= trainStart & lcsTrainTime <= trainEnd, :);
    rcsTrainShankData =  rcsTrainShankData(rcsTrainTime >= trainStart & rcsTrainTime <= trainEnd, :);
    rcsTrainThighData =  rcsTrainThighData(rcsTrainTime >= trainStart & rcsTrainTime <= trainEnd, :);
    
    lcsTrainShankData_byChannel = zeros(size(lcsTrainShankData));
    rcsTrainShankData_byChannel = zeros(size(rcsTrainShankData));
    for i = 1:size(lcsTrainShankData,2)
        lcsTrainShankData_byChannel(:,i) = abs( lcsTrainShankData(:,i) );
        rcsTrainShankData_byChannel(:,i) = abs( rcsTrainShankData(:,i) );
        lcsTrainShankData_byChannel(:,i) = normalize(lcsTrainShankData_byChannel(:,i), 'range', [0 1]);
        rcsTrainShankData_byChannel(:,i) = normalize(rcsTrainShankData_byChannel(:,i), 'range', [0 1]);
    end
    lcsTrainShankDataProcessed = normalize(mean(lcsTrainShankData_byChannel, 2), 'range', [0 1]);
    rcsTrainShankDataProcessed = normalize(mean(rcsTrainShankData_byChannel, 2), 'range', [0 1]);

    lcsTrainThighData_byChannel = zeros(size(lcsTrainThighData));
    rcsTrainThighData_byChannel = zeros(size(rcsTrainThighData));
    for i = 1:size(lcsTrainThighData,2)
        lcsTrainThighData_byChannel(:,i) = abs( lcsTrainThighData(:,i) );
        rcsTrainThighData_byChannel(:,i) = abs( rcsTrainThighData(:,i) );
        lcsTrainThighData_byChannel(:,i) = normalize(lcsTrainThighData_byChannel(:,i), 'range', [0 1]);
        rcsTrainThighData_byChannel(:,i) = normalize(rcsTrainThighData_byChannel(:,i), 'range', [0 1]);
    end
    lcsTrainThighDataProcessed = normalize(mean(lcsTrainThighData_byChannel, 2), 'range', [0 1]);
    rcsTrainThighDataProcessed = normalize(mean(rcsTrainThighData_byChannel, 2), 'range', [0 1]);

    % Calc predictive power of fiber lengths on respective CS signals from
    % each body segment using multiple-linear regressions
    XRegressLShank = zeros(length(trimmedTrainTime), length(shankLMuscleStrings));
    XRegressRShank = zeros(length(trimmedTrainTime), length(shankRMuscleStrings));
    for i = 1:length(shankLMuscleStrings)
        XRegressLShank(:,i) = normalize(muscleFLStruct.(shankLMuscleStrings{i}), 'range', [0 1]);
        XRegressRShank(:,i) = normalize(muscleFLStruct.(shankRMuscleStrings{i}), 'range', [0 1]);
    end
    YRegressLShank = lcsTrainShankDataProcessed;
    YRegressRShank = rcsTrainShankDataProcessed;

    XRegressLThigh = zeros(length(trimmedTrainTime), length(thighLMuscleStrings));
    XRegressRThigh = zeros(length(trimmedTrainTime), length(thighRMuscleStrings));
    for i = 1:length(thighLMuscleStrings)
        XRegressLThigh(:,i) = normalize(muscleFLStruct.(thighLMuscleStrings{i}), 'range', [0 1]);
        XRegressRThigh(:,i) = normalize(muscleFLStruct.(thighRMuscleStrings{i}), 'range', [0 1]);
    end
    YRegressLThigh = lcsTrainThighDataProcessed;
    YRegressRThigh = rcsTrainThighDataProcessed;

    % Multiple-Linear Regression from FLs to CS
    x = XRegressLShank; y = YRegressLShank;
    % x = XRegressLThigh; y = YRegressLThigh;

    xMat = [ones(length(x), 1), x];
    [b,bint,r,rint,stats] = regress(y,xMat);
    yfit = smooth(xMat*b);
    PearsonsR = corrcoef(y,yfit)

    % Visualize how well the FLs (and ACSAs) expalain the CS signal from
    % the multilinear regression (Composite-Cross Sectional Area)
    figure
    plot(y,'Color','#6e9db6','LineWidth',3)
    hold on
    plot(yfit, '--', 'Color','#b76e79', 'LineWidth',3)
    legend('CS','CCSA')
    legend boxoff
    set(gca, 'box', 'off')
    ylim([0 1])
    ylabel('Normalized Profile')
    xlabel('Frame')
    savefig(fullfile(dataDirectory, 'CSvsCCSA.fig'));
    hold off

    %% Go the other way now and build predictive models of FLs from CS for
    % prediction and tracking in optimal control problem (this step
    % requires deep learning toolbox, and requires training models for each
    % segment separately to maximize accuracy)
    
    import org.opensim.modeling.*;

    % Prep the training data for learning a predictive model
    YTrainLShank = zeros(length(trimmedTrainTime), length(shankLMuscleStrings));
    YTrainRShank = zeros(length(trimmedTrainTime), length(shankRMuscleStrings));
    for i = 1:length(shankLMuscleStrings)
        YTrainLShank(:,i) = muscleFLStruct.(shankLMuscleStrings{i});
        YTrainRShank(:,i) = muscleFLStruct.(shankRMuscleStrings{i});
    end
    XTrainLShank = lcsTrainShankDataProcessed;
    XTrainRShank = rcsTrainShankDataProcessed;

    YTrainLThigh = zeros(length(trimmedTrainTime), length(thighLMuscleStrings));
    YTrainRThigh = zeros(length(trimmedTrainTime), length(thighRMuscleStrings));
    for i = 1:length(thighLMuscleStrings)
        YTrainLThigh(:,i) = muscleFLStruct.(thighLMuscleStrings{i});
        YTrainRThigh(:,i) = muscleFLStruct.(thighRMuscleStrings{i});
    end
    XTrainLThigh = lcsTrainThighDataProcessed;
    XTrainRThigh = rcsTrainThighDataProcessed;

    % Augment the signals for training
    numTimes = 5;
    randMagAsPercent = 0.001;
    XTrainLShankAug = [];
    XTrainLThighAug = [];
    XTrainRShankAug = [];
    XTrainRThighAug = [];
    YTrainLShankAug = [];
    YTrainLThighAug = [];
    YTrainRShankAug = [];
    YTrainRThighAug = [];
    for aug = 1:numTimes
        XTrainLShankAug = [XTrainLShankAug; XTrainLShank + XTrainLShank.*randMagAsPercent.*randn(size(XTrainLShank))];
        XTrainLThighAug = [XTrainLThighAug; XTrainLThigh + XTrainLThigh.*randMagAsPercent.*randn(size(XTrainLThigh))];
        XTrainRShankAug = [XTrainRShankAug; XTrainRShank + XTrainRShank.*randMagAsPercent.*randn(size(XTrainRShank))];
        XTrainRThighAug = [XTrainRThighAug; XTrainRThigh + XTrainRThigh.*randMagAsPercent.*randn(size(XTrainRThigh))];
        YTrainLShankAug = [YTrainLShankAug; YTrainLShank];
        YTrainLThighAug = [YTrainLThighAug; YTrainLThigh];
        YTrainRShankAug = [YTrainRShankAug; YTrainRShank];
        YTrainRThighAug = [YTrainRThighAug; YTrainRThigh];
    end
    
    %% Prep the prediction data (load and process pred CS and FLs)
    % Load the FL data for validating predictions
    predictIKData = ReadOpenSimData(predictIKfile);
    origpredicttime = predictIKData.time;
    predictIKData.time =  predictIKData.time(origpredicttime >= predictStart & origpredicttime <= predictEnd);
    trimmedPredictTime = predictIKData.time;
    predictIKData.data =  predictIKData.data(origpredicttime >= predictStart & origpredicttime <= predictEnd, :);
    
    numFrames = length(predictIKData.time);
    for i = 1:size(predictIKData.data,2)
        if ~contains(predictIKData.labels(i), {'tx','ty','tz'})
            predictIKData.data(:,i)  = predictIKData.data(:,i)*pi/180;
        end
    end
    
    % Load the osim model and get access to desired muscles
    osimModel = Model(trajOptModelfile);
    state =  osimModel.initSystem();
    numQ = state.getQ().size();
    for i = 1:length(muscleStrings)
        predmuscleOjbStruct.(muscleStrings{i}) = osimModel.getMuscles().get(muscleStrings{i});
        predmuscleFLStruct.(muscleStrings{i}) = zeros(1,numFrames);
    end
    
    % Update model state and get each muscles fiber length for given IK
    for i = 1:numFrames
        clc
        fprintf('Processing frame %d out of %d...', i, numFrames)
        for j = 1:numQ
            osimModel.updCoordinateSet().get(j-1).setValue(state, predictIKData.data(i,j))
        end
    
        % Update model with kinematics and get muscles
        osimModel.realizePosition(state);
    
        % Get the muscle states (full MTLs)
        for k = 1:length(muscleStrings)
            predmuscleFLStruct.(muscleStrings{k})(i) = predmuscleOjbStruct.(muscleStrings{k}).getLength(state);
        end
    end

    %% Prep predictive CS data for extrapolating trained model 
    lcsPredictDataStruct = ReadOpenSimData(predictLCSfile);
    lcsPredictShankData = lcsPredictDataStruct.data(:,contains(lcsPredictDataStruct.labels, csShankLabels));
    lcsPredictThighData = lcsPredictDataStruct.data(:,contains(lcsPredictDataStruct.labels, csThighLabels));
    lcsPredictIMUData = lcsPredictDataStruct.data(:,contains(lcsPredictDataStruct.labels, 'BNO'));
    lcsPredictTime = lcsPredictDataStruct.time;
    
    rcsPredictDataStruct = ReadOpenSimData(predictRCSfile);
    rcsPredictShankData = rcsPredictDataStruct.data(:,contains(rcsPredictDataStruct.labels, csShankLabels));
    rcsPredictThighData = rcsPredictDataStruct.data(:,contains(rcsPredictDataStruct.labels, csThighLabels));
    rcsPredictIMUData = rcsPredictDataStruct.data(:,contains(rcsPredictDataStruct.labels, 'BNO'));
    rcsPredictTime = rcsPredictDataStruct.time;

    lcsPredictShankData =  lcsPredictShankData(lcsPredictTime >= predictStart & lcsPredictTime <= predictEnd, :);
    lcsPredictThighData =  lcsPredictThighData(lcsPredictTime >= predictStart & lcsPredictTime <= predictEnd, :);
    rcsPredictShankData =  rcsPredictShankData(rcsPredictTime >= predictStart & rcsPredictTime <= predictEnd, :);
    rcsPredictThighData =  rcsPredictThighData(rcsPredictTime >= predictStart & rcsPredictTime <= predictEnd, :);
    
    lcsPredictShankData_byChannel = zeros(size(lcsPredictShankData));
    rcsPredictShankData_byChannel = zeros(size(rcsPredictShankData));
    for i = 1:size(lcsPredictShankData,2)
        lcsPredictShankData_byChannel(:,i) = abs( lcsPredictShankData(:,i) );
        rcsPredictShankData_byChannel(:,i) = abs( rcsPredictShankData(:,i) );
        lcsPredictShankData_byChannel(:,i) = normalize(lcsPredictShankData_byChannel(:,i), 'range', [0 1]);
        rcsPredictShankData_byChannel(:,i) = normalize(rcsPredictShankData_byChannel(:,i), 'range', [0 1]);
    end
    lcsPredictShankDataProcessed = normalize(mean(lcsPredictShankData_byChannel, 2), 'range', [0 1]);
    rcsPredictShankDataProcessed = normalize(mean(rcsPredictShankData_byChannel, 2), 'range', [0 1]);

    lcsPredictThighData_byChannel = zeros(size(lcsPredictThighData));
    rcsPredictThighData_byChannel = zeros(size(rcsPredictThighData));
    for i = 1:size(lcsPredictThighData,2)
        lcsPredictThighData_byChannel(:,i) = abs( lcsPredictThighData(:,i) );
        rcsPredictThighData_byChannel(:,i) = abs( rcsPredictThighData(:,i) );
        lcsPredictThighData_byChannel(:,i) = normalize(lcsPredictThighData_byChannel(:,i), 'range', [0 1]);
        rcsPredictThighData_byChannel(:,i) = normalize(rcsPredictThighData_byChannel(:,i), 'range', [0 1]);
    end
    lcsPredictThighDataProcessed = normalize(mean(lcsPredictThighData_byChannel, 2), 'range', [0 1]);
    rcsPredictThighDataProcessed = normalize(mean(rcsPredictThighData_byChannel, 2), 'range', [0 1]);

    % Prep predictive data for extrapolation and validation
    YPredictLShank = zeros(length(trimmedPredictTime), length(shankLMuscleStrings));
    YPredictRShank = zeros(length(trimmedPredictTime), length(shankRMuscleStrings));
    for i = 1:length(shankLMuscleStrings)
        YPredictLShank(:,i) = predmuscleFLStruct.(shankLMuscleStrings{i});
        YPredictRShank(:,i) = predmuscleFLStruct.(shankRMuscleStrings{i});
    end
    XPredictLShank = lcsPredictShankDataProcessed;
    XPredictRShank = rcsPredictShankDataProcessed;

    YPredictLThigh = zeros(length(trimmedPredictTime), length(thighLMuscleStrings));
    YPredictRThigh = zeros(length(trimmedPredictTime), length(thighRMuscleStrings));
    for i = 1:length(thighLMuscleStrings)
        YPredictLThigh(:,i) = predmuscleFLStruct.(thighLMuscleStrings{i});
        YPredictRThigh(:,i) = predmuscleFLStruct.(thighRMuscleStrings{i});
    end
    XPredictLThigh = lcsPredictThighDataProcessed;
    XPredictRThigh = rcsPredictThighDataProcessed;

    %% Set-Up Net and Training Opts to Learn relationship from CS to FLs
    layers = [ ...
        sequenceInputLayer(1)
        batchNormalizationLayer
        convolution1dLayer(10, 100, 'Padding', 'same', 'NumChannels', 1, 'WeightsInitializer','he')
        maxPooling1dLayer(2, 'Padding', 'same')
        fullyConnectedLayer(36, 'WeightsInitializer','he')
        leakyReluLayer
        dropoutLayer(0.1)
        fullyConnectedLayer(120, 'WeightsInitializer','he')
        leakyReluLayer
        fullyConnectedLayer(length(muscleStrings), 'WeightsInitializer','he')
        regressionLayer];
    regression_net = layerGraph(layers);
    
    rng(1)
    options = trainingOptions('adam', ...
            'ExecutionEnvironment','auto', ...
            'MaxEpochs', ceil(5e4), ...
            'InitialLearnRate', 0.0001, ...
            'Verbose',true, ...
            'L2Regularization', 1.0e-8, ...
            'VerboseFrequency', 100, ...
            'OutputNetwork','best-validation-loss');
            % 'Plots','training-progress');

    lshank_untrainednet = regression_net;
    lthigh_untrainednet = regression_net;
    rshank_untrainednet = regression_net;
    rthigh_untrainednet = regression_net;

    % Training for LShank
    larrayIn = [sequenceInputLayer(1)];
    larrayOut = [fullyConnectedLayer(length(shankLMuscleStrings), 'WeightsInitializer','he')];
    lshank_untrainednet = replaceLayer(lshank_untrainednet,'sequenceinput',larrayIn);
    lshank_untrainednet = replaceLayer(lshank_untrainednet,'fc_3',larrayOut);
    options.ValidationData = {XPredictLShank',YPredictLShank'};
    lshanknet = trainNetwork(XTrainLShank',YTrainLShank',lshank_untrainednet,options);

    % Training for LThigh
    larrayIn = [sequenceInputLayer(1)];
    larrayOut = [fullyConnectedLayer(length(thighLMuscleStrings), 'WeightsInitializer','he')];
    lthigh_untrainednet = replaceLayer(lthigh_untrainednet,'sequenceinput',larrayIn);
    lthigh_untrainednet = replaceLayer(lthigh_untrainednet,'fc_3',larrayOut);
    options.ValidationData = {XPredictLThigh',YPredictLThigh'};
    lthighnet = trainNetwork(XTrainLThigh',YTrainLThigh',lthigh_untrainednet,options);

    % Training for RShank
    larrayIn = [sequenceInputLayer(1)];
    larrayOut = [fullyConnectedLayer(length(shankRMuscleStrings), 'WeightsInitializer','he')];
    rshank_untrainednet = replaceLayer(rshank_untrainednet,'sequenceinput',larrayIn);
    rshank_untrainednet = replaceLayer(rshank_untrainednet,'fc_3',larrayOut);
    options.ValidationData = {XPredictRShank',YPredictRShank'};
    rshanknet = trainNetwork(XTrainRShank',YTrainRShank',rshank_untrainednet,options);

    % Training for RThigh
    larrayIn = [sequenceInputLayer(1)];
    larrayOut = [fullyConnectedLayer(length(thighRMuscleStrings), 'WeightsInitializer','he')];
    rthigh_untrainednet = replaceLayer(rthigh_untrainednet,'sequenceinput',larrayIn);
    rthigh_untrainednet = replaceLayer(rthigh_untrainednet,'fc_3',larrayOut);
    options.ValidationData = {XPredictRThigh',YPredictRThigh'};
    rthighnet = trainNetwork(XTrainRThigh',YTrainRThigh',rthigh_untrainednet,options);

    % Analyze prediction accuracy and package into .mot file for tracking
    % in OpenSim MOCO in the following section

    % Predict the outcomes with the models
    YTrainLShank_estimated = predict(lshanknet, XTrainLShank')';
    YPredictLShank_estimated = predict(lshanknet, XPredictLShank')';
    YTrainLThigh_estimated = predict(lthighnet, XTrainLThigh')';
    YPredictLThigh_estimated = predict(lthighnet, XPredictLThigh')';
    YTrainRShank_estimated = predict(rshanknet, XTrainRShank')';
    YPredictRShank_estimated = predict(rshanknet, XPredictRShank')';
    YTrainRThigh_estimated = predict(rthighnet, XTrainRThigh')';
    YPredictRThigh_estimated = predict(rthighnet, XPredictRThigh')';
    
    predictFlStuct = struct;

    trainLoss = [];
    testLoss = [];
    trimTrain = 400:700;
    trimPred = 400:700;
    % trimTrain = 10:length(YTrainLShank_estimated)-10;
    % trimPred = 10:length(YPredictLShank_estimated)-10;
    for i = 1:length(shankLMuscleStrings)

        % trainLoss(i) = rms(smooth(YTrainLShank_estimated(trimTrain,i)) - YTrainLShank(trimTrain,i), 'all');
        % testLoss(i) = rms(smooth(YPredictLShank_estimated(trimPred,i)) - YPredictLShank(trimPred,i), 'all');

        % Save the FL Predictions
        predictFLStruct.(shankLMuscleStrings{i}) = smooth(YPredictLShank_estimated(trimPred,i));

        % Plots
        % figure
        % hold on
        % title(shankLMuscleStrings{i})
        % plot(YTrainLShank(trimTrain,i),smooth(YTrainLShank_estimated(trimTrain,i)),".")
        % hold on
        % plot(YTrainLShank(trimTrain,i),YTrainLShank(trimTrain,i))
        % hold off
        % xlabel("True")
        % ylabel("Predicted")
        % 
        % figure
        % hold on
        % title(muscleStrings{i})
        % plot(smooth(YTrainLShank_estimated(trimTrain,i)))
        % plot(YTrainLShank(trimTrain,i))
        % hold off

        % Plots
        figure
        hold on
        title(shankLMuscleStrings{i})
        plot(YPredictLShank(trimPred,i),smooth(YPredictLShank_estimated(trimPred,i)),".")
        hold on
        plot(YPredictLShank(trimPred,i),YPredictLShank(trimPred,i))
        hold off
        xlabel("True")
        ylabel("Predicted")

        figure
        hold on
        title(shankLMuscleStrings{i})
        plot(smooth(YPredictLShank_estimated(trimPred,i)))
        plot(YPredictLShank(trimPred,i))
        hold off
    end
    % trainLoss
    % testLoss

    % trainLoss = [];
    % testLoss = [];
    for i = 1:length(thighLMuscleStrings)
        % trainLoss(i) = rms(smooth(YTrainLThigh_estimated(trimTrain,i)) - YTrainLThigh(trimTrain,i), 'all');
        % testLoss(i) = rms(smooth(YPredictLThigh_estimated(trimPred,i)) - YPredictLThigh(trimPred,i), 'all');

        % Save the FL Predictions
        predictFLStruct.(thighLMuscleStrings{i}) = smooth(YPredictLThigh_estimated(trimPred,i));

        % Plots
        figure
        hold on
        title(thighLMuscleStrings{i})
        plot(YPredictLThigh(trimPred,i),smooth(YPredictLThigh_estimated(trimPred,i)),".")
        hold on
        plot(YPredictLThigh(trimPred,i),YPredictLThigh(trimPred,i))
        hold off
        xlabel("True")
        ylabel("Predicted")

        figure
        hold on
        title(thighLMuscleStrings{i})
        plot(smooth(YPredictLThigh_estimated(trimPred,i)))
        plot(YPredictLThigh(trimPred,i))
        hold off
    end
    % testLoss

    % trainLoss = [];
    % testLoss = [];
    for i = 1:length(shankRMuscleStrings)
        % trainLoss(i) = rms(smooth(YTrainRShank_estimated(trimTrain,i)) - YTrainRShank(trimTrain,i), 'all');
        % testLoss(i) = rms(smooth(YPredictRShank_estimated(trimPred,i)) - YPredictRShank(trimPred,i), 'all');

        % Save the FL Predictions
        predictFLStruct.(shankRMuscleStrings{i}) = smooth(YPredictRShank_estimated(trimPred,i));

        % Plots
        % figure
        % hold on
        % title(shankRMuscleStrings{i})
        % plot(YPredictRShank(trimPred,i),smooth(YPredictRShank_estimated(trimPred,i)),".")
        % hold on
        % plot(YPredictRShank(trimPred,i),YPredictRShank(trimPred,i))
        % hold off
        % xlabel("True")
        % ylabel("Predicted")
        % 
        % figure
        % hold on
        % title(shankRMuscleStrings{i})
        % plot(smooth(YPredictRShank_estimated(trimPred,i)))
        % plot(YPredictRShank(trimPred,i))
        % hold off
    end
    % testLoss

    % trainLoss = [];
    % testLoss = [];
    for i = 1:length(thighRMuscleStrings)
        % trainLoss(i) = rms(smooth(YTrainRThigh_estimated(trimTrain,i)) - YTrainRThigh(trimTrain,i), 'all');
        % testLoss(i) = rms(smooth(YPredictRThigh_estimated(trimPred,i)) - YPredictRThigh(trimPred,i), 'all');

        % Save the FL Predictions
        predictFLStruct.(thighRMuscleStrings{i}) = smooth(YPredictRThigh_estimated(trimPred,i));

        % Plots
        % figure
        % hold on
        % title(thighRMuscleStrings{i})
        % plot(YPredictRThigh(trimPred,i),smooth(YPredictRThigh_estimated(trimPred,i)),".")
        % hold on
        % plot(YPredictRThigh(trimPred,i),YPredictRThigh(trimPred,i))
        % hold off
        % xlabel("True")
        % ylabel("Predicted")
        % 
        % figure
        % hold on
        % title(thighRMuscleStrings{i})
        % plot(smooth(YPredictRThigh_estimated(trimPred,i)))
        % plot(YPredictRThigh(trimPred,i))
        % hold off
    end
    % testLoss
end

%% 6. Setup and Solve TrajOpt Problem
clc; close all

% Set the desired number of knot points and start and end times for the
% optimal control simulation (make sure to run a regular tracking
% simulation on just the kinematics to make an initial guess that that
% supports the full problem (every state variable and muscle needs a
% populateds inital guess to run)
knots = 25;
startMOCO = 24.9;
endMOCO = 26.3;

% Create and name an instance of the MocoTrack tool.
import org.opensim.modeling.*;
track = MocoTrack();
track.setName('MocoTrackSolution');

% Construct a ModelProcessor and set it on the tool. The default
% muscles in the model are replaced with optimization-friendly
% DeGrooteFregly2016Muscles, and adjustments are made to the default muscle
% parameters.
baseModel = Model(outputPlacedModelFilename);

% Start editing model here
modelProcessor = ModelProcessor(baseModel);
modelProcessor.append(ModOpIgnoreTendonCompliance());

% https://simtk.org/plugins/phpBB/viewtopicPhpbb.php?f=1815&t=12104&p=34251&start=0&view=
% modelProcessor.append(ModOpTendonComplianceDynamicsModeDGF('implicit'));
modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
% Only valid for DeGrooteFregly2016Muscles.
modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());

% https://simtk.org/plugins/phpBB/viewtopicPhpbb.php?f=1815&t=14291&p=41189&start=0&view=
% modelProcessor.append(ModOpUseImplicitTendonComplianceDynamicsDGF());
modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));

% Remove all the muscles in the model's ForceSet (torque drive)
% modelProcessor.append(ModOpRemoveMuscles());

% Add CoordinateActuators to the model degrees-of-freedom. By default
% ignores if they already have an actuator...
modelProcessor.append(ModOpAddReserves(250));

% modelProcessor.append(ModOpReplaceJointsWithWelds(jointNames));

% Toggle on External Forces
if settings(1) == 0 % If no contact model add the external loads to model
    externalLoadsFilename = fullfile(dataDirectory,ExternalLoadsFile);
    modelProcessor.append(ModOpAddExternalLoads(externalLoadsFilename));
end

tempModel = modelProcessor.updModel();
tempModel.buildSystem();
tempModel.initializeState();
track.setModel(modelProcessor);
% modelobj = modelProcessor.process();
% modelobj.print('visualizeNewModelBeforeTrajopt.osim');

% Construct a TableProcessor of the coordinate data and pass it to the
% tracking tool. TableProcessors can be used in the same way as
% ModelProcessors by appending TableOperators to modify the base table.

% Track Coordinate Data
if settings(2) == 0 || settings(2) == 2
    if (settings(2) == 0)
        track.setStatesReference(TableProcessor(outputIKFilteredFilename));
    elseif (settings(2) == 2)
        track.setStatesReference(TableProcessor(fullfile(dataDirectory, ['ik_', extractBefore(orientationFile,'.sto'), '.mot'])));
    end
    
    track.set_states_global_tracking_weight(10);
    track.set_apply_tracked_states_to_guess(true);

    % Track Marker Data if you want here
    % markers = TimeSeriesTableVec3(fullfile(outputDirectory, markerTrialFile));
    % tableProcessor = TableProcessor(markers.flatten());
    % track.setMarkersReference(tableProcessor);
    % track.set_markers_global_tracking_weight(10);

    % This setting allows extra data columns contained in the states
    % reference that don't correspond to model coordinates.
    track.set_allow_unused_references(true);

    % Since there is only coordinate position data the states references, this
    % setting is enabled to fill in the missing coordinate speed data using
    % the derivative of splined position data.
    track.set_track_reference_position_derivatives(true);

    % Initial time, final time, and mesh interval. 
    track.set_initial_time(startMOCO);
    track.set_final_time(endMOCO);
    track.set_mesh_interval( (endMOCO-startMOCO)/knots )

elseif settings(2) == 1 % Track IMU Data

    % Use this to set the initial guess, then just turn off the cost later
    track.setStatesReference(TableProcessor(outputIKFilteredFilename));
    track.set_states_global_tracking_weight(10);
    track.set_apply_tracked_states_to_guess(true);

    % This setting allows extra data columns contained in the states
    % reference that don't correspond to model coordinates.
    track.set_allow_unused_references(true);

    % Since there is only coordinate position data the states references, this
    % setting is enabled to fill in the missing coordinate speed data using
    % the derivative of splined position data.
    track.set_track_reference_position_derivatives(false);

    % Initial time, final time, and mesh interval. (mocotrack has some defaults
    % that are not exposed, could create your own version of tracking as well)
    track.set_initial_time(startMOCO);
    track.set_final_time(endMOCO);
    track.set_mesh_interval( (endMOCO-startMOCO)/knots )
end

% Set the level of the control penalty weight here if desiring to change it
track.set_control_effort_weight(0.001)

% Instead of calling solve(), call initialize() to receive a pre-configured
% MocoStudy object based on the settings above. Use this to customize the
% problem beyond the MocoTrack interface.

if settings(4) == 0 % If not using CS, can just initialize MocoStudy
    study = track.initialize();

elseif settings(4) == 1 || settings(4) == 2 % If using CS we have to edit XML to add custom goals!
    % Initialize and output a printed template for .omoco
    studyTemplate = track.initialize();
    studyTemplate.print(fullfile(dataDirectory,'temp.omoco'));

    % Add CS Tracking Goal by editing struct, rewrite, load lib, load temp
    % https://blogs.mathworks.com/community/2010/11/01/xml-and-matlab-navigating-a-tree/
    % https://blogs.mathworks.com/community/2010/09/13/simple-xml-node-creation/
    % Read the xml template into a struct for editing using XML Objects
    opensimCommon.LoadOpenSimLibrary(fullfile(currentDirectory, 'CSTrackingGoalPluginRelease', 'MocoCSTrackingGoal.dll'));
    xmlNode = xmlread(fullfile(dataDirectory,'temp.omoco'));
    OpenSimDocumentNode = xmlNode.getDocumentElement;
    OpenSimDocumentChildrenNodesList = OpenSimDocumentNode.getChildNodes;
    MocoStudyChildrenNodesList = OpenSimDocumentChildrenNodesList.item(1).getChildNodes;
    MocoProblemChildrenNodesList = MocoStudyChildrenNodesList.item(3).getChildNodes;
    MocoPhaseChildrenNodesList = MocoProblemChildrenNodesList.item(3).getChildNodes;

    goalsNode = MocoPhaseChildrenNodesList.getElementsByTagName('goals').item(0);
    MocoCSTrackingGoalNode = xmlNode.createElement('MocoCSTrackingGoal');
    MocoCSTrackingGoalNode.setAttribute('name','cs_tracking');
    goalsNode.appendChild(MocoCSTrackingGoalNode);
    xmlwrite(fullfile(dataDirectory,'temp.omoco'), xmlNode);
    study = MocoStudy(fullfile(dataDirectory,'temp.omoco'));
    study.print(fullfile(dataDirectory,'temp.omoco'));
end

% Access and edit solver settings
solver = MocoCasADiSolver.safeDownCast(study.updSolver());
solver.set_multibody_dynamics_mode('explicit')
% solver.set_multibody_dynamics_mode('implicit')
solver.set_optim_max_iterations(1000);
% solver.set_optim_max_iterations(3000);
% solver.set_parallel(6) % 1 is use all cores, 0 is 1 core, 6 is 6

% https://simtk.org/plugins/phpBB/viewtopicPhpbb.php?f=1815&t=14650&p=42355&start=0&view=
% this one debugs the nans in the jacobian matrices and how to create the
% initial guess when using the implicit forms of multibody dynamics
% https://simtk.org/plugins/phpBB/viewtopicPhpbb.php?f=1815&t=12104&p=34251&start=0&view=
% solver.set_minimize_implicit_auxiliary_derivatives(true)
% N = 92;
% waux = 0.0001/N; % N should be number of muscles
% solver.set_implicit_auxiliary_derivatives_weight(waux)

% if (settings(2) == 0)
%     knownGuessTable = TimeSeriesTable(outputIKFilteredFilename);
% elseif (settings(2) == 2)
%     knownGuessTable = TimeSeriesTable(fullfile(outputDirectory, ['ik_', extractBefore(orientationFile,'.sto'), '.mot']));
% end
    
% problem = study.updProblem();
% solver.resetProblem(problem);
% guess = solver.createGuess('bounds');
% guess.randomizeAdd();
% solver.setGuess(guess);
% guess.insertStatesTrajectory(knownGuessTable,true);
% guess.generateSpeedsFromValues();
% guess.generateAccelerationsFromValues();
% guess.write('temp_guesstraj.sto');
% solver.setGuessFile('temp_guesstraj.sto');

% solver.set_multibody_dynamics_mode('implicit')
% solver.set_optim_sparsity_detection("random")
% solver.set_transcription_scheme('hermite-simpson');
% solver.set_optim_convergence_tolerance(1e-2);
% solver.set_optim_constraint_tolerance(1e-2);
% solver.set_optim_hessian_approximation('exact'); % By default uses quasi-newton steps
% solver.set_optim_finite_difference_scheme('forward'); % Forward is faster but less accurate
% solver.set_parallel(0) % Turn off parallel computations
% solver.set_output_interval(10); % Monitor progress with everything 10th iteration

% Defaults for MocoTrack

%     solver: MocoCasADiSolver
%     multibody_dynamics_mode: explicit
%     transcription_scheme: Hermite-Simpson
%     optim_convergence_tolerance: 1e-2
%     optim_constraint_tolerance: 1e-2
%     optim_sparsity_detection: random
%     optim_finite_difference_scheme: 'forward'

% Get a reference to the MocoControlGoal that is added to every MocoTrack
% problem by default
problem = study.updProblem();
effort = MocoControlGoal.safeDownCast(problem.updGoal("control_effort"));
% effort.setWeight(1e-3); % default (can also set above)

% Put a large weight on the pelvis CoordinateActuators, which act as the
% residual, or 'hand-of-god', forces which we would like to keep as small
% as possible.
modelobj = modelProcessor.process();
trajOptModelfile = fullfile(dataDirectory, [opensimModelName, '_trajopt', osimModelExt]);
modelobj.print(trajOptModelfile);
modelobj.initSystem();
forceSet = modelobj.getForceSet();
for i = 0:forceSet.getSize()-1
    forcePath = forceSet.get(i).getAbsolutePathString();
    if contains(string(forcePath), 'pelvis')
        effort.setWeightForControl(forcePath, 100);
        % effort.setWeightForControl(forcePath, 0);
    end
end

% Unbound pelvix_tx to track any starting & ending x-position
problem.setStateInfo('/jointset/ground_pelvis/pelvis_tx/value', [-inf inf], [-inf inf]);

% Track IMU data if toggled on
if settings(2) == 1
    % % Created IMU trajectories from pred. soln, then track them
    % predictedSolution = MocoTrajectory(fullfile(outputDirectory,'MOCOPredictionSoln.sto'));
    %
    % accelPaths = StdVectorString();
    % accelPaths.add('.*accelerometer_signal');
    % accelTimeSeriesTableVec3 = opensimSimulation.analyzeVec3(modelobj, ...
    %     predictedSolution.exportToStatesTable(), ...
    %     predictedSolution.exportToControlsTable(), ...
    %     accelPaths);
    % accelTimeSeriesTableVec3.setColumnLabels(imuFramePaths);
    %
    % gyroPaths = StdVectorString();
    % gyroPaths.add('.*gyroscope_signal');
    % gyroTimeSeriesTableVec3 = opensimSimulation.analyzeVec3(modelobj, ...
    %     predictedSolution.exportToStatesTable(), ...
    %     predictedSolution.exportToControlsTable(), ...
    %     gyroPaths);
    % gyroTimeSeriesTableVec3.setColumnLabels(imuFramePaths);

    % Debug here to visualize the signals
    % plotAccelerationSignals(accelTimeSeriesTableVec3);
    % plotAccelerationSignals(gyroTimeSeriesTableVec3);

    % Track the IMU component output accelerometer_signal and gyroscope_signal
    accelTracking = org.opensim.modeling.MocoAccelerationTrackingGoal('accel_tracking');
    accelTracking.setAccelerationReference(accelTimeSeriesTableVec3);
    accelTracking.setFramePaths(accelTimeSeriesTableVec3.getColumnLabels);
    accelTracking.setWeight(5);
    accelTracking.setGravityOffset(true);
    accelTracking.setExpressAccelerationsInTrackingFrames(true);
    %     accelTracking.setWeightForFrame(accelTimeSeriesTableVec3.getColumnLabels.get(0),100) % add weight for torso IMU
    %     accelTracking.setWeightForFrame(accelTimeSeriesTableVec3.getColumnLabels.get(1),100) % add weight for pelvis IMU
    problem.addGoal(accelTracking);

    gyroTracking = org.opensim.modeling.MocoAngularVelocityTrackingGoal('gyro_tracking');
    gyroTracking.setAngularVelocityReference(gyroTimeSeriesTableVec3)
    gyroTracking.setFramePaths(gyroTimeSeriesTableVec3.getColumnLabels);
    gyroTracking.setWeight(10);
    %     gyroTracking.setWeightForFrame(gyroTimeSeriesTableVec3.getColumnLabels.get(0),100) % add weight for torso IMU
    %     gyroTracking.setWeightForFrame(gyroTimeSeriesTableVec3.getColumnLabels.get(1),100) % add weight for pelvis IMU
    problem.addGoal(gyroTracking);

    % Warm start with initial guess for model from prev simulation
    warmStartFile = fullfile(dataDirectory,'InitializeMocoTrajectory.sto');
    solver.setGuessFile(warmStartFile);
    %     if settings(3) == 1 && settings(5) == 1
    %         % Do not set the initial guess, it has to construct from bounds,
    %         % found bug where .addScaleFactor doesn't work with initial
    %         % guesses since it is not a moco parameter we can put in the
    %         % trajectory 
    %         solver.setGuessFile(warmStartFile);
    %     else
    %         solver.setGuessFile(warmStartFile);
    %     end

    % Disable the state tracking goal on mocotrack instance
    stateTrackingGoalRef = MocoStateTrackingGoal.safeDownCast(problem.updGoal("state_tracking"));
    stateTrackingGoalRef.setEnabled(false);

    % Need to set initial bounds on state to the desired kinematics (assume
    % we know the initial position and use some default starting pose)
    warmSolnTraj = MocoTrajectory(warmStartFile);
    valuesOsimMat = warmSolnTraj.getValuesTrajectory();
    valueNamesOsim = warmSolnTraj.getValueNames();
    numDOF = valueNamesOsim.capacity();
    valuesMatrix = valuesOsimMat.getAsMat();
    timesMatrix = warmSolnTraj.getTimeMat();
    for i = 1:numDOF
        % Clamp Initial State to Correct Starting Calibration Pose
%         problem.setStateInfo(valueNamesOsim.get(i-1).toCharArray()', [], valuesMatrix(1,i));
        problem.setStateInfo(valueNamesOsim.get(i-1), [], valuesMatrix(1,i));

        % If we want to bound any particular DOF based on prior knowledge
        %         if strcmp(valueNamesOsim.get(i-1).toCharArray()', '/jointset/ground_pelvis/pelvis_tz/value')
        %             problem.setStateInfo(valueNamesOsim.get(i-1).toCharArray()', [min(valuesMatrix(:,i)), max(valuesMatrix(:,i))], valuesMatrix(1,i));
        %         else
        %             problem.setStateInfo(valueNamesOsim.get(i-1).toCharArray()', [], valuesMatrix(1,i));
        %         end
    end

    % Debug/Check your work
    %     phase = MocoPhase.safeDownCast(problem.updPhase());
    %     phase.getStateInfo(valueNamesOsim.get(7).toCharArray()');

elseif settings(2) == 2
    % Warm start with initial guess for model from prev simulation
    warmStartFile = fullfile(dataDirectory,'InitializeMocoTrajectory.sto');
    solver.setGuessFile(warmStartFile);

    % Need to set initial bounds on state to the desired kinematics (assume
    % we know the initial position and use some default starting pose)
    warmSolnTraj = MocoTrajectory(warmStartFile);
    valuesOsimMat = warmSolnTraj.getValuesTrajectory();
    valueNamesOsim = warmSolnTraj.getValueNames();
    numDOF = valueNamesOsim.capacity();
    valuesMatrix = valuesOsimMat.getAsMat();
    timesMatrix = warmSolnTraj.getTimeMat();
%     startPhase = problem.getPhase(0);
    for i = 1:numDOF
        % Clamp Initial State to Correct Starting Calibration Pose
        problem.setStateInfo(valueNamesOsim.get(i-1).toCharArray()', [], valuesMatrix(1,i));

%         currVarInfo = startPhase.getStateInfo(valueNamesOsim.get(i-1));
%         currMocoBounds = currVarInfo.getBounds();
%         valueNamesOsim.get(i-1)
%         valuesMatrix(1,i)

        % Bound state variables to within range of motion of joints
        problem.setStateInfo(valueNamesOsim.get(i-1), [min(valuesMatrix(:,i)) - .2*abs(min(valuesMatrix(:,i))), max(valuesMatrix(:,i)) + .2*abs(max(valuesMatrix(:,i)))], valuesMatrix(1,i));
        
        % If we want to bound any particular DOF based on prior knowledge
        %         if strcmp(valueNamesOsim.get(i-1).toCharArray()', '/jointset/ground_pelvis/pelvis_tz/value')
        %             problem.setStateInfo(valueNamesOsim.get(i-1).toCharArray()', [min(valuesMatrix(:,i)), max(valuesMatrix(:,i))], valuesMatrix(1,i));
        %         else
        %             problem.setStateInfo(valueNamesOsim.get(i-1).toCharArray()', [], valuesMatrix(1,i));
        %         end
    end
end

% Track EMG data if toggled on
if settings(3) == 1
    % Part 3b: Create a MocoControlTrackingGoal, set its weight, and provide
    % the EMG data as the tracking reference. We also need to specify the
    % reference labels for the muscles whose EMG we will track.
    emgTracking = MocoControlTrackingGoal('emg_tracking');
    emgTracking.setWeight(10);
    if settings(5) == 0
        refEMGTable = fullfile(dataDirectory, 'enter_synthetic_activations_here.mot');
        %     emgReference = TimeSeriesTable('emg.sto');
        emgTracking.setReference(TableProcessor(refEMGTable));
        %     tracking.setReferenceLabel('/forceset/gasmed_l', 'gastrocnemius');

        % Part 3c: The EMG signals in the tracking are all normalized to have
        % a maximum value of 1, but the magnitudes of the excitations from the
        % effort minimization solution suggest that these signals should be
        % rescaled. Use addScaleFactor() to add a MocoParameter to the problem that
        % will scale the reference data for the muscles in the tracking cost.
        %     tracking.addScaleFactor('gastroc_factor', '/forceset/gasmed_l', [0.01 1.0]);
        %     tracking.addScaleFactor('tibant_factor', '/forceset/tibant_l', [0.01 1.0]);
        %     tracking.addScaleFactor('bifem_factor', '/forceset/bfsh_l', [0.01 1.0]);
        %     tracking.addScaleFactor('gluteus_factor', '/forceset/glmax2_l', [0.01 1.0]);

        % Part 3d: Add the tracking goal to the problem.
        problem.addGoal(emgTracking)

        % Part 3e: Update the MocoCasADiSolver with the updated MocoProblem using
        % resetProblem(). might not need these lines here
        %     solver = MocoCasADiSolver.safeDownCast(study.updSolver());
        %     solver.resetProblem(problem);

        % Part 3f: Tell MocoCasADiSolver that the MocoParameters we added to the
        % problem via addScaleFactor() above do not require initSystem() calls on
        % the model. This provides a large speed-up.
        %     solver.set_parameters_require_initsystem(false);
    elseif settings(5) == 1
        refEMGTable = TimeSeriesTable(fullfile(dataDirectory, 'OpenSim_Sub08_EMGBaseline_EMG.mot'));

        % Create new path names for tracking table
        emgControlPaths = StdVectorString();
        emgControlPaths.add('/forceset/tibant_l');
        emgControlPaths.add('/forceset/gasmed_l');
        emgControlPaths.add('/forceset/vaslat_l');
        emgControlPaths.add('/forceset/semiten_l');
        emgControlPaths.add('/forceset/tibant_r');
        emgControlPaths.add('/forceset/gasmed_r');
        emgControlPaths.add('/forceset/vaslat_r');
        emgControlPaths.add('/forceset/semiten_r');
        refEMGTable.setColumnLabels(emgControlPaths);

        % Convert to struct from time series table object
        emgStruct = osimTableToStruct(refEMGTable);

        % Do linear envelop filter and normalize signals for tracking
        time = emgStruct.time;
        samplingFrequency = 1/(time(2) - time(1));
        scale = 0.75;
        filterOrder = 2;
        bandpassLowFreq = 10;
        bandpassHighFreq = 400;
        lowpassSmoothFreq = 5;
        fields = fieldnames(emgStruct);
        fields(strcmp(fields,'time')) = [];
        for i = 1:length(fields)
            emgStruct.(fields{i}) = ProcessEMGLinearEnvelope(emgStruct.(fields{i}), time, samplingFrequency, filterOrder, bandpassLowFreq, bandpassHighFreq, lowpassSmoothFreq, startMOCO, endMOCO, scale);
        end

        % Account for any time delay here we would like to correct for
        h = 0;
        emgStruct.time = emgStruct.time - h;

        % Convert back to times series table object
        emgTimeSeriesTable = osimTableFromStruct(emgStruct);

        % Add Labels
        emgTimeSeriesTable.setColumnLabels(emgControlPaths);

        % Write .mot for posthoc checking after filtering
        emgTimeSeriesTable.addTableMetaDataString('DataRate',string(samplingFrequency))
        emgTimeSeriesTable.addTableMetaDataString('Units','m')
        STOFileAdapter().write(emgTimeSeriesTable, fullfile(dataDirectory, 'TrackedEMGProfiles.sto'))

        % Set the reference for the tracking goal
        emgTracking.setReference(TableProcessor(emgTimeSeriesTable));
        %         emgTracking.setReferenceLabel('/forceset/tibant_l', 'tibialis_anterior_l');
        %         emgTracking.setReferenceLabel('/forceset/gasmed_l', 'gastrocnemius_l');
        %         emgTracking.setReferenceLabel('/forceset/vaslat_l', 'vastus_lateralus_l');
        %         emgTracking.setReferenceLabel('/forceset/semiten_l', 'semitendenosous_l');
        %         emgTracking.setReferenceLabel('/forceset/tibant_r', 'tibialis_anterior_r');
        %         emgTracking.setReferenceLabel('/forceset/gasmed_r', 'gastrocnemius_r');
        %         emgTracking.setReferenceLabel('/forceset/vaslat_r', 'vastus_lateralus_r');
        %         emgTracking.setReferenceLabel('/forceset/semiten_r', 'semitendenosous_r');

        % Part 3c: The EMG signals in the tracking are all normalized to have
        % a maximum value of 1, but the magnitudes of the excitations from the
        % effort minimization solution suggest that these signals should be
        % rescaled. Use addScaleFactor() to add a MocoParameter to the problem that
        % will scale the reference data for the muscles in the tracking cost.
        %         emgTracking.addScaleFactor('tibant_l_factor', '/forceset/tibant_l', [0.01 1.0]);
        %         emgTracking.addScaleFactor('gastroc_l_factor', '/forceset/gasmed_l', [0.01 1.0]);
        %         emgTracking.addScaleFactor('vaslat_l_factor', '/forceset/vaslat_l', [0.01 1.0]);
        %         emgTracking.addScaleFactor('semiten_l_factor', '/forceset/semiten_l', [0.01 1.0]);
        %         emgTracking.addScaleFactor('tibant_r_factor', '/forceset/tibant_r', [0.01 1.0]);
        %         emgTracking.addScaleFactor('gastroc_r_factor', '/forceset/gasmed_r', [0.01 1.0]);
        %         emgTracking.addScaleFactor('vaslat_r_factor', '/forceset/vaslat_r', [0.01 1.0]);
        %         emgTracking.addScaleFactor('semiten_r_factor', '/forceset/semiten_r', [0.01 1.0]);

        % Part 3d: Add the tracking goal to the problem.
        problem.addGoal(emgTracking)

        % Part 3e: Update the MocoCasADiSolver with the updated MocoProblem using
        % resetProblem(). might not need these lines
        %         solver = MocoCasADiSolver.safeDownCast(study.updSolver());
        %         solver.resetProblem(problem);

        % Part 3f: Tell MocoCasADiSolver that the MocoParameters we added to the
        % problem via addScaleFactor() above do not require initSystem() calls on
        % the model. This provides a large speed-up.
        %         solver.set_parameters_require_initsystem(false);
    end
end

% Track CS data if toggled on
if settings(4) == 1 || settings(4) == 2
    % Downcast the plugin goal and add table and use propertyhelper
    % Property helper example:
    % prop = obj.getPropertyByName("propertyName")
    % currentValue = PropertyHelper.getValueDouble(prop)
    % PropertyHelper.setValueDouble(newValue, prop)

    if settings(2) == 0 || settings(2) == 2
        stateTrackingGoalRef = MocoStateTrackingGoal.safeDownCast(problem.updGoal("state_tracking"));
        referenceProp =  stateTrackingGoalRef.updPropertyByName('reference');
         if (settings(2) == 0)
            referenceProp.setValueAsObject(TableProcessor(outputIKFilteredFilename));
        elseif (settings(2) == 2)
            referenceProp.setValueAsObject(TableProcessor(fullfile(dataDirectory, ['ik_', extractBefore(orientationFile,'.sto'), '.mot'])));
        end
    end

    plugin_csTracking = MocoGoal.safeDownCast( problem.updGoal('cs_tracking') );
    weightProp =  plugin_csTracking.updPropertyByName('weight');
    PropertyHelper.setValueDouble(1e6, weightProp);
    if settings(5) == 0
        refFLTable = fullfile(dataDirectory, 'MarkerMocapFLsDeGroote.mot');
    elseif settings(5) == 1
        refFLTable = fullfile(dataDirectory, 'CSPredictedFLsDeGroote.mot');
    end
    referenceProp = plugin_csTracking.updPropertyByName('reference');
    referenceProp.setValueAsObject(TableProcessor(refFLTable));
%     PropertyHelper.setValueString(refFLTable,referenceProp);

    % Debug with plugin of statetracking goal
    %     plugin_stateTracking = MocoGoal.safeDownCast( problem.updGoal('state_tracking_plugin') );
    %     weightProp =  plugin_stateTracking.getPropertyByName('weight');
    %     PropertyHelper.setValueDouble(1000, weightProp);
    %     referenceProp = plugin_stateTracking.getPropertyByName('reference');
    %     referenceProp.setValueAsObject(TableProcessor(outputIKFilteredFilename));
end

% Set output location, solve, and visualize (check how it write the moco
% solution [https://opensim-org.github.io/opensim-moco-site/docs/1.1.0/html_user/classOpenSim_1_1MocoStudy.html])
study.set_results_directory(dataDirectory)
study.set_write_solution(1);

% Save setup file as .omoco
study.print(fullfile(dataDirectory, 'MOCOTrackingProblemSetup.omoco'));

% Solve TrajOpt Problem
mocoOutputSolutionFile = fullfile(dataDirectory, ['MOCOoutput_', num2str(startMOCO), 'to', num2str(endMOCO), 'secs_', num2str(knots), 'knots.sto']);
% save(fullfile(outputDirectory,'Presimulation_Workspace.mat'))
solution = study.solve();
solution.unseal();
solution.write(mocoOutputSolutionFile);

% Create tables and save tables as opensim files if you want here
% outputs = StdVectorString();
% outputs.add('.*active_force_length_multiplier');
% table = study.analyze(solution, outputs);
% STOFileAdapter().write(table,fullfile(outputDirectory,'muscle_activations.sto'))

% https://simtk.org/plugins/phpBB/viewtopicPhpbb.php?f=1815&t=15032&p=0&start=0&view=&sid=b7c46f4c22b4299a3e7fb84faa72dd76
% Write solution's GRF to a file for comparison with ground truth
if settings(1) == 1 % If using contact model
    contact_r = StdVectorString();
    contact_l = StdVectorString();
    contact_r.add('/forceset/contactHeel_r');
    contact_r.add('/forceset/contactLateralRearfoot_r');
    contact_r.add('/forceset/contactLateralMidfoot_r');
    contact_r.add('/forceset/contactLateralToe_r');
    contact_r.add('/forceset/contactMedialToe_r');
    contact_r.add('/forceset/contactMedialMidfoot_r');
    contact_l.add('/forceset/contactHeel_l');
    contact_l.add('/forceset/contactLateralRearfoot_l');
    contact_l.add('/forceset/contactLateralMidfoot_l');
    contact_l.add('/forceset/contactLateralToe_l');
    contact_l.add('/forceset/contactMedialToe_l');
    contact_l.add('/forceset/contactMedialMidfoot_l');
    externalForcesTableFlat = opensimMoco.createExternalLoadsTableForGait(baseModel,solution,contact_r,contact_l);
    STOFileAdapter.write(externalForcesTableFlat,fullfile(dataDirectory,'MOCOpredictedGRF.mot'));
end

% diary('off')

study.visualize(solution); % Can save frames here for packaging into movie below

%% Appendix A. Generate MOCO Report Using Utility Function for Easy PDF Reports
doGenerateReport = 0;
if(doGenerateReport)
    import org.opensim.modeling.*;
    osimModel = Model(trajOptModelfile);
    % trajReport = osimMocoTrajectoryReport(osimModel, mocoOutputSolutionFile); % must use external converter
    trajReport = osimMocoTrajectoryReport(osimModel, mocoOutputSolutionFile,  'outputFilePath', fullfile(dataDirectory, 'MOCOReport.pdf'), 'bilateral', true);
    reportFilepath = trajReport.generate();
end

%% Appendix B. Create Movie From Images For Rapid Sharing without GUI
doMovie = 0;
if doMovie == 1
    selpath = uigetdir('Choose image folder');
    if isequal(selpath, 0)
        return;
    end
    imgFilePath = selpath;
    filename_out = fullfile(pwd, 'Movie.avi');
    % "hgate00.png" to "hgate79.png"
    startnum = 1;
    endnum = 30;
    % create video file
    writerObj = VideoWriter(filename_out, 'Uncompressed AVI');
    writerObj.FrameRate = 30;
    open(writerObj);
    h = waitbar(0, '', 'Name', 'Write Video File...');
    steps = endnum - startnum;
    for num = startnum : endnum
        if num < 100
            file = sprintf('Frame00%02d.png', num);
        elseif num >= 100
            file = sprintf('Frame0%02d.png', num);
        end
        file = fullfile(imgFilePath, file);
        frame = imread(file);
        frame = im2frame(frame);
        writeVideo(writerObj,frame);
        pause(0.01);
        step = num - startnum;
        waitbar(step/steps, h, sprintf('Process：%d%%', round(step/steps*100)));
    end
    close(writerObj);
    close(h);
end

%% Appendix C. Plotting Kinematics Tracking
import org.opensim.modeling.*;
% mocoPlotTrajectory(mocoOutputSolutionFile);
solntraj = MocoTrajectory(mocoOutputSolutionFile);
valuesOsimMat = solntraj.getValuesTrajectory();
valueNamesOsim = solntraj.getValueNames();
valuesMatrix = valuesOsimMat.getAsMat();
timesMatrix = solntraj.getTimeMat();

ikFiltData = ReadOpenSimData(outputIKFilteredFilename);

fig = figure;
plot(timesMatrix, valuesMatrix(:,[7, 13, 17]))
hold on
plot(ikFiltData.time, ikFiltData.data(:,[7,10,12]) * pi/180)
legend('rhip','rknee','rankle', 'rhip-IK','rknee-IK','rankle-IK')
ylabel('rad')
xlabel('sec')
set(gca, 'box','off')
legend boxoff
hold off

saveas(fig, fullfile(dataDirectory, 'MOCOKinematicsOutput.fig'));

%% Appendix D. Plotting Accelerometer Tracking
if settings(2) == 1 || settings(2) == 2
    import org.opensim.modeling.*;
    trackingSolution = MocoTrajectory(mocoOutputSolutionFile);

    outputPaths = StdVectorString();
    outputPaths.add('.*accelerometer_signal');
    accelerometerSignalsTracking = opensimSimulation.analyzeVec3(modelobj, ...
        trackingSolution.exportToStatesTable(), ...
        trackingSolution.exportToControlsTable(), ...
        outputPaths);
    accelerometerSignalsTracking.setColumnLabels(imuFramePaths);
    fig1 = plotAccelerationSignals(accelTimeSeriesTableVec3, accelerometerSignalsTracking);

    outputPaths = StdVectorString();
    outputPaths.add('.*gyroscope_signal');
    gyroscopeSignalsTracking = opensimSimulation.analyzeVec3(modelobj, ...
        trackingSolution.exportToStatesTable(), ...
        trackingSolution.exportToControlsTable(), ...
        outputPaths);
    gyroscopeSignalsTracking.setColumnLabels(imuFramePaths);
    fig2 = plotAccelerationSignals(gyroTimeSeriesTableVec3, gyroscopeSignalsTracking);

    saveas(fig1, fullfile(dataDirectory, 'Accels.fig'));
    saveas(fig2, fullfile(dataDirectory, 'Angvels.fig'));
end

%% Appendix E. Calculate RMSEs from MOCO tracking versus input IK
doRMSEs = 1;
if doRMSEs == 1
    RMSEMocoStruct = calculateRMSEsfromReferenceKinematics(outputIKFilteredFilename, mocoOutputSolutionFile, startMOCO, endMOCO)

    figure
    grabInds = [4 10 14];
    X = 1:length(grabInds);
    Y = [RMSEMocoStruct.KINrmse(grabInds)']
    h = bar(X,Y);
    xticklabels({'Hip Flexion','Knee Flexion','Ankle Flexion'})
    set(gca, 'box', 'off')
    ylabel('RMSE (deg)')
    h(1).FaceColor = [0 0 .7];
    ylim([0 20])
    title('Kinematics Error')
    
    save(fullfile(dataDirectory,'RMSEMocoStruct.mat'), 'RMSEMocoStruct')
end
%% Appendix F. Plotting Fiber Length Tracking
doFLTrackCheck = 1;
if(doFLTrackCheck)
    muscString = {'vaslat_l', 'semiten_l',  'gasmed_l', 'tibant_l'};
    for m = 1:length(muscString)
        import org.opensim.modeling.*;
        
        refFLTable = fullfile(dataDirectory, 'MarkerMocapFLsDeGroote.mot');
        flData = ReadOpenSimData(refFLTable);

        mocoOutputSolutionFile = fullfile(dataDirectory, ['MOCOoutput_', num2str(startMOCO), 'to', num2str(endMOCO), 'secs_', num2str(knots), 'knots.sto']);
        solntraj = MocoTrajectory(mocoOutputSolutionFile);
        valuesOsimMat = solntraj.getValuesTrajectory();
        valueNamesOsim = solntraj.getValueNames();  
        valuesMatrix = valuesOsimMat.getAsMat();
        timesMatrix = solntraj.getTimeMat();

        baseModel = Model(trajOptModelfile);
        state =  baseModel.initSystem(); updateVec = state.getQ();
        numQ = state.getQ().size();
        muscle = baseModel.getMuscles().get(muscString{m});

        numFrames = length(timesMatrix);
        fl = zeros(1,numFrames);
    
        tic
        for i = 1:numFrames
            clc
            fprintf('Processing frame %d out of %d...', i, numFrames)
            for j = 1:numQ
                currCoordName = baseModel.updCoordinateSet().get(j-1).getName().toCharArray()';

                for k = 1:valueNamesOsim.size()
                    if contains(valueNamesOsim.get(k-1).toCharArray()', currCoordName)
                      % if contains(valueNamesOsim.get(k-1), currCoordName)
                        break
                    end
                end
                
                baseModel.updCoordinateSet().get(j-1).setValue(state, valuesMatrix(i,k))
            end

            % Update model with kinematics and get muscles
            baseModel.realizePosition(state);

            % Get the muscle states
            fl(i) = muscle.getLength(state);
        end
        toc

        fig = figure;
        plot(timesMatrix, fl)
        hold on
        plot(flData.time, flData.data(:, find(contains(flData.labels,muscString{m}))) )
        legend('MOCO','Ground Truth')
        ylabel('fiber length (m)')
        xlabel('time (s)')
        set(gca, 'box','off')
        legend boxoff
        title(muscString{m})
        hold off
        saveas(fig, fullfile(dataDirectory, [muscString{m},'.fig']));
    end
end

%% Appendix G. Internal Utility Functions
function addIMUFrame(model, bodyName, translation, orientation)
    import org.opensim.modeling.*;
    body = model.updBodySet().get(bodyName);
    name = [char(body.getName()) '_imu_offset'];
    bodyOffset = PhysicalOffsetFrame(name, body, Transform());
    bodyOffset.set_translation(translation);
    bodyOffset.set_orientation(orientation);
    body.addComponent(bodyOffset);
    model.finalizeConnections();
end

function [RMSEStruct] = calculateRMSEsfromReferenceKinematics(refIKFile, mocoSolnFile, startTime, endTime)
    import org.opensim.modeling.*;
    ikRefData = ReadOpenSimData(refIKFile);
    solnTraj = MocoTrajectory(mocoSolnFile);
    valuesOsimMat = solnTraj.getValuesTrajectory();
    valueNamesOsim = solnTraj.getValueNames();
    numDOF = valueNamesOsim.capacity();
    valuesMatrix = valuesOsimMat.getAsMat();
    timesMatrix = solnTraj.getTimeMat();
    
    % Get traj labels as cell array of strings
    trajLabels = cell(1,numDOF);
    for i = 1:numDOF
        trajLabels{i} = valueNamesOsim.get(i-1).toCharArray()';
    %     trajLabels{i} = valueNamesOsim.get(i-1);
    end
    
    % Get the start and end indices for columns in ref and traj time...
    startRefInd = find(round(ikRefData.time,4) == startTime);
    endRefInd = find(round(ikRefData.time,4) == endTime);
    startTrajInd = find(round(timesMatrix,4) == startTime);
    endTrajInd = find(round(timesMatrix,4) == endTime);
    
    % Calculate the RMSE of the errors for the
    COM_Position_RMSE = zeros(1, 3);
    COM_Position_Labels = cell(1,3);
    COM_translation_prompt = {'tx','ty','tz'};
    for i = 1:length(COM_translation_prompt)
        currRefIndex = contains(ikRefData.labels, COM_translation_prompt{i});
        currTrajIndex = contains(trajLabels, COM_translation_prompt{i});
    %         COM_Position_RMSE(i) = rms( interp1(ikRefData.time(startRefInd:endRefInd), ikRefData.data(startRefInd:endRefInd, currRefIndex), timesMatrix(startTrajInd:endTrajInd), 'spline', 'extrap') - valuesMatrix(startTrajInd:endTrajInd, currTrajIndex));
        adjustedStateCOMVal = valuesMatrix(startTrajInd:endTrajInd, currTrajIndex) +  ( ikRefData.data(startRefInd, currRefIndex) - valuesMatrix(startTrajInd, currTrajIndex) );
        COM_Position_RMSE(i) = rms( interp1(ikRefData.time(startRefInd:endRefInd), ikRefData.data(startRefInd:endRefInd, currRefIndex), timesMatrix(startTrajInd:endTrajInd), 'spline', 'extrap') - adjustedStateCOMVal );
        COM_Position_Labels{i} = trajLabels{contains(trajLabels, COM_translation_prompt{i})};
    
        titleString = ikRefData.labels{currRefIndex};
        titleString = extractAfter(titleString, '/');
        titleString = extractAfter(titleString, '/');
        titleString = extractAfter(titleString, '/');
        titleString = extractBefore(titleString, '/');
    
    %     figure
    %     plot(timesMatrix(startTrajInd:endTrajInd), interp1(ikRefData.time(startRefInd:endRefInd), ikRefData.data(startRefInd:endRefInd, currRefIndex), timesMatrix(startTrajInd:endTrajInd), 'spline', 'extrap') - adjustedStateCOMVal)
    %     title(titleString)
    %     xlabel('Time (s)')
    %     ylabel('Position (m)')
    %     set(gca,'Box','off')
    %     hold off
    % 
    %     figure
    %     plot(timesMatrix(startTrajInd:endTrajInd), interp1(ikRefData.time(startRefInd:endRefInd), ikRefData.data(startRefInd:endRefInd, currRefIndex), timesMatrix(startTrajInd:endTrajInd), 'spline', 'extrap'))
    %     hold on
    %     plot(timesMatrix(startTrajInd:endTrajInd), adjustedStateCOMVal)
    %     title(titleString)
    %     xlabel('Time (s)')
    %     ylabel('Position (m)')
    %     set(gca,'Box','off')
    %     savefig(gcf, fullfile(resultsDirectory, [titleString, '_MocoTrackingResults.fig']))
    %     hold off
    end
    
    Kinematics_RMSEs = zeros(1, numDOF-3);
    Kinematics_Labels = cell(1, numDOF-3);
    nonComInds = find(~contains(trajLabels, COM_translation_prompt));
    for i = 1:length(nonComInds)
        currRefIndex = contains(ikRefData.labels, trajLabels{nonComInds(i)});
        currTrajIndex = contains(trajLabels, trajLabels{nonComInds(i)});
        Kinematics_RMSEs(i) = rms( interp1(ikRefData.time(startRefInd:endRefInd), ikRefData.data(startRefInd:endRefInd, currRefIndex), timesMatrix(startTrajInd:endTrajInd), 'spline', 'extrap') - (valuesMatrix(startTrajInd:endTrajInd, currTrajIndex) * 180/pi) );
        Kinematics_Labels{i} = trajLabels{nonComInds(i)};
    
        titleString = ikRefData.labels{currRefIndex};
        titleString = extractAfter(titleString, '/');
        titleString = extractAfter(titleString, '/');
        titleString = extractAfter(titleString, '/');
        % titleString = extractBefore(titleString, '/');
    
        % [13,16,18]
        if( any(find(currRefIndex) == [7,10,12]) )
            cycle = linspace(0,100,length(timesMatrix(startTrajInd:endTrajInd)));
    
            figure
            plot(cycle, interp1(ikRefData.time(startRefInd:endRefInd), ikRefData.data(startRefInd:endRefInd, currRefIndex), timesMatrix(startTrajInd:endTrajInd), 'spline', 'extrap'))
            hold on
            plot(cycle, (valuesMatrix(startTrajInd:endTrajInd, currTrajIndex) * 180/pi))
            title(titleString)
            xlabel('Time (s)')
            ylabel('Angle (deg)')
            set(gca,'Box','off')
            % savefig(gcf, fullfile(resultsDirectory, [titleString, '_MocoTrackingResults.fig']))
            hold off
        end
    end
    
    % Package
    RMSEStruct.COMrmse = COM_Position_RMSE;
    RMSEStruct.COMlabels = COM_Position_Labels;
    RMSEStruct.KINrmse = Kinematics_RMSEs;
    RMSEStruct.KINlabels = Kinematics_Labels;
end


function [RMSEStruct] = calculateOpenSenseIKRMSEsfromReferenceKinematics(refIK,openSenseIK, startTime, endTime)
    import org.opensim.modeling.*;
    
    ikRefData = ReadOpenSimData(fullfile(refIK));
    ikTrackData = ReadOpenSimData(fullfile(openSenseIK));
    
    % Get same values corresponding to ikTrackDataStruct
    trajLabels = ikTrackData.labels;
    numDOF = length(trajLabels);
    timesMatrix = ikTrackData.time;
    valuesMatrix = ikTrackData.data;
    
    % Get the start and end indices for columns in ref and traj time...
    startRefInd = find(ikRefData.time == startTime);
    endRefInd = find(ikRefData.time == endTime);
    startTrajInd = find(timesMatrix == startTime);
    endTrajInd = find(timesMatrix == endTime);
    
    % Calculate the RMSE of the errors for the
    COM_Position_RMSE = zeros(1, 3);
    COM_Position_Labels = cell(1,3);
    COM_translation_prompt = {'tx','ty','tz'};
    for i = 1:length(COM_translation_prompt)
        currRefIndex = contains(ikRefData.labels, COM_translation_prompt{i});
        currTrajIndex = contains(trajLabels, COM_translation_prompt{i});
        %     COM_Position_RMSE(i) = rms( interp1(ikRefData.time(startRefInd:endRefInd), ikRefData.data(startRefInd:endRefInd, currRefIndex), timesMatrix(startTrajInd:endTrajInd), 'spline', 'extrap') - valuesMatrix(startTrajInd:endTrajInd, currTrajIndex));
        adjustedStateCOMVal = valuesMatrix(startTrajInd:endTrajInd, currTrajIndex) +  ( ikRefData.data(startRefInd, currRefIndex) - valuesMatrix(startTrajInd, currTrajIndex) );
        COM_Position_RMSE(i) = rms( interp1(ikRefData.time(startRefInd:endRefInd), ikRefData.data(startRefInd:endRefInd, currRefIndex), timesMatrix(startTrajInd:endTrajInd), 'spline', 'extrap') - adjustedStateCOMVal );
        COM_Position_Labels{i} = trajLabels{contains(trajLabels, COM_translation_prompt{i})};
    
        titleString = ikRefData.labels{currRefIndex};
        titleString = extractAfter(titleString, '/');
        titleString = extractAfter(titleString, '/');
        titleString = extractAfter(titleString, '/');
        titleString = extractBefore(titleString, '/');
    
    %     figure
    %     plot(timesMatrix(startTrajInd:endTrajInd), interp1(ikRefData.time(startRefInd:endRefInd), ikRefData.data(startRefInd:endRefInd, currRefIndex), timesMatrix(startTrajInd:endTrajInd), 'spline', 'extrap') - adjustedStateCOMVal)
    %     title(titleString)
    %     xlabel('Time (s)')
    %     ylabel('Position (m)')
    %     set(gca,'Box','off')
    %     hold off
    % 
    %     figure
    %     plot(timesMatrix(startTrajInd:endTrajInd), interp1(ikRefData.time(startRefInd:endRefInd), ikRefData.data(startRefInd:endRefInd, currRefIndex), timesMatrix(startTrajInd:endTrajInd), 'spline', 'extrap'))
    %     hold on
    %     plot(timesMatrix(startTrajInd:endTrajInd), adjustedStateCOMVal)
    %     title(titleString)
    %     xlabel('Time (s)')
    %     ylabel('Position (m)')
    %     set(gca,'Box','off')
    %     savefig(gcf, fullfile(resultsDirectory, [titleString, '_IMUOnlyPrediction.fig']))
    %     hold off
    end
    
    Kinematics_RMSEs = zeros(1, numDOF-3);
    Kinematics_Labels = cell(1, numDOF-3);
    nonComInds = find(~contains(trajLabels, COM_translation_prompt));
    for i = 1:length(nonComInds)
        currRefIndex = contains(ikRefData.labels, trajLabels{nonComInds(i)});
        if(length(find(currRefIndex)) > 1)
            currRefIndex = find(currRefIndex);
            currRefIndex = currRefIndex(1);
        end
        currTrajIndex = contains(trajLabels, trajLabels{nonComInds(i)});
        if(length(find(currTrajIndex)) > 1)
            currTrajIndex = find(currTrajIndex);
            currTrajIndex = currTrajIndex(1);
        end
        Kinematics_RMSEs(i) = rms( interp1(ikRefData.time(startRefInd:endRefInd), ikRefData.data(startRefInd:endRefInd, currRefIndex), timesMatrix(startTrajInd:endTrajInd), 'spline', 'extrap') - (valuesMatrix(startTrajInd:endTrajInd, currTrajIndex)) );
        Kinematics_Labels{i} = trajLabels{nonComInds(i)};
    
        titleString = ikRefData.labels{currRefIndex};
        titleString = extractAfter(titleString, '/');
        titleString = extractAfter(titleString, '/');
        titleString = extractAfter(titleString, '/');
        titleString = extractBefore(titleString, '/');
    
        cycle = linspace(0,100,length(timesMatrix(startTrajInd:endTrajInd)));

        if( any(find(currRefIndex) == [7,10,12]) )
            figure
            plot(cycle, interp1(ikRefData.time(startRefInd:endRefInd), ikRefData.data(startRefInd:endRefInd, currRefIndex), timesMatrix(startTrajInd:endTrajInd), 'spline', 'extrap'))
            hold on
            plot(cycle, smooth((valuesMatrix(startTrajInd:endTrajInd, currTrajIndex))))
            title(titleString)
            xlabel('Time (s)')
            ylabel('Angle (deg)')
            set(gca,'Box','off')
            legend('Marker-based', 'IMU-Only')
            legend boxoff
            savefig(gcf, fullfile(fileparts(refIK), [titleString, '_IMUOnlyPrediction.fig']))
            hold off
        end
    end
    
    % Package
    RMSEStruct.COMrmse = COM_Position_RMSE;
    RMSEStruct.COMlabels = COM_Position_Labels;
    RMSEStruct.KINrmse = Kinematics_RMSEs;
    RMSEStruct.KINlabels = Kinematics_Labels;
end


function [fig] = plotAccelerationSignals(varargin)

import org.opensim.modeling.*;

accelerationsReference = varargin{1};
if nargin == 2
    accelerationsTracking = varargin{2};
end

% Create a time vector that can be used for plotting
timeVec = accelerationsReference.getIndependentColumn();
time = zeros(timeVec.size(),1);
for i = 1:timeVec.size()
    time(i) = timeVec.get(i-1);
end

fig = figure;
% Plot the torso accelerations
subplot(1,3,1)
torso = accelerationsReference.getDependentColumn(...
    '/bodyset/torso/torso_imu_offset').getAsMat();
plot(time, torso(:,1), 'r-', 'linewidth', 3)
if nargin == 2

    timeTrackVec = accelerationsTracking.getIndependentColumn();
    timeTrack = zeros(timeTrackVec.size(),1);
    for i = 1:timeTrackVec.size()
        timeTrack(i) = timeTrackVec.get(i-1);
    end

    hold on
    torsoTrack = accelerationsTracking.getDependentColumn(...
        '/bodyset/torso/torso_imu_offset').getAsMat();
    plot(timeTrack, torsoTrack(:,1), 'b--', 'linewidth', 3)
end
if nargin == 2
    legend('predict', 'track', 'location', 'best');
end
title('torso')
xlabel('time (s)')
ylabel('acceleration (m/s^2)')

% Plot the femur accelerations
subplot(1,3,2)
femur = accelerationsReference.getDependentColumn(...
    '/bodyset/femur_r/femur_r_imu_offset').getAsMat();
plot(time, femur(:,1), 'r-', 'linewidth', 3)
if nargin == 2
    hold on
    femurTrack = accelerationsTracking.getDependentColumn(...
        '/bodyset/femur_r/femur_r_imu_offset').getAsMat();
    plot(timeTrack, femurTrack(:,1), 'b--', 'linewidth', 3)
end
title('femur')
xlabel('time (s)')
ylabel('acceleration (m/s^2)')

% Plot the tibia accelerations
subplot(1,3,3)
tibia = accelerationsReference.getDependentColumn(...
    '/bodyset/tibia_r/tibia_r_imu_offset').getAsMat();
plot(time, tibia(:,1), 'r-', 'linewidth', 3)
if nargin == 2
    hold on
    tibiaTrack = accelerationsTracking.getDependentColumn(...
        '/bodyset/tibia_r/tibia_r_imu_offset').getAsMat();
    plot(timeTrack, tibiaTrack(:,1), 'b--', 'linewidth', 3)
end
title('tibia')
xlabel('time (s)')
ylabel('acceleration (m/s^2)')

end