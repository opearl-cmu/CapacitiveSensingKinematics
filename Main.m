%% CS-Predicted Muscle Length Inverse Kinematics
% Author: Owen Douglas Pearl | Carnegie Mellon University
% Musculoskeletal Biomechanics Lab & Micro Robotics Lab

% This script shows an example of using a pretrained LSTM to predict
% normalized muscle-tendon-unit lengths with a capacitive sensing shank
% sleeve before unnormalizing the MTU lengths and finding the
% matching ankle angle with an opensim model. As is the case with the
% provided example opensim model, the model must use a scaled OpenSim model
% for the test subject with DeGroote muscle models, since the LSTM training
% data contained shank CS data and matching DeGroote muscle MTU lengths.

% When using this work, please cite the following literature:
% Pearl, O., Rokhmanova, N., Dankovich, L., Faille, S., Bergbreiter, S., & Halilaj, E. (2022). 
% Capacitive Sensing for Natural Environment Rehabilitation Monitoring [Preprint]. 
% In Review. https://doi.org/10.21203/rs.3.rs-1902381/v1

%% Main
clear all; close all; clc
load('Data/setup.mat')

% Forward pass with network for prediction 
% (comment out the next two lines if you do not have DeepLearningToolbox):
% MTUTestSubject = predict(net, CSTestSubject);
% MTUTestSubject = double(MTUTestSubject{1});

% Visualize capacitive sensing data
figure
plot(CSTestSubject{1},'b')
set(gca, 'box', 'off')
ylabel('Capacitance (Normalized)')
xlabel('Samples')
xlim([0 2000])

% Visualize predicted and ground truth MTU lengths
figure
plot(MTUTestSubjectGroundTruth','k')
hold on
plot(MTUTestSubject','r')
set(gca, 'box', 'off')
ylabel('Muscle Lengths (Normalized)')
xlabel('Samples')
xlim([0 2000])
hold off

% Setup parameters
osimModelFilename = 'Data/TestSubjectModel.osim';
testSubjectKinematics = ReadOpenSimData('Data/TestSubjectKinematics.mot');
inds = round(linspace(1300,1470,30)); % set start, end, and number of discretized knot points

% Do trimming and grab knot points
fullInds = inds(1):inds(end); 
time = testSubjectKinematics.time(find(round(testSubjectKinematics.time,2) == round(testTime(1),2)):find(round(testSubjectKinematics.time,2) == round(testTime(end),2))); 
timeFull = time(fullInds); time = time(inds);
GTAnkleKins = testSubjectKinematics.data(find(round(testSubjectKinematics.time,2) == round(testTime(1),2)):find(round(testSubjectKinematics.time,2) == round(testTime(end),2)),18)*pi/180; 
GTAnkleKinsFull = GTAnkleKins(fullInds,:); GTAnkleKins = GTAnkleKins(inds,:);
PredictedNormMTUs = MTUTestSubject(:,inds)';

% Imports
import org.opensim.modeling.*;
myMatlabLog = JavaLogSink();
Logger.addSink(myMatlabLog);
muscleStrings = {'tib_ant_l', 'tib_post_l', 'med_gas_l', 'soleus_l'};
osimModel = Model(osimModelFilename);
state =  osimModel.initSystem(); 
clc;

% Set initial guess (mess it up drastically to challenge the optimizer)
InitGuess = zeros(length(GTAnkleKins)+4,1);
rng(3); randPercentMagnitudeAsDecimal = .7; 
InitGuess(1:length(GTAnkleKins)) = GTAnkleKins+GTAnkleKins.*randPercentMagnitudeAsDecimal.*randn(length(GTAnkleKins),1); 
InitGuess(end-3:end) = RefSubC;

% Setup fmincon options 
% (important to use a large enough FD step size to avoid local minima)
optfunc = @(predict) MuscleLengthForwardKinematicsLoss(predict, time, PredictedNormMTUs, osimModel, state, muscleStrings, RefSubS);
options = optimoptions('fmincon', 'Display', 'iter', 'MaxIter', 1000, 'PlotFcns','optimplotfval', 'FiniteDifferenceStepSize', 1e-4);

% Run fmincon while timing runtime
tstart = tic;
[optimal_predictions, optJ] = fmincon(optfunc, InitGuess, [], [], [], [], [], [], [], options);
RunTime = toc(tstart)

% Unpackage outcomes
optimalAnkle_knots = optimal_predictions(1:length(GTAnkleKins))*180/pi;
GTAnkle_knots = GTAnkleKins*180/pi;

% Do cubic spline for smoothing
x = linspace(0,100,length(GTAnkleKins));
xsmooth = linspace(0,100,1000);
xFull = linspace(0,100,length(timeFull));
initGuessAnkle = interp1(x, InitGuess(1:length(GTAnkleKins))*180/pi, xsmooth, 'cubic');
GTAnkle = interp1(xFull, GTAnkleKinsFull*180/pi, xsmooth, 'cubic');
optimalAnkle = interp1(x, optimalAnkle_knots, xsmooth, 'cubic');

% Plot Ankle Angle Prediction
figure
plot(xsmooth, initGuessAnkle, 'k--')
hold on
plot(xsmooth, GTAnkle, 'r')
plot(xsmooth, optimalAnkle, 'b')
set(gca, 'Box', 'off')
legend('Initial Guess', 'Ground Truth', 'CS-Predicted')
legend boxoff
xticklabels([]);
ylabel('Ankle Dorsiflexion (deg)')
legend boxoff

% Calculate Ankle Angle RMSE in degrees
AnkleRMSEDeg = rms(optimalAnkle - GTAnkle)

%% Forward rollout to evaluate tracking results on predicted angle
AnkleKins = optimal_predictions(1:length(time));
C = optimal_predictions(end-3:end)';
S = RefSubS;

% Load in model and prep state
muscle1 = osimModel.getMuscles().get(muscleStrings{1});
muscle2 = osimModel.getMuscles().get(muscleStrings{2});
muscle3 = osimModel.getMuscles().get(muscleStrings{3});
muscle4 = osimModel.getMuscles().get(muscleStrings{4});

numFrames = length(time);
fl = zeros(numFrames,4);

for i = 1:numFrames
    % Set the ankle value (much faster than useing updCoordinateSet)
    osimModel.setStateVariableValue(state,'/jointset/ankle_l/ankle_angle_l/value',AnkleKins(i)); 

    % Update model with kinematics and get muscles length
    osimModel.realizePosition(state);
   
    % Get the muscle tendon unit length
    fl(i,1) = muscle1.getLength(state);
    fl(i,2) = muscle2.getLength(state);
    fl(i,3) = muscle3.getLength(state);
    fl(i,4) = muscle4.getLength(state);
end

% Plot fls against ground truths to visualize the tracking
for j = 1:length(muscleStrings) 
    if j ~= 3 % does not track gastroc since that would require the knee angle too
        figure
        plot(fl(:,j))
        hold on
        plot((PredictedNormMTUs(:,j).*S(j) + C(j)))
        set(gca, 'Box', 'off')
        xticklabels([])
        ylabel('MTU Length (m)')
        legend('Model Muscle Length', 'Unnormalized CS-Predicted Muscle Length')
        legend boxoff
        title(erase(muscleStrings(j), '_'))
    end
end

%% Function for use in fmincon (forward loss with muscle lengths tracking)
function [J] = MuscleLengthForwardKinematicsLoss(predict, time, PredictedNormMTUs, osimModel, state, muscleString, RefSubS)

    % Unwrap Vars
    Kins = predict(1:end-4);
    C = predict(end-3:end)'; % C varies a lot between subjects, so we need to guess using a different subject and then optimize it
    S = RefSubS; % S is not optimized because it does not vary between subjects much at all, so it can simply be prescribed

    % Load in model and prep state
    muscle1 = osimModel.getMuscles().get(muscleString{1});
    muscle2 = osimModel.getMuscles().get(muscleString{2});
    muscle3 = osimModel.getMuscles().get(muscleString{3});
    muscle4 = osimModel.getMuscles().get(muscleString{4});

    numFrames = length(time);
    fl = zeros(numFrames,4);

    for i = 1:numFrames
        % Set the ankle value (much faster than useing updCoordinateSet)
        osimModel.setStateVariableValue(state,'/jointset/ankle_l/ankle_angle_l/value',Kins(i));

        % Update model with kinematics and get muscles length
        osimModel.realizePosition(state);
       
        % Get the muscle tendon unit length
        fl(i,1) = muscle1.getLength(state);
        fl(i,2) = muscle2.getLength(state);
        fl(i,3) = muscle3.getLength(state);
        fl(i,4) = muscle4.getLength(state);
    end
 
% Weight on tracking tibant, tibpost, medgas, and soleus (need to turn
% off the gastroc here since it depends on the knee as well)
w = [1e5 1e5 0 1e5]; 

% Cost Function
Cost = sum( w.*(fl - (PredictedNormMTUs.*S + C) ).^2, 2);
J = trapz(time, Cost);
end