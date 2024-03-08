function [outputStruct] = CheckErrorOpenSimIK(modelFile, trialDataFile,...
    IKSettingsFile, IKErrorParameters, startIK, endIK)
% CheckErrorOpenSimIK
%   This function runs the InverseKinematicSolver that powers OpenSim's
%   InverseKinematics Tool.  It calculates frame by frame errors and the
%   coordinate trajectories and stores them in the output struct.
% 
% Input:
%    modelFile: Name of the Placed model File from OpenSim Scaling 
%    trialDataFile: Name of the .trc file of Mocap Data
%    IKSettingsFile: Name of the IK settings file in the ProcessedData folder
%    IKErrorParameters: Parameter Struct in Study Parameters

% Output:
%    outputStruct: Output Structure of all the relevant data
%       .time = time;
%     	.labels = labels for the generalized coordinates
%       .data = data of generalized coordinates
%     	.numRows = num of rows in .data field
%       .numCols = num of columns in .data field
%       .inDegrees = flag to true for angles in degrees;
%       .errorLabels = labels for the errors (1 x #Cols);
%       .totalSquaredError = total squared error for IK (#Rows x #Cols)
%       .rmseError = rmse error for IK (#Rows x x 1)
%       .worstMarker = name of worst marker (#Rows x 1)
%
% Usage: [outputStruct] = CheckErrorOpenSimIK(modelFile, trialDataFile,...
%     IKSettingsFile, IKErrorParameters)
% ----------------------------------------------------------------------- 
% Authors: Daniel Jacobs, Owen Pearl
% Created: 2019
% ----------------------------------------------------------------------- 
    fprintf('ErrorCheck_IKOpenSimModel\n');
    % Import OpenSim Libraries
    import org.opensim.modeling.*;
    
    % Load Model
    model = Model(modelFile);
    state = model.initSystem();
    
    % ParseTrialName
    [~, trialName, ~] = fileparts(trialDataFile);
    
    % Get Marker Set from Model
    numModelMarkers = model.getNumMarkers();
    modelMarkerNames = cell(1,numModelMarkers);
    for iter = 1:numModelMarkers
        modelMarkerNames{1, iter} = model.getMarkerSet().get(iter-1).getName();
    end

    % Load IK Settings File
    ikTool = InverseKinematicsTool(IKSettingsFile);
    
    % Get Task Set
    ikTaskSet = ikTool.getIKTaskSet();
    numIKTasks = ikTaskSet.getSize();
    ikTaskSetNames = cell(1,numIKTasks);
    isCoordinateReference = false(1, numIKTasks);
    for iter = 1:numIKTasks  
        ikTaskSetNames{1, iter} = ikTaskSet.get(iter-1).getName();
        isCoordinateReference(1, iter) = strcmp(ikTaskSet.get(iter-1).getConcreteClassName(), 'IKCoordinateTask');
    end
    easierTaskNames = string(ikTaskSetNames);
 
    % Marker Ref
    markerRef = MarkersReference();
    
    % Create Weights Following Model Marker List
    markerWeightSet = SetMarkerWeights();
    numericalList = 1:numModelMarkers;
    for iter = 1:numModelMarkers
        taskLogical = strcmp(easierTaskNames, modelMarkerNames{1, iter});
        if(any(taskLogical(1:length(numericalList))))
            taskIndex = numericalList(taskLogical(1:length(numericalList)));
            aTask = ikTaskSet.get(taskIndex-1);
            if(aTask.getApply())
                markerWeight = MarkerWeight(aTask.getName(), aTask.getWeight); 
                markerWeightSet.cloneAndAppend(markerWeight);
            end
        else
            % Marker Exists - Define MarkerWeight but Set Apply to Zero
        end
    end
    markerRef.setMarkerWeightSet(markerWeightSet);

    % Create Coordinate Weights
    numericalList = 1:numIKTasks;
    coordinateTaskIndex = numericalList(isCoordinateReference);
    numCoordinateReferences = sum(isCoordinateReference);
    coordReferenceArray = SimTKArrayCoordinateReference();
    if(numCoordinateReferences ~= 0)
        for iter = 1:numCoordinateReferences
            aTask = ikTaskSet.get(coordinateTaskIndex(iter)-1);
            if(aTask.getApply())
                coordRef = CoordinateReference(aTask.getName() , Constant);
                coordRef.setWeight(aTask.getWeight);
                coordRef.markAdopted;
                coordReferenceArray.push_back(coordRef);
                clear coordRef;
            end
        end
    end

    % Load Data
%     markerRef.set_marker_file(trialDataFile)
    markerRef.initializeFromMarkersFile(trialDataFile, markerWeightSet)

    % Assume Model has some Marker Set, if not load from Tasks
    timeRange = markerRef.getValidTimeRange;
    startTime = timeRange.get(0);

    % Create Solver
    ikSolver = InverseKinematicsSolver(model,markerRef,coordReferenceArray,IKErrorParameters.constraintWeight);

    % Set ikSolver accuracy - from default or user input
    ikSolver.setAccuracy(IKErrorParameters.accuracy);

    % Assemble the model
    state.setTime(startIK);
    ikSolver.assemble(state);

    % Marker Order in Solver can differ from Model and Marker Ref
    % Reassemble names from solver
    numMarkersInUse = ikSolver.getNumMarkersInUse;
    finalWeightNames = cell(1, numModelMarkers);
    for iterMarker = 1:numMarkersInUse
        finalWeightNames{1,iterMarker} = ikSolver.getMarkerNameForIndex(iterMarker-1).toCharArray()';
    end

    % Set up Data Storage
%     numFrames = markerRef.getMarkerTable().getNumRows();
    numMarkers = markerRef.getMarkerTable().getNumColumns();
    timeVector = markerRef.getMarkerTable.getIndependentColumn();
    
    sampleFreq = markerRef.getSamplingFrequency;
    startFrame = round(startIK * sampleFreq) + 1;
    endFrame = round(endIK * sampleFreq) + 1;
    numFrames = startFrame - endFrame + 1;

    time = nan(numFrames, 1);
    squaredError = nan(numFrames, numModelMarkers);
    totalSquaredError = nan(numFrames, 1);
    rmseError = nan(numFrames, 1);
    worstMarker = cell(numFrames, 1);

    numGenCoordinates = model.getNumCoordinates;
    

    coordinateValues = nan(numFrames, numGenCoordinates);
    coordinateLabels = cell(1,numGenCoordinates); 
    coordinateSet = model.getCoordinateSet();
    for iterCoord = 1:numGenCoordinates
        coordinateLabels{1, iterCoord} = coordinateSet.get(iterCoord-1).getName.toCharArray';
    end


    for iterFrame = startFrame:endFrame
        time(iterFrame, 1) = timeVector.get(iterFrame-1);
        state.setTime(time(iterFrame, 1));
        ikSolver.track(state);

        for iterCoord = 1:numGenCoordinates
            aCoord = coordinateSet.get(iterCoord-1);
            if(strcmp(aCoord.getMotionType().toString, 'Translational'))
                coordinateValues(iterFrame, iterCoord) = aCoord.getValue(state);
            else
                coordinateValues(iterFrame, iterCoord) = aCoord.getValue(state)*180/pi;
            end
        end

        squaredErrorArray = SimTKArrayDouble;
        ikSolver.computeCurrentSquaredMarkerErrors(squaredErrorArray);
        for iterMarker = 1:numMarkersInUse
            squaredError(iterFrame, iterMarker) = squaredErrorArray.at(iterMarker-1);
        end

        [maxValue, maxIndex] = max(squaredError( iterFrame, :), [], 2);
        totalSquaredError(iterFrame, 1) = sum(squaredError(iterFrame, :), 'omitnan');
        rmseError(iterFrame, 1) = sqrt(totalSquaredError(iterFrame, 1)/numMarkers);
        worstMarker{iterFrame, 1} = finalWeightNames{maxIndex};
        if(IKErrorParameters.doDebugPrint)
            message = 'Frame %i (t= %0.5f): \ttotal squared error=%0.8f, marker error: RMS=%0.7f, max=%0.7f (%s)\n';
            fprintf(message, iterFrame-1, state.getTime(), totalSquaredError(iterFrame, 1), rmseError(iterFrame, 1), sqrt(maxValue), worstMarker{iterFrame, 1});
        end
    end
    
    % Print out warnings for squared error
    [maxValue, ~] = max(squaredError, [], 1);
    indexOverSquaredError = sqrt(maxValue) > IKErrorParameters.maxAllowedError;
    numMarkersOverSquaredError = sum(indexOverSquaredError);
    fprintf(['IK Error Checker: Squared Error Violation.\n', ...
        'Trial: %s. There were %i markers over Error threshold.\n'], trialName, numMarkersOverSquaredError);
    if(numMarkersOverSquaredError > 0 )
        badMarkers = finalWeightNames(indexOverSquaredError);
        badMarkerValues = sqrt(maxValue(indexOverSquaredError));  
        for iterMarkers = 1:numMarkersOverSquaredError
            fprintf('\t Marker: %s \t Value: %0.7f\n', badMarkers{iterMarkers}, badMarkerValues(iterMarkers));
        end
    end
    
    % Print out warnings for RMSE error
    indexOverRMSError = rmseError > IKErrorParameters.maxAllowedRMSE;
    numFramesOverError = sum(indexOverRMSError);
    fprintf(['IK Error Checker: RMSE Error Violation.\n', ...
        'Trial: %s. There were %i frames over RMSE Error threshold.\n'], trialName, numFramesOverError);
    
    % Assemble output
    outputStruct.time = time;
    outputStruct.labels = coordinateLabels;
    outputStruct.data = coordinateValues;
    outputStruct.errorLabels = finalWeightNames;
    outputStruct.markerErrors = sqrt(squaredError);
    outputStruct.totalSquaredError = totalSquaredError;
    outputStruct.rmseError = rmseError;
    outputStruct.worstMarker = worstMarker;
    [outputStruct.numRows, outputStruct.numCols] = size(outputStruct.data);
    outputStruct.inDegrees = true;
end
