function [] = ConvertStructDataToOpenSimTRC(dataStruct, filename_output, units) 
% ConvertStructDataToOpenSimTRC
%   This function converts a force data struct created by the function
%   ImportAndProcessQualisysForce Data into a OpenSim file (.mot or .sto
%   type)
% 
% Input:
%    dataStruct: Struct with the following fields
%       .time - a column matrix of time values
%       .data - a matrix of data to print
%       .dataRate - data capture rate for the file
%       .labels - labels of the columns of the data field
%     filename_output - filename to write
% 
% Output:
%
% Usage: 
% [] = ConvertStructDataToOpenSimMotionStorage(forceDataStruct, filename_output) 
    
    try
        % Strip output path paths
        [path_out, name_out, ext_out] = fileparts(filename_output);

        % Open File for writing
        fileid = fopen(filename_output, 'w'); 
        if(fileid == -1)
            error('ConvertViconMarkerDataToOpenSim:FileError', ...
                ['The output file could not be opened.' ...
                ' Check to see if it is open in another program']);
        end
        
        % Header First Row
        PrintDelimitedString(fileid, {...
            'PathFileType', '4', '(X/Y/Z)', [name_out, ext_out]}, '\t', '\n');

        % Header Second Row
        PrintDelimitedString(fileid, {...
            'DataRate', 'CameraRate', 'NumFrames', 'NumMarkers', 'Units', ...
            'OrigDataRate', 'OrigDataStartFrame', 'OrigNumFrames'}, '\t', '\n');

        % Header Third Row
        dataRateStr = num2str(dataStruct.dataRate);
        numMarkers = size(dataStruct.data, 2)/3;
        startFrame = 1;
        endFrame = size(dataStruct.data, 1);
%         units = 'mm';
        
        PrintDelimitedString(fileid, {...
            dataRateStr, dataRateStr, num2str(endFrame), num2str(numMarkers), units, ...
            dataRateStr, num2str(startFrame), num2str(endFrame)}, '\t', '\n');

        % Header Fourth Row
        PrintDelimitedString(fileid, [{'Frame#'}, {'Time'}, dataStruct.labels], '\t', '\n');

        % Header Fifth Row
        markerLabels = cell(1, 3*numMarkers+2);
        markerLabels(:) = {''};
        for i = 1:1:numMarkers
            markerLabels{3*i} = sprintf('X%d', i);
            markerLabels{3*i+1} = sprintf('Y%d', i);
            markerLabels{3*i+2} = sprintf('Z%d', i);

        end
        PrintDelimitedString(fileid, markerLabels, '\t', '\n');

        % Header Sixth Row  (Blank Row)
        PrintDelimitedString(fileid, '', '\t', '\n');
      
        % Print Data
        dataToWrite(:,1) = (startFrame:1:endFrame)';
        dataToWrite(:,2) = dataStruct.time';
        dataToWrite(:,3:numMarkers*3+2) = dataStruct.data;  
        for i = 1:1:endFrame
           PrintDelimitedString(fileid, dataToWrite(i,:), '\t', '\n'); 
        end

    catch matlabException
        fclose(fileid);
        error(matlabException.identifier, matlabException.message);
    end
    
    % Close File
    fclose(fileid);
end