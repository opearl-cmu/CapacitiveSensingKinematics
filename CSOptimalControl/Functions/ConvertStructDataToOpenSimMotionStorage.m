function [] = ConvertStructDataToOpenSimMotionStorage(dataStruct, filename_output) 
% ConvertStructDataToOpenSimMotionStorage
%   This function converts a force data struct created by the function
%   ImportAndProcessQualisysForce Data into a OpenSim file (.mot or .sto
%   type)
% 
% Input:
%    dataStruct: Struct with the following fields
%       .time - a column matrix of time values
%       .data - a matrix of data to print
%       .labels - labels of the columns of the data field
%     filename_output - filename to write
% 
% Output:
%
% Usage: 
% [] = ConvertStructDataToOpenSimMotionStorage(forceDataStruct, filename_output) 

    try 
        % Process input data
        [path_out, name_out, ext_out] = fileparts(filename_output); 
       
        % Open File for writing
        fileid = fopen(filename_output, 'w'); 
        if(fileid == -1)
            ME = MException('ConvertStructDataToOpenSimMotionStorage:FileError', ...
                ['The output file could not be opened.' ...
                ' Check to see if it is open in another program.']);
            throw(ME)
        end
           
        % Row and Column count ( data-numNonForces + time)
        [numOutputRows, numOutputCols] = size(dataStruct.data);
        numOutputCols = numOutputCols + 1;
              
        % Header First Row
        PrintDelimitedString(fileid, {[name_out, ext_out]}, '\t', '\n');

        % Header Second Row
        PrintDelimitedString(fileid, {'version=1'}, '\t', '\n');

        % Header Third Row
        PrintDelimitedString(fileid, {sprintf('nRows=%d', numOutputRows)}, '\t', '\n');

        % Header Fourth Row
        PrintDelimitedString(fileid, {sprintf('nColumns=%d', numOutputCols)}, '\t', '\n');

        % Header Fourth Row
%         PrintDelimitedString(fileid, {'inDegrees=no'}, '\t', '\n');
        if strcmp(dataStruct.inDegrees,'yes')
            PrintDelimitedString(fileid, {'inDegrees=yes'}, '\t', '\n');
        elseif strcmp(dataStruct.inDegrees,'no')
            PrintDelimitedString(fileid, {'inDegrees=no'}, '\t', '\n'); 
        end

        % Header Sixth Row 
        PrintDelimitedString(fileid, {'endheader'}, '\t', '\n');

        % Header Seventh Row
        PrintDelimitedString(fileid, [{'time'}, dataStruct.labels], '\t', '\n');
          
        % Assemble Data
        dataToWrite = [reshape(dataStruct.time, numOutputRows, 1), dataStruct.data];

        % Print Data
        for i = 1:1:length(dataToWrite)
            PrintDelimitedString(fileid, dataToWrite(i,:), '\t', '\n');
        end
        
    catch matlabException
        fclose(fileid);
        rethrow(matlabException);
    end
    
    % Close File
    fclose(fileid);
end