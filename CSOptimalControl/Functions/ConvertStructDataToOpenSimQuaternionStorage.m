function [] = ConvertStructDataToOpenSimQuaternionStorage(dataStruct, filename_output) 
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
        PrintDelimitedString(fileid, {'DataType=Quaternion'}, '\t', '\n');

        % Header Second Row
        PrintDelimitedString(fileid, {'version=3'}, '\t', '\n');

        % Header Third Row
        PrintDelimitedString(fileid, {'OpenSimVersion=4.4-2022-07-23-0e9fedc'}, '\t', '\n');

        % Header Sixth Row 
        PrintDelimitedString(fileid, {'endheader'}, '\t', '\n');

        % Header Seventh Row
        PrintDelimitedString(fileid, [{'time'}, dataStruct.labels], '\t', '\n');
          
        % Assemble Data
        dataToWrite = [reshape(dataStruct.time, numOutputRows, 1), dataStruct.data];

        % Print Data
        numQuats = (numOutputCols - 1)/4;
        for i = 1:1:length(dataToWrite)
            % Write time row
            PrintDelimitedString(fileid,dataToWrite(i,1), '', '\t')
            
            % Write each quaternion group with commas and tabs
            ind = 2;
            for k = 1:numQuats
                if k < numQuats
                    PrintDelimitedString(fileid, dataToWrite(i,ind:ind+3), ',', '\t');
                else
                    PrintDelimitedString(fileid, dataToWrite(i,ind:ind+3), ',', '\n');
                end
                ind = ind + 4;
            end
        end
        
    catch matlabException
        fclose(fileid);
        rethrow(matlabException);
    end
    
    % Close File
    fclose(fileid);
end