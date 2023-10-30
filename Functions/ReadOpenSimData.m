%ReadOpenSimData  
%   OutputData = ReadOpenSimData(dataFileName) creates a data
%   structure from an OpenSim storage (.sto) or motion (.mot) file. 
%
% Note: This function is designed for the data file format used in
% OpenSim 2.3.2+.  For older files, please convert the data file using
% the ConvertFiles... function in the tools menu of the OpenSim GUI
%
% Input:
%   dataFileName: An OpenSim data file (.sto or .mot)
%
% Output:
%   The output of this script is a Matlab structure named OutputData. The
%   format of this structure can be passed to PlotOpenSimFunction.m for
%   plotting.
%
%   The stucture fields are:
%       name: A char array identifier of the data
%       numRows: the number of rows of data in the data field
%       numCols: the number of columns of data in the data field
%       labels: an array of char arrays of data names from the header file
%       data: a nRows by nColumnss matrix of data values
% -----------------------------------------------------------------------  
function OutputData = ReadOpenSimData(dataFileName)
    % Check location with exists
    if(exist(dataFileName, 'file') == 0)
       error('ReadOpenSimStorage:InvalidArgument', ...
           ['\tError in ReadOpenSimStorage:\n', ...
           '\t%s cannot be found'], dataFileName); 
    end

    % Create output structure
    OutputData = struct();

    % Import Data
    unparsedData = importdata(dataFileName, '\t');

    % First line in text always name
    OutputData.name = unparsedData.textdata{1};
    
    % Parse remaining text lines
    for i = 2:1:size(unparsedData.textdata,1)-1
        if(~isempty(unparsedData.textdata{i}))
            scan = textscan(unparsedData.textdata{i}, '%s', 'delimiter', '=');
            if(strcmp(scan{1}(1), 'version'))
                OutputData.version = str2double(scan{1}(2));   
            elseif(strcmp(scan{1}(1), 'nRows'))
                OutputData.numRows = str2double(scan{1}(2));   
            elseif(strcmp(scan{1}(1), 'nColumns'))
                OutputData.numCols = str2double(scan{1}(2));   
            elseif(strcmp(scan{1}(1), 'inDegrees'))
                OutputData.inDegrees = char(scan{1}(2));         
            end
        end
    end
    
    % Find Time column (Should be first)
    timeIndex = strcmp(unparsedData.colheaders, 'time');
    if(~any(timeIndex))
        error('ReadOpenSimData: time column not found')
    else
        % Add in labels and data
        OutputData.time = unparsedData.data(:, timeIndex);
        OutputData.labels = unparsedData.colheaders(:, ~timeIndex); 
        OutputData.data = unparsedData.data(:, ~timeIndex);
    end
end

