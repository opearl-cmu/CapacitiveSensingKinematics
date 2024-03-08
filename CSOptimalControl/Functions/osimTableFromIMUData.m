function timeseriesosimtable = osimTableFromIMUData(s, onlyFrameLabels, bodyLabels)
% Import opensim and get CRANKING
import org.opensim.modeling.*;

timeseriesosimtable = TimeSeriesTableVec3();
% Set the TimesSeriesTable() column names
osimlabels = StdVectorString();
for i = 1:length(onlyFrameLabels)
    osimlabels.add( onlyFrameLabels{i} );
end
timeseriesosimtable.setColumnLabels(osimlabels);

% Get an OpenSim Row Vector
row = RowVectorVec3(length(onlyFrameLabels));
nRows = length(s.time);
for iRow = 1:nRows
    % Create and fill a row of data
    for iCol = 1:length(onlyFrameLabels)
        % Make a vec3 element from the rowdata
        row.set(iCol-1, osimVec3FromArray(s.(bodyLabels{iCol})(iRow,:)));
    end
    % Append the RowVectorofVec3's to the opensim table
    timeseriesosimtable.appendRow(iRow-1, row);
end

timeColumn = timeseriesosimtable.getIndependentColumn();
for i = 1:nRows
    timeColumn.set(i-1, s.time(i));
end

end
