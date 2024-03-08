function [] = ParforSave_SingleStructure(filename, structToSave)
% ParforSave_SingleStructure
%   This splits the save into a function so that it is compatible with
%   parfor from the Parallel computing toolbox.
% 
% Input:
%    filename: the filename to write
%    inputArg2: threshold value
%
% Output:
%    None
%
% Usage: ParforSave_SingleStructure(filename, structToSave); 
    save(filename, '-struct', 'structToSave');
end
 

