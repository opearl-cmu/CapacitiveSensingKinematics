function [status] = PrintDelimitedString(fileid, args, delimiter, closing)
% PrintDelimitedString
%   This function prints a delimted line to the file id using the given
%   delimiter.  Each line is terminated with the closing input.  Spaces in
%   strings are replaced with an underscore.
% 
% Input:
%    fileid: fileid to print to
%    args: cell array of arguments to print
%    delimiter: delimiter to seperate multiple arguments
%    closing: closing line characters
%
% Output:
%    status: 0 if succeeded, 1 if failed.
%
% Usage: [status] = PrintDelimitedString(fileid, args, closing)
    % Get Number of Args
    numArgs = length(args);
      
    % Create a String from a 1 by N Cell or Array
    if(iscell(args))   
        % Replace Spaces with Underscores
        cleanArgs = strrep(args,' ', '_');
        
        % Format and Print
        stringFormat = [];
        for i = 1:numArgs
            if(~isempty(args{i}))
                stringFormat = strcat(stringFormat, '%s');
            end
            if(i < numArgs)
                stringFormat = strcat(stringFormat, delimiter);
            end
        end
        fprintf(fileid, stringFormat, cleanArgs{:});

    elseif(isnumeric(args))
        % Format and Print
        stringFormat = [];
        for i = 1:numArgs
            if(mod(args(i), 1))
                stringFormat = strcat(stringFormat, '%.8f');
            else
                stringFormat = strcat(stringFormat, '%d');
            end
            if(i < numArgs)
                stringFormat = strcat(stringFormat, delimiter);
            end
        end
        fprintf(fileid, stringFormat, args(:));
 
    end
    fprintf(fileid, closing);
end
