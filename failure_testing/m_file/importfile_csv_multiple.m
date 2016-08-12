function [secs,nsecs,foi_1,foi_2,foi_3,foi_4,foi_5,foi_6,foi_7,foi_8] = importfile(filename, startRow, endRow, topic)
% relative time (sample time), seconds, foi = fields of interest
%IMPORTFILE Import numeric data from a text file as column vectors.
%   [secs,,nsecs,foi_1,foi_2,foi_3,foi_4,foi_5,foi_6,foi_7,foi_8]
%   = IMPORTFILE(filename, startRow, endRow, topic) Reads data from rows 
%   STARTROW through ENDROW of text file FILENAME. Topic specifies the 
%   particular format to use when importing the data;
%
%    See also TEXTSCAN.
% Auto-generated by MATLAB on 2015/05/28 13:32:31

%% Initialize variables.
delimiter = ',';
if nargin<=2
    startRow = 2;
    endRow = inf;
end

%% Format string for each line of text:
if topic == 0 % sensor v0
    formatSpec = '%f%[^\n\r]';
elseif topic == 1 % position
    formatSpec = '%*s%f%f%f%f%f%f%f%f%f%f%s%[^\n\r]';
elseif topic == 2 % state
    formatSpec = '%*s%f%f%f%f%s%f%[^\n\r]';
elseif topic == 3 % status
    formatSpec = '%f%f%f%f%s%f%f%f%f%f%f%s%f%f%[^\n\r]';
elseif topic == 4 % vicon
    formatSpec = '%s%s%f%f%f%f%f%f%f%f%f%f%[^\n\r]';
elseif topic == 5 % control input
    formatSpec = '%f%d%s%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';
elseif topic == 6 % sensor v1
    formatSpec = '%f%s%f%f%f%f%f%[^\n\r]';
elseif topic == 7 % sensor v2
    formatSpec = '%f%f%f%s%f%f%f%f%f%f%[^\n\r]';
else
    return;
end

% For more information, see the TEXTSCAN documentation.
%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
if topic < 6
    dataArray = textscan(fileID, formatSpec, endRow(1)-startRow(1)+1, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines', startRow(1)-1, 'ReturnOnError', false);
    for block=2:length(startRow)
        frewind(fileID);
        dataArrayBlock = textscan(fileID, formatSpec, endRow(block)-startRow(block)+1, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines', startRow(block)-1, 'ReturnOnError', false);
        for col=1:length(dataArray)
            dataArray{col} = [dataArray{col};dataArrayBlock{col}];
        end
    end
else
    dataArray = textscan(fileID, formatSpec, endRow(1)-startRow(1)+1, 'Delimiter', delimiter, 'HeaderLines', startRow(1)-1, 'ReturnOnError', false);
    for block=2:length(startRow)
        frewind(fileID);
        dataArrayBlock = textscan(fileID, formatSpec, endRow(block)-startRow(block)+1, 'Delimiter', delimiter, 'HeaderLines', startRow(block)-1, 'ReturnOnError', false);
        for col=1:length(dataArray)
            dataArray{col} = [dataArray{col};dataArrayBlock{col}];
        end
    end
end
%% Close the text file.
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Allocate imported array to column variable names
if topic == 0 % sensor
    secs = dataArray{:, 1};
elseif topic == 1 % position
    foi_7 = dataArray{:, 1};
    secs = dataArray{:, 3};
    nsecs = dataArray{:, 2};
    foi_5 = dataArray{:, 5};
    foi_6 = dataArray{:, 6};
    foi_4 = dataArray{:, 7};
    foi_1 = dataArray{:, 8};
    foi_2 = dataArray{:, 9};
    foi_3 = dataArray{:, 10};
elseif topic == 2 % state
    foi_7 = dataArray{:, 1};
    secs = dataArray{:, 3};
    nsecs = dataArray{:, 2};
    foi_2 = dataArray{:, 2};
    foi_1 = dataArray{:, 4};
elseif topic == 3 % status
    foi_1 = dataArray{:, 2};
    foi_7 = dataArray{:, 6};
    nsecs = dataArray{:,7};
    secs = dataArray{:,8};
    foi_2 = dataArray{:, 9};
    foi_3 = dataArray{:, 11};
    foi_4 = dataArray{:, 14};
elseif topic == 4 % vicon
    foi_8 = dataArray{:, 3};
    foi_7 = dataArray{:, 6};
    secs = dataArray{:, 5};
    nsecs = dataArray{:, 4};
    foi_1 = dataArray{:, 7};
    foi_2 = dataArray{:, 8};
    foi_3 = dataArray{:, 9};
    foi_4 = dataArray{:, 10};
    foi_5 = dataArray{:, 11};
    foi_6 = dataArray{:, 12};
elseif topic == 5 % control input
    foi_5 = dataArray{:, 2};
    foi_7 = dataArray{:, 4}; % controller sequence
    nsecs = dataArray{:, 5};
    secs = dataArray{:, 6};
    foi_1 = dataArray{:, 7};
    foi_2 = dataArray{:, 9};
    foi_3 = dataArray{:, 11};
    foi_4 = dataArray{:, 13};
    foi_6 = dataArray{:, 4};
elseif topic == 6 % modified sensor message
    foi_3 = dataArray{:, 1}; % flying
    nsecs = dataArray{:, 4};
    secs = dataArray{:, 5};
    foi_2 = dataArray{:, 6}; % task
    foi_1 = dataArray{:, 7}; % value
elseif topic == 7 % version three sensor message
    nsecs = dataArray{:, 6};
    secs = dataArray{:, 7};
    foi_1 = dataArray{:, 9};
    foi_2 = dataArray{:, 10};
    foi_3 = dataArray{:, 1};
    foi_4 = dataArray{:, 2};
    foi_5 = dataArray{:, 8};
    foi_6 = dataArray{:, 3};
end
