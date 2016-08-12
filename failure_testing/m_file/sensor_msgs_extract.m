function [C] = sensor_msgs_extract(filename)
    % converts a .csv to a cell array
    % input: path/file.csv
    % output: cell array
    fid = fopen(filename);
    C = textscan(fid, ...
    '%s%n%n%n%n%n', ...
    'delimiter',',', ...
    'headerLines', 1);
    fclose(fid);
end

