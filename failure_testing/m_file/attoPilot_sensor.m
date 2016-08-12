function [bag_data] = attoPilot_sensor(cell_array, v_or_c)
    % converts a cell array to a struct with either voltage or current
    % as a heading
    if v_or_c == 1
        bag_data = struct(...
            'voltage', cell_array{6});
    elseif v_or_c == 0
        bag_data = struct(...
            'current', cell_array{2});
    end
end