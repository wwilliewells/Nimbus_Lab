function [is_interrupt,diff_time] = qci_interrupt(qci_time,start_index,stop_index,ctrl_rate)
%QCI Interrupt: Check if Quad Control Input was interrupted
%   Detailed explanation goes here
    is_interrupt = 0;
    diff_time=0.0;
    for i = start_index+1:1:stop_index
        diff_time = qci_time(i) - qci_time(i-1);
        if qci_time(i) - qci_time(i-1) > 4*ctrl_rate
            is_interrupt = 1;
            return;
        end
    end
end

