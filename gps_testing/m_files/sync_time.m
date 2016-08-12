function [index2] = sync_time(time1,time2,index1)
%sync time given an index of one time vector return corresponding index
%   in second time vector
    for i=1:1:length(time2)
        if time2(i) > time1(index1)
            index2 = i;
            return;
        end
    end
    index2 = length(time2);
end
