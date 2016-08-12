function [m,t] = unixTime2TestTime(sync_s,sync_ns,topic_s,topic_ns)
%unixTime2TestTime: convert unix time to a time from start in seconds
%   return size (m) of time array and times (t) from start of test 
    m = length(topic_s);
    t = zeros(1,m);
    for i = 1:1:m
        t(i) = topic_s(i) - sync_s...
            + (topic_ns(i) - sync_ns)/1000000000;
    end
end
