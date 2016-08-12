function [t_s,t_ns] = minTime(t1_s, t1_ns, t2_s,t2_ns)
%minTime returns the minimum unix time pair
%   Subtracts time 2 from time 1; returns time 2 if the result is negative
%   and time 1 if the result is positive.
    t = t1_s - t2_s...
        + (t1_ns - t2_ns)/1000000000;
    if t < 0
        t_s = t1_s;
        t_ns = t1_ns;
    else
        t_s = t2_s;
        t_ns = t2_ns;
    end

end

