function [ signal ] = smoothNoise(signal,upper_bound,lower_bound,sizeSig)
%smoothNoise: smooth noisy data points
%   smooth out signal noise outside of [lower_bound, upper_bound]
    for i = 1:1:sizeSig
        if signal(i,1) > upper_bound || signal(i,1) < lower_bound
            if i > 1 && i < sizeSig && signal(i+1,1) < upper_bound && signal(i+1,1) > lower_bound
                signal(i,1) = (signal(i-1,1) + signal(i+1,1))/2;
            elseif i == 1
                signal(i,1) = signal(i+1,1);
			else
				signal(i,1) = signal(i-1,1);
            end
        end
    end
end
