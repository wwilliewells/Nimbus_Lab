function [signal1, signal2, m] = equalizePower(signal1,signal2,version)
%equalizePower: removes noisy values and returns equal sized arrays 
%   Forces larger length signal to smaller length and returns both signals 
%   and the length; version is based on changes in power toipic output  
    mc = length(signal2');
    mv = length(signal1');

    % smooth out noise
    signal1 = smoothNoise(signal1,15.0,9.0,mv)';
    signal2 = smoothNoise(signal2,30.0,0.0,mc)';

    % equalize amount of data points
    if version < 3
        if mc < mv
            signal1 = signal1(1:mc);
            m = mc;
        elseif mc > mv
            signal2 = signal2(1:mv);
            m = mv;
        else
            m = mv;
        end
    else
        m = mv;
    end
end

