function [left_s,right_s] = cropSig(signal,sizeSig,threshhold,sk)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    flag = 0;
    count = 0;
    lcount = 0;
    left_s = 1;
    right_s = 1;
    if sk == 1    
        for i = 1:1:sizeSig-10
            if signal(i,1) > threshhold && flag == 0
                s = i;
                flag = 1;
            elseif signal(i,1) > threshhold && flag == 1 && lcount < 10
                lcount = lcount + 1;
            elseif flag == 1 && signal(i,1) <= threshhold && lcount < 10
                lcount = 0;
                flag = 0;
            end
            if lcount == 10
                left_s = s;
                break;
            end
        end
    else
        flag = 1;
        lcount = 10;
        left_s = 1;
    end
    for i = left_s+10:1:sizeSig-10
        if flag == 1 && signal(i,1) < threshhold && lcount >= 10
            s = i;
            flag = 0;
        elseif flag == 0 && lcount >= 10 && signal(i,1) < threshhold
            count = count + 1;
        elseif flag == 0 && lcount >= 10 && signal(i,1) >= threshhold
            count = 0;
        end
        if count == 10
            right_s = i;
            break;
        end
    end
    
end

