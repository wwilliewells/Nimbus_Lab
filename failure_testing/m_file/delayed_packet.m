function [delay,packet_loss,flag,max_delay] = delayed_packet(topic_time,...
    begin_time,end_time,m,gap,gaptime_flag,land_flag)
%delayed_packet: Return delayed packet timing information
%   Returns a set of time pairs which are the start and end of a delay in 
%   delay; packet_loss contains the last abnormal delay time and length of 
%   the corresponding delay; flag is initialized to 0 and set to 1 if there
%   is a delay in the interval; max_delay records the longest delay and the
%   time it occurred 
    count = 1;
    delay = zeros(1,2);
    packet_loss = zeros(2,1);
    flag = 0;
    max_delay =  zeros(2,1);
    for i = 2:1:m
        if topic_time(i) > begin_time && topic_time(i) < end_time && topic_time(i-1) > 14.0
            gap_time = topic_time(i) - topic_time(i-1); 
%             if gap_time > gap/2
%                 [gap_time topic_time(i)]
%             end
            if land_flag == 1 && topic_time(i) >= end_time - 2.0
                gap = gap + 2.0;
            end
                
            if (gaptime_flag == 0 && gap_time > gap) || (gaptime_flag == 1 && abs(1.0 - gap_time) > gap) 
                if gap_time > max_delay(1,1)
                    max_delay(1,1) = gap_time;
                    max_delay(2,1) = topic_time(i-1);
                end
                packet_loss(1,1) = gap_time;
                packet_loss(2,1) = topic_time(i-1);
                flag = 1; % delay of topic
                
                delay(count) = topic_time(i-1);
                delay(count+1) = topic_time(i);
                count = count + 2;
            end
        end
    end
end
