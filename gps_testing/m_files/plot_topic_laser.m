function [gps_stats]=plot_topic_laser(test_type,location,start_test,end_test,save_plot,view_plot)
%% Plot Gps Testing UAV Statistics
if save_plot == 1
    mkdir(sprintf('../plot/%s/',location))
end

% %ld2=zeros(141,2);
% ttime=0;
% ftime=0;
% ref_x = -96.5984788;
% ref_y = 40.858359;
for test = start_test:1:end_test
    test_index = test - start_test + 1;
    
    close all
    if save_plot == 1
        mkdir(sprintf('../plot/%s/%s%i/',location,test_type,test))
    end
 
    % import data from csv files
    % subject gps
    try import_csv=genericExtractor(sprintf('../csv/%s/%s%i/%s%i.csv',...
        location,test_type,test,test_type,test),true);
        l_ns=table2array(import_csv(:,4));
        l_s=table2array(import_csv(:,5));
        adc=table2array(import_csv(:,7));
    catch
        l_ns=0;
        l_s=0;
        yaw_pose=0;
        adc=0;
        y=0;
        z=0;       
    end
   %l_ns
    %% time conversion and sizes
    % find first time as reference 0
    if size(l_ns) > [1,0]
        [start_s,start_ns] = minTime(l_s(1,1),l_ns(1,1),l_s(1,1),l_ns(1,1));
    end
    % convert time vectors from unix time to time from start
    [ml,l_time] = unixTime2TestTime(start_s,start_ns,l_s',l_ns');
     
    %% laser values
    %[min(adc) max(adc) mean(adc) mode(adc) median(adc)]
    dist=zeros(1,ml);
    sdist=zeros(1,ml);
    buff=zeros(1,3);
%     j=1;
    for i=1:1:ml
      if test ~= 17 && abs(adc(i) - mean(adc)) > 3.0
        adc(i) = round(mean(adc));
      end
      if adc(i) < 14
        dist(i) = (adc(i) - 10.0)*(0.1*log((adc(i) - 10.0)*0.1) +0.44);
      else
        dist(i) = (adc(i) - 10.0)*(0.1303*log((adc(i) - 10.0)*0.6)+0.3924); 
      end
      
      if buff(3) ~= 0
        if abs(adc(i) - adc(i-1)) < 2.0 && abs(dist(i) - buff(1)) < 1.0 ...
            && abs(dist(i) - buff(3)) < 1.0 && abs(dist(i) - buff(3)) < 1.0
          sdist(i) = (buff(1) + buff(2) + buff(3) + dist(i))/4;
        else
          sdist(i) = dist(i);
        end
      else
        sdist(i) = dist(i);
      end
      buff(3) = buff(2);
      buff(2) = buff(1);
      buff(1) = sdist(i);
      %[dist(i) sdist(i) buff]
    end
    %[min(adc) max(adc) mean(adc(3*ml/4:ml)) median(adc) median(adc(3*ml/4:ml))]
    %[max(dist) - min(dist) min(dist)]
        %% position Plot
    % 
    if view_plot == 1
        figure
        plot(l_time,adc,'b.') 
        hold on
        grid on
        grid minor
        title('ADC')
        xlabel('Time (seconds)')
        ylabel('ADC counts')
        %axis([0 lp_time(mlp) 0 1 + .1])
        %legend('X Position','Y Position','Z Position')
        hold off
        
        % print figure
        if save_plot == 1
            print(sprintf('../plot/%s/%s%i/adc_%s%i',...
                location,test_type,test,test_type,test),'-djpeg')
        end
        
        figure
        plot(l_time,dist,'b.') 
        hold on
        plot(l_time,sdist,'r.')
        grid on
        grid minor
        title('Dist')
        xlabel('Time (seconds)')
        ylabel('dist meters')
        %axis([0 lp_time(mlp) 0 1 + .1])
        %legend('X Position','Y Position','Z Position')
        hold off
        
        % print figure
        if save_plot == 1
            print(sprintf('../plot/%s/%s%i/dist_%s%i',...
                location,test_type,test,test_type,test),'-djpeg')
        end
    end
end
