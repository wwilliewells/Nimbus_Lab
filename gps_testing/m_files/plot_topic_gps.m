function [gps_stats]=plot_topic_gps(vehicle,vehicle_type,start_test,end_test,save_plot,view_plot)
%% Plot Gps Testing UAV Statistics
if save_plot == 1
    mkdir(sprintf('../plot/%s/',vehicle_type))
    mkdir(sprintf('../plot/%s/%s/',vehicle_type,vehicle))
end


ld2=zeros(141,2);
for test = start_test:1:end_test
    test_index = test - start_test + 1;
    
    close all
    if save_plot == 1
        mkdir(sprintf('../plot/%s/%s/%s%i/',vehicle_type,vehicle,vehicle,test))
    end
 
    % import data from csv files
    % subject pose
    import_csv=genericExtractor(sprintf('../csv/%s/%s/%s%i/position_%s%i.csv',...
        vehicle_type,vehicle,vehicle,test,vehicle,test),true);
    if size(import_csv) > [0,0]
        sp_ns=table2array(import_csv(:,3));
        sp_s=table2array(import_csv(:,4));
        yaw_sp=table2array(import_csv(:,8));
        x=table2array(import_csv(:,9));
        y=table2array(import_csv(:,10));
        z=table2array(import_csv(:,11));
    else
        sp_ns=0;
        sp_s=0;
        yaw_sp=0;
        x=0;
        y=0;
        z=0;
    end
    
    % rf test
%     vehicle_type
%     import_csv=genericExtractor(sprintf('../csv/%s/%s/%s%i/radio_%s%i.csv',...
%         vehicle_type,vehicle,vehicle,test,vehicle,test),true);
%     if size(import_csv) > [0,0]
%         rf_ns=table2array(import_csv(:,4));
%         rf_s=table2array(import_csv(:,5));
%         bytes=table2array(import_csv(:,1));
%         total_bytes=table2array(import_csv(:,7));
%         %y=table2array(import_csv(:,10));
%     else
%         rf_ns=0;
%         rf_s=0;
%         bytes=0;
%         total_bytes=0;
%         %z=0;
%     end
    
    % subject gps
%     import_csv=genericExtractor(sprintf('../csv/%s/%s/%s%i/gps_%s%i.csv',...
%         vehicle_type,vehicle,vehicle,test,vehicle,test),true);
%     if size(import_csv) > [0,0]
%         sg_ns=table2array(import_csv(:,3));
%         sg_s=table2array(import_csv(:,4));
%         yaw_sg=table2array(import_csv(:,10));
%         lat_sg=table2array(import_csv(:,5));
%         long_sg=table2array(import_csv(:,6));
%         z_g=table2array(import_csv(:,1));
%     end
    % laser reading
    import_csv=genericExtractor(sprintf('../csv/%s/%s/%s%i/sensor_%s%i.csv',...
        vehicle_type,vehicle,vehicle,test,vehicle,test),true);
    if size(import_csv) > [0,0]
        
        if test == 6
            lp_ns=table2array(import_csv(:,5));
            lp_s=table2array(import_csv(:,6));
        else
            lp_ns=table2array(import_csv(:,4));
            lp_s=table2array(import_csv(:,5));
        end
        if test > 17
            laser_percent=table2array(import_csv(:,6));
            laser=table2array(import_csv(:,8));
        elseif test > 1 && test < 4
            laser=table2array(import_csv(:,8));
        elseif test == 6
            laser=table2array(import_csv(:,9));
            laser_percent=table2array(import_csv(:,7));
            laser_dist=table2array(import_csv(:,1));
        else
            laser=table2array(import_csv(:,7));
            
        end
    else
        lp_ns=0;
        lp_s=0;
        laser=0;
    end
    % time conversion and sizes
    % find first time as reference 0 
    if size(sp_ns) > [1,0]
        [start_s,start_ns] = minTime(sp_s(1,1),sp_ns(1,1),lp_s(1,1),lp_ns(1,1));
    else
        if size(laser) > [1,0]
            [start_s,start_ns] = minTime(lp_s(1,1),lp_ns(1,1),lp_s(1,1),lp_ns(1,1));
        else
            [start_s,start_ns] = minTime(rf_s(1,1),rf_ns(1,1),rf_s(1,1),rf_ns(1,1));
        end
    end
%     [start_s,start_ns] = minTime(start_s,start_ns,sg_s(1,1),sg_ns(1,1));
    % convert time vectors from unix time to time from start
    [msp,sp_time] = unixTime2TestTime(start_s,start_ns,sp_s',sp_ns');
    %[mrf,rf_time] = unixTime2TestTime(start_s,start_ns,rf_s',rf_ns');
%     [msg,sg_time] = unixTime2TestTime(start_s,start_ns,sg_s',sg_ns');
    [mlp,lp_time] = unixTime2TestTime(start_s,start_ns,lp_s',lp_ns');
    
    %% bad serial
    laser = smoothNoise(laser,1024,0,mlp);
    
    
    %% position Plot
    %[msp,mlp]
    % 3-D position
    if view_plot == 1
%         figure
%         plot3(x,y,z,'c.') % subject pose
%         hold on 
%         %plot3(x,y,laser,'r.') % task waypose
%         grid on
%         grid minor
%         title(sprintf('Path of %s%i',vehicle,test))
%         xlabel('Position in X Direction (meters)')
%         ylabel('Position in Y Direction (meters)')
%         zlabel('Position in Z Direction (meters)')
%         %legend('Actual Path', 'Ideal Path')

%         % print
%         if save_plot == 1
%             print(sprintf('../plot/%s/%s/%s%i/path_%s%i',...
%                 vehicle_type,vehicle,vehicle,test,vehicle,test),'-djpeg')
%         end

        % 1-D position with respect to time
        figure
        %subplot(2,1,1), 
        hold on
         %plot(sp_time,z,'b.')
        plot(lp_time,laser_percent*9.8+.2,'c.')
        plot(lp_time,laser_dist,'k.')%0.0046*laser+0.2781
        plot(lp_time,0.0098*laser+0.2,'r.')
        grid on
        grid minor
        title('Distance')
        xlabel('Time (seconds)')
        ylabel('Distance (meters)')
        %axis([0 lp_time(mlp) 0 1 + .1])
        %legend('X Position','Y Position','Z Position')
        hold off

        % print figure
        if save_plot == 1
            print(sprintf('../plot/%s/%s/%s%i/position_%s%i',...
                vehicle_type,vehicle,vehicle,test,vehicle,test),'-djpeg')
        end

        % 1-D position with respect to time
%         figure
%         hold on
%         plot(rf_time,bytes,'b.')
%          %plot(sp_time,y,'c.')
%          %plot(sp_time,x,'k.')%0.0046*laser+0.2781
%         %plot(lp_time,0.0097*laser+0.175,'r.')
%         grid on
%         grid minor
%         title('Bytes')
%         xlabel('Time (seconds)')
%         ylabel('Bytes (count)')
%         axis([165 175 0 20])
%         %legend('X Position','Y Position','Z Position')
%         hold off
% 
%         % print figure
%         if save_plot == 1
%             print(sprintf('../plot/%s/%s/%s%i/rf_%s%i',...
%                 vehicle_type,vehicle,vehicle,test,vehicle,test),'-djpeg')
%         end
        
        % 1-D position with respect to time
        figure
        hold on
        plot(lp_time,laser_percent,'b.')
        grid on
        grid minor
        title('percent adc')
        xlabel('Time (seconds)')
        ylabel('percent (count)')
        %legend('X Position','Y Position','Z Position')
        hold off

        % print figure
        if save_plot == 1
            print(sprintf('../plot/%s/%s/%s%i/percent_%s%i',...
                vehicle_type,vehicle,vehicle,test,vehicle,test),'-djpeg')
        end

        % 1-D position with respect to time
        figure
        plot(lp_time,laser_dist,'c.')
        grid on
        grid minor
        title('distance')
        xlabel('Time (seconds)')
        ylabel('Distance (meters)')
        %legend('X Position','Y Position','Z Position')

        % print figure
        if save_plot == 1
            print(sprintf('../plot/%s/%s/%s%i/distance_%s%i',...
                vehicle_type,vehicle,vehicle,test,vehicle,test),'-djpeg')
        end
        
%         figure
%         hold on
%         plot(ld2(:,2),ld2(:,1),'b.')
%         grid on
%         grid minor
%         title('')
%         xlabel('raw values)')
%         ylabel('dist')
% %         axis([0 sp_time(mx) + 0.001 min(min(min(z,y),x)) - .1 max(max(max(z,y),x)) + .1])
%         %legend('X Position','Y Position','Z Position')
%         hold off
% 
%         % print figure
%         if save_plot == 1
%             print(sprintf('../plot/%s/%s/%s%i/dist_%s%i',...
%                 vehicle_type,vehicle,vehicle,test,vehicle,test),'-djpeg')
%         end
        
        % gps
%         subplot(2,1,2),  
%         hold on
%         plot(sg_time,lat_sg,'b.')
%         plot(sg_time,long_sg,'c.')
%         plot(sg_time,z_g,'k.')
%         plot(lp_time,laser,'r.')
%         %axis([0 max(qci_time) + 0.001 -1.1 1.1])
%         grid on
%         grid minor
%         title('Quad Control Input Pitch, Roll, and Thrust')
%         xlabel('Time (seconds)')
%         ylabel('Controller Control Rates (unit normalized)')
%         hold off

%         figure
%          plot3(sp_time,x,y,'c.') % subject pose
%         hold on 
%         plot3(lp_time,laser/1024,laser/2048,'r.') % task waypose
%         grid on
%         grid minor
%         title(sprintf('Path of %s%i',vehicle,test))
%         xlabel('Time (seconds)')
%         ylabel('Position in X Direction (meters)')
%         zlabel('Position in Y Direction (meters)')
%         
%         % print figure
%         if save_plot == 1
%             print(sprintf('../plot/%s/%s/%s%i/twod_1_%s%i',...
%                 vehicle_type,vehicle,vehicle,test,vehicle,test),'-djpeg')
%         end
%         
%         figure
%         plot3(sp_time,x,z,'c.') % subject pose
%         hold on 
%         plot3(lp_time,laser/1024,laser/2048,'r.') % task waypose
%         grid on
%         grid minor
%         title(sprintf('Path of %s%i',vehicle,test))
%         xlabel('Time (seconds)')
%         ylabel('Position in X Direction (meters)')
%         zlabel('Position in Z Direction (meters)')
%         
%         % print figure
%         if save_plot == 1
%             print(sprintf('../plot/%s/%s/%s%i/twod_2_%s%i',...
%                 vehicle_type,vehicle,vehicle,test,vehicle,test),'-djpeg')
%         end
%         
%         figure
%         plot3(sp_time,y,z,'c.') % subject pose
%         hold on 
%         plot3(lp_time,laser/1024,laser/2048,'r.') % task waypose
%         grid on
%         grid minor
%         title(sprintf('Path of %s%i',vehicle,test))
%         xlabel('Time (seconds)')
%         ylabel('Position in Y Direction (meters)')
%         zlabel('Position in Z Direction (meters)')
% 
%         % print figure
%         if save_plot == 1
%             print(sprintf('../plot/%s/%s/%s%i/twod_3_%s%i',...
%                 vehicle_type,vehicle,vehicle,test,vehicle,test),'-djpeg')
%         end
     end

    
end
%ld2
ld3=zeros(13,2);
ld3(:,1)=0.8:0.1:2.0;
k=1;
for j=1:1:13
    for i=k:1:141
        if ld2(i,1) == ld3(j,1) 
            if ld2(i,2) > 0
                ld3(j,2) = ld2(i,2);
                k=i+1;
                break;
            else
                for m=i-1:1:i+1
                    if ld2(m,2) > 0
                        ld3(j,2) = ld2(m,2);
                        break
                    end
                end
            end
        end
    end
end
%ld3

% b=ld2(1,2);
% for i=2:1:141
%     if abs(ld2(i,2) - b) > 10
%         s=i;
%         break;
%     end
% end
% for i=1:1:s
    