function [gps_stats]=plot_topic_odroid(test_type,start_test,end_test,save_plot,view_plot)
%% Plot Gps Testing UAV Statistics
if save_plot == 1
    mkdir(sprintf('../plot/odroid/'))
    % test_type: rf, gps, full (both rf and gps)
    mkdir(sprintf('../plot/odroid/%s/',test_type))
end

%ld2=zeros(141,2);
for test = start_test:1:end_test
    test_index = test - start_test + 1;
    
    close all
    if save_plot == 1
        mkdir(sprintf('../plot/odroid/%s/%s%i/',test_type,test_type,test))
    end
 
    % import data from csv files
    if (strcmp(test_type,'rf') == 1 || strcmp(test_type,'full') == 1)
        % ack, rf_receive, rf_transmit
        import_csv=genericExtractor(sprintf('../csv/odroid/%s%i/ack%i.csv',...
            test_type,test,test),true);
        if size(import_csv) > [0,0]
            if test < 20
                ack_ns=table2array(import_csv(:,4));
                ack_s=table2array(import_csv(:,5));
                bytes=table2array(import_csv(:,1));
                rssi=table2array(import_csv(:,6));
            else
                ack_ns=table2array(import_csv(:,4));
                ack_s=table2array(import_csv(:,5));
                bytes=table2array(import_csv(:,1));
                seq=table2array(import_csv(:,6));
                rssi=table2array(import_csv(:,7));
            end
        else
            ack_ns=0;
            ack_s=0;
            bytes=0;
            rssi=0;
        end

%         % rf_receive
%         import_csv=genericExtractor(sprintf('../csv/odroid/%s%i/rf_receive%i.csv',...
%             test_type,test,test),true);
%         if size(import_csv) > [0,0]
%             stream_len=table2array(import_csv(:,4));
%             rx_data=table2array(import_csv(:,1))
%         else
%             stream_len=0;
%             rx_data=0;
%         end
% 
%         % rf_transmit
%         import_csv=genericExtractor(sprintf('../csv/odroid/%s%i/rf_transmit%i.csv',...
%             test_type,test,test),true);
%         if size(import_csv) > [0,0]
%             stream_len=table2array(import_csv(:,4));
%             rx_data=table2array(import_csv(:,1));
%         else
%             stream_len=0;
%             rx_data=0;
%         end
    end
    
    % time conversion and sizes
    % find first time as reference 0 
    if size(ack_ns) > [1,0]
        [ma,ack_time] = unixTime2TestTime(ack_s(1,1),ack_ns(1,1),ack_s',ack_ns');
    end
    
    
    %% Calculate bytes received per unit time
    t_rssi = zeros(ma,1);
    t_bytes = zeros(ma,1);
    t_atime = zeros(ma,1);
    j=1;
    for i=1:1:ma
        if rssi(i) ~= 0
            t_rssi(j) = rssi(i);
            t_bytes(j) = bytes(i);
            t_atime(j) = ack_time(i);
            j = j + 1;
        end
    end
    rssi=t_rssi(1:j-1);
    bytes=t_bytes(1:j-1);
    ack_time=t_atime(1:j-1);
    ma = j-1;

    %
    if test < 20
        a = 6;
    else
        a=7;
    end
    test_cycles=ceil((ack_time)/5);
    expected_bytes=zeros(ma,3);
    expected_bytes(2,:)=25-a;
    cumalitive_bytes=zeros(ma,3);
    cumalitive_bytes(2,:) = bytes(2);
    interval = 25*9; 
    first_interval = interval - a;
    for i=3:1:ma
        cumalitive_bytes(i,1) = bytes(i) + cumalitive_bytes(i-1,1);
        expected_bytes(i,1) = 25*(test_cycles(i)-1) + expected_bytes(2,1);
        if expected_bytes < first_interval + 1
            expected_bytes(i,2)=mod(expected_bytes(i,1),first_interval);
            if expected_bytes(i,2)== 0
                expected_bytes(i,2)=first_interval;
            end
            cumalitive_bytes(i,2)=mod(cumalitive_bytes(i,1),first_interval+1);
            j=i;
        else
            expected_bytes(i,2)=mod(expected_bytes(i,1)-first_interval,interval);
            if expected_bytes(i,2)== 0
                expected_bytes(i,2)=interval;
            end
            if expected_bytes(i,2) < expected_bytes(i-1,2)
                j=i-1;
            end
            cumalitive_bytes(i,2)=mod(cumalitive_bytes(i,1)-cumalitive_bytes(j,1),interval+1);
        end
    end
    data_rx3 = cumalitive_bytes(:,1)./expected_bytes(:,1);
    data_rx4 = cumalitive_bytes(:,2)./expected_bytes(:,2);
    
    %expected_bytes
    %cumalitive_bytes
    
    data_rx = zeros(ma-1,1);
    data_rx2 = zeros(ma-1,1);
    %data_rx3 = zeros(ma,1);
    for i=1:1:ma-1
        data_rx(i) = (bytes(i+1) + bytes(i))/(ack_time(i+1) - ack_time(i));
        data_rx2(i) = (bytes(i+1))/((5)*(ack_time(i+1) - ack_time(i)));
        %(ack_time(i+1) - ack_time(i))
        %data_rx3(i) = (bytes(i))/((4.990)*(ack_time(i+1) - ack_time(i)));
        if data_rx2(i) > 1
            data_rx2(i)=1;
        end
    end

%     median(data_rx)
%     mode(data_rx)
%     mean(data_rx)     
    
    %% 
    if test > 19
        packets = zeros(2,1);%2^16
        j = 0; k = 256;
        for i=2:1:ma
            if i > k
                j = k;
                k = k + 256;
            end
            packets(seq(i)+j+1) = 1;
        end
        mp=length(packets);
        if view_plot == 1
            % packets
            figure 
            plot((1:1:mp),packets,'bx')
            hold on
            grid on
            grid minor
            title('packets')
            xlabel('packet (count)')
            ylabel('success (1,0)')
            %axis([0 lp_time(mlp) 0 1 + .1])
            %legend('X Position','Y Position','Z Position')

            % print figure
            if save_plot == 1
                print(sprintf('../plot/odroid/%s/%s%i/packet%i',...
                    test_type,test_type,test,test),'-djpeg')
            end
        end
    end
    %mp=size(packets)
    %% Ack Plots
    if view_plot == 1
        % rssi
        figure
        subplot(2,1,1), 
        plot(ack_time,rssi,'bx')
        hold on
        grid on
        grid minor
        title('RSSI')
        xlabel('Time (seconds)')
        ylabel('RSSI (dBm)')
        %axis([0 lp_time(mlp) 0 1 + .1])
        %legend('X Position','Y Position','Z Position')
        
        % bytes
        subplot(2,1,2), 
        plot(ack_time,bytes,'bx')
        grid on
        grid minor
        title('Bytes')
        xlabel('Time (seconds)')
        ylabel('bytes (count)')
        %axis([0 lp_time(mlp) 0 1 + .1])
        %legend('X Position','Y Position','Z Position')
        hold off

        % print figure
        if save_plot == 1
            print(sprintf('../plot/odroid/%s/%s%i/ack%i',...
                test_type,test_type,test,test),'-djpeg')
        end
        
        % bytes received per unit time
        figure
        plot(ack_time(2:ma),data_rx3(2:ma)*100,'bx')
        hold on
        plot(ack_time(2:ma),data_rx4(2:ma)*100,'k+')
        plot(ack_time(3:ma),data_rx2(2:ma-1)*100,'g.')
        grid on
        grid minor
        title('Percent ')
        xlabel('Time (seconds)')
        ylabel('percent of expected bytes received per unit time')
        %axis([0 lp_time(mlp) 0 1 + .1])
        %legend('X Position','Y Position','Z Position')
        
        % print figure
        if save_plot == 1
            print(sprintf('../plot/odroid/%s/%s%i/percent%i',...
                test_type,test_type,test,test),'-djpeg')
        end
    end

end