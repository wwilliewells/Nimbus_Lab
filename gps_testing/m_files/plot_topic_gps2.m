function [gps_stats]=plot_topic_gps(vehicle,vehicle_type,start_test,end_test,locale,save_plot,view_plot)
%% Plot Gps Testing UAV Statistics
if save_plot == 1
    mkdir(sprintf('../../plot/%s/',vehicle_type))
end

ttime=0;
ftime=0;
switch locale
  case 0
    % schorr
    ref_x = -96.705753;
    ref_y = 40.819432;
    ref_z = 357.7323;
  case 1
    % green
    ref_x = -96.705569;
    ref_y = 40.818347;
    ref_z = 360.2323;
  case 2
    % havelock
    ref_x = -96.597602;
    ref_y = 40.858968;
    ref_z = 348.7323;
  otherwise
    ref_x = 0;
    ref_y = 0;
    ref_z = 0;
end

for test = start_test:1:end_test
    test_index = test - start_test + 1;
    
    close all
    if save_plot == 1
        mkdir(sprintf('../../plot/%s/%s%i/',vehicle_type,vehicle,test))
    end
 
    % import data from csv files
    %% byte log
%     import_bag=bagReader(sprintf('../../bags/%s_bag%i.bag',...
%     vehicle_type,test), '/a/byte');
%     tb=table2array(import_bag(:,1));
%     b_s=table2array(import_bag(:,7));
%     b_s=b_s-b_s(1);
%     packet_size=table2array(import_bag(:,2));
%     ps=packet_size(1:2:length(packet_size))+packet_size(2:2:length(packet_size));
%     b_s2=b_s(1:2:length(packet_size));
%     tb2=tb(1:2:length(packet_size));
%     fail=zeros(3,1);
%     j=1;
%     for i=2:1:length(b_s)
%       if packet_size(i) ~= 7 && packet_size(i) ~= 13
%         fail(1,j) = packet_size(i);
%         fail(2,j) = b_s(i);
%         fail(3,j) = tb(i);
%         j=j+1;%
%       end
%     end
% %     for i=1:1:length(b_s2)
% %       if ps(i) ~= 20 && ps(i) ~= 26 && ps(i) ~= 7 && ps(i) ~= 13 && ps(i) ~= 14
% %         fail(1,j) = ps(i);
% %         fail(2,j) = b_s2(i);
% %         fail(3,j) = tb2(i);
% %         j=j+1;%
% %       end
% %     end
% %     figure(1)
% %     plot(fail(2,:),fail(1,:),'r.')
% %     axis([min(fail(2,:))-0.1 max(fail(2,:))+0.1 min(fail(1,:))-0.1 max(fail(1,:))+0.1])
%     
%     fail
%     t1=0;
%     tb1=0;
%     for i=1:1:length(fail)
%       if fail(1,i) == 11
%         
%         %if fail(3,i) - tb1 > 3400
%           fail(2,i) - t1 
%           [8*(fail(3,i) - tb1) 10*(fail(3,i) - tb1)]
%           (8*(fail(3,i) - tb1))/(fail(2,i) - t1)
%           t1=fail(2,i);
%           tb1=fail(3,i);
%         %end
%       end
%     end
%     fail(2,2:j-1)-fail(2,1:j-2)
%     fail(3,2:j-1)-fail(3,1:j-2)
    
    %% subject gps
%     try import_bag=bagReader(sprintf('../../bags/%s/%s_bag%i.bag',...
%         vehicle_type,vehicle_type,test), '/w/subject_gps');
%         pose_ns=table2array(import_bag(:,8));
%         pose_s=table2array(import_bag(:,7));
%         yaw_pose=table2array(import_bag(:,11));
%         x=table2array(import_bag(:,3));
%         y=table2array(import_bag(:,4));
%         z=table2array(import_bag(:,2));
%     catch
%         try import_bag=bagReader(sprintf('../../bags/%s/%s_bag%i.bag',...
%             vehicle_type,vehicle_type,test), '/a/subject_gps');
%             pose_ns=table2array(import_bag(:,4));
%             pose_s=table2array(import_bag(:,5));
%             yaw_pose=table2array(import_bag(:,11));
%             x=table2array(import_bag(:,7));
%             y=table2array(import_bag(:,6));
%             z=table2array(import_bag(:,1));
%         catch
%             pose_ns=0;
%             pose_s=0;
%             yaw_pose=0;
%             x=0;
%             y=0;
%             z=0;
%         end
%     end
    
    %% subject pose
%     try import_bag=bagReader(sprintf('../../bags/%s/%s_bag%i.bag',vehicle_type,...
%         vehicle_type,test), '/w/subject_pose');
%       sp_ns=table2array(import_bag(:,5));
%         sp_s=table2array(import_bag(:,4));
%         yaw_sp=table2array(import_bag(:,11));
%         x_sp=table2array(import_bag(:,6));
%         y_sp=table2array(import_bag(:,7));
%         z_sp=table2array(import_bag(:,8));
%     catch
%         try import_bag=bagReader(sprintf('../../bags/%s/%s_bag%i.bag',...
%             vehicle_type,vehicle_type,test), '/a/subject_pose');
%             sp_ns=table2array(import_bag(:,3));
%             sp_s=table2array(import_bag(:,4));
%             yaw_sp=table2array(import_bag(:,8));
%             x_sp=table2array(import_bag(:,9));
%             y_sp=table2array(import_bag(:,10));
%             z_sp=table2array(import_bag(:,11));
%         catch
%             sp_ns=0;
%             sp_s=0;
%             yaw_sp=0;
%             x_sp=0;
%             y_sp=0;
%             z_sp=0;
%         end
%     end
    
    %% rf test
%     import_bag=bagReader(sprintf('../../bags/%s/%s_bag%i.bag',vehicle_type,...
%       vehicle_type,test), '/a/ack');
%     if size(import_bag) > [1,0] 
%         if test > 1 
%             rf_ns=table2array(import_bag(:,8));
%             rf_s=table2array(import_bag(:,7));
%             bytes=table2array(import_bag(:,1));
%             seq=table2array(import_bag(:,3));
%             rssi=table2array(import_bag(:,4));
%             source=table2array(import_bag(:,2));
%         else
%             rf_ns=table2array(import_bag(:,4));
%             rf_s=table2array(import_bag(:,5));
%             bytes=table2array(import_bag(:,1));
%             seq=table2array(import_bag(:,6));
%             rssi=table2array(import_bag(:,7));
%             source=table2array(import_bag(:,8));
%         end
%     else
%       import_bag=bagReader(sprintf('../../bags/%s/%s_bag%i.bag',vehicle_type,...
%         vehicle_type,test), '/a/rssi');
%       bytes=table2array(import_bag(:,1));
%       source=table2array(import_bag(:,2));
%       seq=table2array(import_bag(:,3));
%       rssi=table2array(import_bag(:,4));
%       rf_time=table2array(import_bag(:,9));
%       rf_time=rf_time-rf_time(1);
%       rf_ns=0;
%       rf_s=0;
%     end
    
    %% rf_rx test
    import_bag=bagReader(sprintf('../../bags/%s/%s_bag%i.bag',vehicle_type,...
      vehicle_type,test), '/a/rf_rx_data');
    if size(import_bag) > [1,0] 
      %if test > 1  
        rf_rx=table2array(import_bag(:,1));
        mar=1;
        j=1;
        n=0;
        rfrx=zeros(length(rf_rx),1);
        for i=1:1:length(rf_rx)
          %w=cell2mat(cellfun(@uint8, rf_rx(i), 'UniformOutput', false))';
          w=cell2mat(rf_rx(i))';
          for k = 1:1:length(w)
            if w(k)== 126 && n > 6
              if i < length(rf_rx)-1
                v=cell2mat(rf_rx(i+1));
                if length(v) == 1
                  wv=cell2mat(rf_rx(i+2));
                  vt = zeros(1,2);
                  vt(1)=v;
                  vt(2)=wv(1);
                  v=vt;
                end
              end
              if (k < length(w)-1 && w(k+1) == 0 && abs(w(k+2) - 17) < 10 ) ||...
                  (k == length(w) && v(1) == 0 && abs(v(2) - 17) < 10) ||...
                  (k == length(w)-1 && w(k+1) == 0 && abs(v(1) - 17) < 10)
                
                if n > mar
                  mar = n;
                end
                j=j+1;
                n=1;
              else
                n=n+1;
              end
            else
              n=n+1;
            end
            %[j,n]
            rfrx(j,n)=w(k);
          end  
        end
        rfrx=rfrx(1:j,1:mar);

        %rfrx(1:100,1:14);
        [m,n] = size(rfrx)
        rf_reach=zeros(1,n-11);
        reach_data1=zeros(1,16);
        j=1;t=1;r=1;
        for i=1:1:m
          if rfrx(i,4) == 129
            for k = n:-1:13
              if rfrx(i,k) == 12 && rfrx(i,k-1) == 9 
                s = k - 2;
                break;
              end
            end
            for k = 12:1:s
              reach_data1(t,r)=rfrx(i,k);
              if r == 16
                t=t+1;
                r=0;
              end
              %[t,r]
              r=r+1;
            end
            
            rf_reach(j,:) = rfrx(i,12:n);
            j=j+1;
          end
        end
        %reach_data1(1:100,1:14)
        %rf_reach(1:100,1:14)
      %else
        %rf_rx=table2array(import_csv(:,1));
      %end
    else
      rf_rx=0;
    end
    %rea=0;
    %% rf_rx test
    import_bag=bagReader(sprintf('../../bags/%s/%s_bag%i.bag',vehicle_type,...
      vehicle_type,test), '/a/rx_data');
    if size(import_bag) > [1,0] 
      %if test > 1  
        rx=table2array(import_bag(:,1));
        mrd=1;
        j=1;
        n=1;
        rx_d=zeros(1,1);
        for i=1:1:length(rx)
          %w=cell2mat(cellfun(@uint8, rx(i), 'UniformOutput', false))';
          w=cell2mat(rx(i));
          for k = 1:1:length(w)
            if w(k) == 129
              if n > mrd
                mrd = n;
              end
              if i < length(rx)-1
                v=cell2mat(rx(i+1));
                if length(v) == 1
                  wv=cell2mat(rx(i+2));
                  vt = zeros(1,2);
                  vt(1)=v;
                  vt(2)=wv(1);
                  v=vt;
                end
              end
              if (k < length(w)-1 && w(k+1) == 0 && w(k+2) == 35) ||...
                (k == length(w) && v(1) == 0 && v(2) == 35) ||...
                (k == length(w)-1 && w(k+1) == 0 && v(1) == 35)

                j=j+1;
                n=1;
              else
                if n > 255
                  j=j+1;
                  n=0;
                end
                n=n+1;[j,n,length(rx),i,length(w),k]
              end
            else
              if n > 255
                j=j+1;
                n=0;
              end
              n=n+1;   
            end
            rx_d(j,n)=w(k);
          end  
        end
        %rx_d=rx_d(1:j,1:mrd);
        
        [m,n] = size(rx_d)
        rd_reach=zeros(1,n-8);
        reach_data2=zeros(1,16);
        j=1;t=1;r=1;
        for i=1:1:m
          if rx_d(i,3) == 35
            for k = n:-1:10
              if rx_d(i,k) == 12 && rx_d(i,k-1) == 9 
                s = k - 2;
                break;
              end
            end
            for k = 9:1:s
              reach_data2(t,r)=rx_d(i,k);
              if r == 16
                t=t+1;
                r=0;
              end
              %[t,r,i,k]
              r=r+1;
            end
            rd_reach(j,:) = rx_d(i,9:n);
            j=j+1;
          end
        end
        %reach_data2(1:100,1:14)
      %else
        %rf_rx=table2array(import_csv(:,1));
      %end
    else
      rx=0;
    end
    
    %%
    [m1, n1]=size(reach_data2); [m2,n2]= size(reach_data1);
    [m1 n1 m2 n2]
    s=0;
    for i=1:1:min(m1,m2)
      for j=1:1:16
        if reach_data2(i,j) ~= reach_data1(i,j)
          [i j reach_data2(i,j) reach_data1(i,j)]
          s=s+1;
        end
      end
      if s >= 100
        break;
      end
    end
    %sum(reach_data2(1:3333,:) - reach_data1(1:3333,:))
    %% rf_tx test
%     import_csv=bagReader(sprintf('../../bags/%s_bag%i.bag',vehicle_type,...
%       test), '/a/rf_tx_data');
%     if size(import_csv) > [1,0] 
%       %if test > 1
%         rf_tx=table2array(import_csv(:,1));
%         mft=1;
%         j=0;
%         n=1;
%         rftx=zeros(length(rf_tx),50);
%         for i=1:1:length(rf_tx)
%           w=cell2mat(rf_tx(i))';
%           for k = 1:1:length(w)
%             if w(k)== 126
%               if n > mft
%                 mft = n;
%               end
%               j=j+1;
%               n=1;
%             else
%               n=n+1;
%             end
%             rftx(j,n)=w(k);
%           end    
%         end
%         rftx=rftx(1:j,1:mft);
%         %else
%             %rf_tx=table2array(import_csv(:,1));
%         %end
%     else
%         rf_tx=0;
%     end
    
    %% reach_tx test
%     import_bag=bagReader(sprintf('../../bags/%s_bag%i.bag',vehicle_type,...
%       test), '/r/reach_tx_data');
%     if size(import_bag) > [1,0] 
%       %if test > 1
%         reach_tx=table2array(import_bag(:,1));
%         mrt=1;
% %           j=0;
%         reachtx=zeros(length(reach_tx),1);
%         for i=1:1:length(reach_tx)
%           w=cell2mat(cellfun(@uint8, reach_tx(i), 'UniformOutput', false));
%           n=1;
%           for k = 1:1:length(w)
%             reachtx(i,n)=w(k);
%             n=n+1;
%           end    
%         end
%         %reachtx=reachtx(1:i,:);%reachtx(:,1:14)
%       %else
%         %reach_tx=table2array(import_csv(:,1));
%       %end
%       rea=1;
%     else
%       rea=0;
%       reach_tx=0;
%     end
    %% nshp_tx test
%     import_bag=bagReader(sprintf('../../bags/%s_bag%i.bag',vehicle_type,...
%       test), '/n/nshp_tx_data');
%     if size(import_bag) > [1,0] 
%       %if test > 1
%         nshp_tx=table2array(import_bag(:,1));
%         mrt=1;
%         j=1;
%         n=1;
%         nshptx=zeros(1,1);
%         for i=1:1:length(nshp_tx)
%           w=cell2mat(cellfun(@uint8, nshp_tx(i), 'UniformOutput', false));
%           for k = 1:1:length(w)
%             if w(k) == 36
%               j = j + 1;
%               n=1;
%             end
%             nshptx(j,n)=w(k);
%             n=n+1;
%           end    
%         end
%         nshptx(1:10,1:28)
%       %else
%         %reach_tx=table2array(import_csv(:,1));
%       %end
%       nea=1;
%     else
%       nea=0;
%       nshp_tx=0;
%     end
    
    %% reach_rx test
%     import_csv=bagReader(sprintf('../../bags/%s_bag%i.bag',vehicle_type,...
%       test), '/r/reach_rx_data');
%     if size(import_csv) > [1,0] 
%       %if test > 1
%         reach_rx=table2array(import_csv(:,1));
%         mrr=1;
%         j=1;
%         n=1;
%         reachrx=zeros(1,1);
%         buf=0;
%         for i=1:1:length(reach_rx)
%           w=cell2mat(cellfun(@uint8, reach_rx(i), 'UniformOutput', false));
%           for k = 1:1:length(w)
%             if w(k) == 211
%               buf = w(k);
%             elseif buf == 211
%               if w(k) == 0
%                 n=2;
%                 reachrx(j,1)=211;
%                 reachrx(j,2)=0;
%                 j=j+1;
%               else
%                 reachrx(j,n+1)=211;
%                 reachrx(j,n+2)=w(k);
%                 n=n+2;
%               end
%               buf = 0;
%             else
%               reachrx(j,n)=w(k);
%               n=n+1;
%             end                    
%           end
%           if n > mrr
%             mrr = n-1;%reachrx(j,1:n-1);
%           end
%         end
%         %size(reachrx)
%         %reachrx(:,1:14)
%       %else
%         %reach_tx=table2array(import_csv(:,1));
%       %end
%       rea=1;
%     else
%       reach_rx=0;
%       rea=0;
%     end
    
    %% nshp_rx test
%     import_csv=bagReader(sprintf('../../bags/%s_bag%i.bag',vehicle_type,...
%       test), '/n/nshp_rx_data');
%     if size(import_csv) > [1,0] 
%       %if test > 1
%         nshp_rx=table2array(import_csv(:,1));
%         mrr=1;
%         j=1;
%         n=1;
%         nshprx=zeros(1,1);
%         buf=0;
%         for i=1:1:length(nshp_rx)
%           w=cell2mat(cellfun(@uint8, nshp_rx(i), 'UniformOutput', false));
%           for k = 1:1:length(w)
% %             if w(k) == 211
% %               buf = w(k);
% %             elseif buf == 211
% %               if w(k) == 0
% %                 n=2;
% %                 nshprx(j,1)=211;
% %                 nshprx(j,2)=0;
% %                 j=j+1;
% %               else
% %                 nshprx(j,n+1)=211;
% %                 nshprx(j,n+2)=w(k);
% %                 n=n+2;
% %               end
% %               buf = 0;
% %             else
%               nshprx(i,k)=w(k);
%               n=n+1;
% %             end                    
%           end
%           if n > mrr
%             mrr = n-1;%reachrx(j,1:n-1);
%           end
%         end
%         %size(reachrx)
%         nshprx(:,1:14)
%       %else
%         %reach_tx=table2array(import_csv(:,1));
%       %end
%       nsh=1;
%     else
%       nshp_rx=0;
%       nsh=0;
%     end
     %% device_rx test
    import_bag=bagReader(sprintf('../../bags/%s/%s_bag%i.bag',vehicle_type,...
      vehicle_type,test), '/a/device_rx_data');
    gx=0;gy=0;
    if size(import_bag) > [1,0] 
      %if test > 1  
        device_rx=table2array(import_bag(:,1));
        m=1;
        n=1;
        nmea=zeros(1,1);
        if rea == 1
          ub = [1 2 4 5 6 9 10 11 13 16 19 33 39 40];
          ub5 = [0 1];
          ub1 = [1 2 3 4 5 6 7 9 16 17 18 32 33 34 35 36 37 38 48 49 50 52 53 57 96 97];
          ub2 = [19 20 21 32 65 89 97];
          ub4 = [0 1 2 3 4];
          ub6 = [0 1 2 4 6 8 9 17 19 22 23 27 30 35 36 49 52 57 59 61 62 71 83 87 92 96 97 98 105 132 133 134];
          ub9 = 20;
          ub10 = [2 4 6 7 8 9 11 33 39 40 46];
          ub11 = [1 2 48 49 51];
          ub13 = [1 3 4 6 17 18 19 21 22 23];
          ub16 = [2 3 16 21];
          ub19 = [0 2 3 5 6 32 33 64 96 128];
          ub33 = [3 4 7 8 9 11 13 14 15];
          ub39 = [1 3];
          ub40 = 0;
          for i=1:1:length(device_rx)
            w=cell2mat(cellfun(@uint8, device_rx(i), 'UniformOutput', false));

            for k = 4:1:length(w)-2
              if w(k) == 181
                if i < length(device_rx)
                  v=cell2mat(cellfun(@uint8, device_rx(i+1), 'UniformOutput', false));
                else
                  v=[-1 -1 -1 -1 -1];
                end
                if w(k+1) == 98 || (w(k+1) ~= 98 && v(4) == 98)
                  if k < length(w)-4 && ismember(w(k+2),ub)
                    ubc = w(k+2);
                    ubid = w(k+3);
                    %[ubc ubid]
                  elseif k == length(w)-4 && ismember(w(k+2),ub)
                    ubc = w(k+2);
                    ubid = v(4);
                  elseif k == length(w)-3 && ismember(v(4),ub)
                    ubc = v(4);
                    ubid = v(5);
                  elseif k == length(w)-2 && ismember(v(5),ub)
                    ubc = v(5);
                    ubid = v(6);
                  else
                    ubc = -1;
                    ubid = -1;
                  end
                  switch ubc
                    case 1
                      if ismember(ubid,ub1)
                        n = n + 1;
                        m = 1;
                      end
                    case 2
                      if ismember(ubid,ub2)
                        n = n + 1;
                        m = 1;
                      end
                    case 4
                      if ismember(ubid,ub4)
                        n = n + 1;
                        m = 1;
                      end
                    case 5
                      if ismember(ubid,ub5)
                        n = n + 1;
                        m = 1;
                      end
                    case 6
                      if ismember(ubid,ub6)
                        n = n + 1;
                        m = 1;
                      end
                    case 9
                      if ismember(ubid,ub9)
                        n = n + 1;
                        m = 1;
                      end
                    case 10
                      if ismember(ubid,ub10)
                        n = n + 1;
                        m = 1;
                      end
                    case 11%1 2 4 5 6 9 10 11 13 16 19 33 39 40
                      if ismember(ubid,ub11)
                        n = n + 1;
                        m = 1;
                      end
                    case 13
                      if ismember(ubid,ub13)
                        n = n + 1;
                        m = 1;
                      end
                    case 16
                      if ismember(ubid,ub16)
                        n = n + 1;
                        m = 1;
                      end
                    case 19
                      if ismember(ubid,ub19)
                        n = n + 1;
                        m = 1;
                      end
                    case 33
                      if ismember(ubid,ub33)
                        n = n + 1;
                        m = 1;
                      end
                    case 39
                      if ismember(ubid,ub39)
                        n = n + 1;
                        m = 1;
                      end
                    case 40
                      if ismember(ubid,ub40)
                        n = n + 1;
                        m = 1;
                      end
                    otherwise
                      %
                   end
                end
              end
              nmea(n,m)=w(k);
              m=m+1;
            end  
          end
          %nmea(:,1:14)
          nmea=nmea(2:n,:);%size(nmea)
%           gps_estimate=zeros(1,7);
%           lh=1;
%           ref_t = 0;
%           for i=1:1:n-1
%             if nmea(i,3) == 1 && nmea(i,4) == 2 && nmea(i,5) == 28 && nmea(i,6) == 0
%               rtime = (nmea(i,10)*2^24+nmea(i,9)*2^16+...
%                 nmea(i,8)*2^8+nmea(i,7));
%               lon =((255-nmea(i,14))*2^24+(255-nmea(i,13))*2^16+...
%                 (255-nmea(i,12))*2^8+(255-nmea(i,11)))*1e-7;
%               lat = ((nmea(i,18))*2^24+(nmea(i,17))*2^16+...
%                 (nmea(i,16))*2^8+(nmea(i,15)))*1e-7;
%               altitude = (nmea(i,22)*2^24+nmea(i,21)*2^16+...
%                   nmea(i,20)*2^8+nmea(i,19))*1e-3;
%               amsl = (nmea(i,26)*2^24+nmea(i,25)*2^16+...
%                   nmea(i,24)*2^8+nmea(i,23))*1e-3;
%               if rtime-ref_t > 0 && abs(-1*lon - ref_x) < 0.1 && abs(lat - ref_y) < 0.1 && abs(altitude - ref_z) < 250 && abs(amsl - ref_z) < 250
%                 if lh == 1
%                   ref_x=-1*lon;
%                   ref_y=lat;
%                   ref_t = rtime;
%                 end
%                 gps_estimate(lh,7)=(rtime - ref_t)/1000;
%                 % longitude WGS84
%                 lotm = (111412.84*cos(1/2*(lat-ref_y))-93.5*cos(3/2*(lat-ref_y))-0.118*cos(5/2*(lat-ref_y)));
%                 gps_estimate(lh,1)=-1*lon;
%                 gps_estimate(lh,5)=(-1*lon-ref_x)*lotm;
%                 % latitude WGS84
%                 latm = (111132.92 - 559.82*cos(1*(lat-ref_y))+1.175*cos(2*(lat-ref_y))-...
%                   0.0023*cos(3*(lat-ref_y)));
%                 gps_estimate(lh,2)=lat;
%                 gps_estimate(lh,6)=(lat-ref_y)*latm;
%                 gps_estimate(lh,3)=altitude;
%                 gps_estimate(lh,4)=amsl;
%                 lh=lh+1;
%               end
%             end
%           end
%           %gps_estimate
%           gx=gps_estimate(:,5);
%           gy=gps_estimate(:,6);
%           gt=gps_estimate(:,7);
        else
          ne=1;
          for i=1:1:length(device_rx)
            w=cell2mat(cellfun(@uint8, device_rx(i), 'UniformOutput', false))';
            if i < length(device_rx)
              v=cell2mat(cellfun(@uint8, device_rx(i+1), 'UniformOutput', false))';
            else
              v=[-1 -1 -1 -1 -1];
            end
            for k = 4:1:length(w)-2
%               if n == 257
%                 m = m+1;
%                 n=1;
%               end
              if length(v) > 5 && k==length(w)-2
                nshp1 = v(4);
                nshp2 = v(5);
              elseif length(v) > 5 && k==length(w)-3
                nshp2 = v(4);
                nshp1 = w(k+1);
              else
                nshp2 = w(k+2);
                nshp1 = w(k+1);
              end
              if w(k) == 13 && nshp1 == 10 && nshp2 == 36
                %ne
                m=m+1;
                ne=1;
              end
              nmea(m,ne)=w(k);
              n=n+1;
              ne =ne+1;
            end
          end
          %nmea(:,1:28)
          [m,n]=size(nmea);
          nmtime=zeros(1,12);
          lat=zeros(1,12);
          lon=zeros(1,12);
          altitude=zeros(1,12);
          nmea_c=({'ga'  'pd'; 'at'  'df'});
          j=1;
          c=1;
          a=1;b=1;
          for i = 1:1:m
            a=1;
            for k = 1:1:n
              if nmea(i,k) == 44 || nmea(i,k) == 36 || nmea(i,k) == 42
                if j == 21
                  c=c+1;j=1;
                end
                b=k;
                nmea_c(c,j)=({sprintf('%s',char(nmea(i,a:b-1)))});
                j=j+1;
                a=k+1;
              end
              %if nmea(i,4) == 71 && nmea(i,5) == 80 && nmea(i,6) == 71 && nmea(i,7) == 71 && nmea(i,8) == 65
%                 nmtime(j,:) = nmea(i,a:b-1);
%                 lat(j,:) = nmea(i,20:29);
%                 lon(j,:) = nmea(i,32:42);
%                 altitude(j,:) = nmea(i,52:56);
                %j=j+1;
              %end
            end
          end
          nmea_c(1:10,1:20)
          %char(altitude)
          %[char(nmtime) char(lat) char(lon) char(altitude)]
          %char(nmea)
        end
    
      %else
        %device_rx=table2array(import_csv(:,1));
      %end
    else
      device_rx=0;
    end
    
    %% gps_pose test
    import_bag=bagReader(sprintf('../../bags/%s/%s_bag%i.bag',vehicle_type,...
      vehicle_type,test), '/r/reach_gps');
    gx=0;gy=0;gt=0;gz=0;gz2=0;
    if size(import_bag) > [1,0] 
      %if test > 1
      reach_amsl=table2array(import_bag(:,1));
      reach_alt=table2array(import_bag(:,2));
      reach_long=table2array(import_bag(:,3));
      %reach_time2=table2array(import_bag(:,4));
      reach_lat=table2array(import_bag(:,5));
      reach_vacc=table2array(import_bag(:,6));
      reach_hacc=table2array(import_bag(:,7));
      reach_time=table2array(import_bag(:,12));
      
      reach_time=reach_time-reach_time(1);
      
      j=1;
      for i=1:1:length(reach_time)
        if (j == 1 && abs(reach_long(i) - ref_x) < 1.0 && abs(reach_lat(i) - ref_y) < 1.0) ||...
          (abs(reach_long(i) - ref_x) < 0.001 && abs(reach_lat(i) - ref_y) < 0.001)
          if j == 1
            ref_x = reach_long(i);
            ref_y = reach_lat(i);
          end
          lotm = (111412.84*cos(1/2*(reach_lat(i)-ref_y))-93.5*cos(3/2*(reach_lat(i)-ref_y))-...
            0.118*cos(5/2*(reach_lat(i)-ref_y)));
          latm = (111132.92 - 559.82*cos(1*(reach_lat(i)-ref_y))+1.175*cos(2*(reach_lat(i)-ref_y))-...
            0.0023*cos(3*(reach_lat(i)-ref_y)));
          gx(j)=(reach_long(i)-ref_x)*lotm;
          gy(j)=(reach_lat(i)-ref_y)*latm;
          gt(j)=reach_time(i);
          if abs(reach_alt(i) - ref_z) > 50 
            gz(j)=ref_z;
          else
            gz(j)=reach_alt(i);
          end
          if abs(reach_amsl(i) - ref_z) > 50 
            gz2(j)=ref_z;
          else
            gz2(j)=reach_amsl(i);
          end
          j=j+1;
        end
      end

      %else
            %device_tx=table2array(import_csv(:,1));
        %end
    else
      reach_amsl=0;
      reach_alt=0;
      reach_long=0;
      %reach_time=0;
      reach_lat=0;
      reach_vacc=0;
      reach_hacc=0;
      reach_time=0;
    end
    
    %% laser reading
%     import_csv=genericExtractor(sprintf('../csv/%s/%s/%s%i/sensor_%s%i.csv',...
%         vehicle_type,vehicle,vehicle,test,vehicle,test),true);
%     if size(import_csv) > [0,0]
%         
%         if test == 6
%             lp_ns=table2array(import_csv(:,5));
%             lp_s=table2array(import_csv(:,6));
%         else
%             lp_ns=table2array(import_csv(:,4));
%             lp_s=table2array(import_csv(:,5));
%         end
%         if test > 17
%             laser_percent=table2array(import_csv(:,6));
%             laser=table2array(import_csv(:,8));
%         elseif test > 1 && test < 4
%             laser=table2array(import_csv(:,8));
%         elseif test == 6
%             laser=table2array(import_csv(:,9));
%             laser_percent=table2array(import_csv(:,7));
%             laser_dist=table2array(import_csv(:,1));
%         else
%             laser=table2array(import_csv(:,7));
%             
%         end
%     else
%         lp_ns=0;
%         lp_s=0;
%         laser=0;
%     end
   
    %% time conversion and sizes
    % find first time as reference 0
    if size(pose_ns) > [1,0]
%       [start_s,start_ns] = minTime(rf_s(1,1),rf_ns(1,1),rf_s(1,1),rf_ns(1,1));
        [start_s,start_ns] = minTime(pose_s(1,1),pose_ns(1,1),rf_s(1,1),rf_ns(1,1));
    else
        [start_s,start_ns] = minTime(rf_s(1,1),rf_ns(1,1),rf_s(1,1),rf_ns(1,1));
    end
    if size(sp_ns) > [1,0]
        [start_s,start_ns] = minTime(start_s,start_ns,sp_s(1,1),sp_ns(1,1));
    end
    % convert time vectors from unix time to time from start
    [mp,pose_time] = unixTime2TestTime(start_s,start_ns,pose_s',pose_ns');
    [mrf,rf_time2] = unixTime2TestTime(start_s,start_ns,rf_s',rf_ns');
    [msp,sp_time] = unixTime2TestTime(start_s,start_ns,sp_s',sp_ns');
%     [mlp,lp_time] = unixTime2TestTime(start_s,start_ns,lp_s',lp_ns');
     if pose_time(mp) > 0
        ttime = ttime + pose_time(mp);
     end
     begin_flight=ones(4,1);
     end_flight = ones(4,1);
     end_flight(:,1) = mp;
     k=1;
     for j=1:1:4
         for i=k:1:mp
             if z(i) > z(1) + 0.5 && begin_flight(j) == 1
                 begin_flight(j) = i;
                 break;
             end
         end
         for k=i:1:mp
             if z(k) < z(1) + 0.5 && end_flight(j) == mp
                 end_flight(j) = k;
                 break;
             end
         end
     end
     %[test begin_flight end_flight]
%      [pose_time(begin_flight) pose_time(end_flight) pose_time(end_flight) - pose_time(begin_flight)]
    for j=1:1:4
        ftime = ftime + pose_time(end_flight(j)) - pose_time(begin_flight(j));
    end
%     [test] 
%     [(pose_time(end_flight) - pose_time(begin_flight))/60 ]
    %% lat/lon --> meters
% %     if test == 14
        ref_x = -96.5984559;
        ref_y = 40.8583787;
% %     end
    
    h_g=0;
    for i=1:1:mp
      latm = (111132.92 - 559.82*cos(2*(y(i)-ref_y))+1.175*cos(4*(y(i)-ref_y))-0.0023*cos(6*(y(i)-ref_y)));
      lotm = (111412.84*cos(1*(y(i)-ref_y))-93.5*cos(3*(y(i)-ref_y))-0.118*cos(5*(y(i)-ref_y)));
      x(i) = (x(i) - ref_x)*lotm;
      y(i) = (y(i) - ref_y)*latm;
        h_g(i) = sqrt(x(i)^2+y(i)^2);%*12*2.54/10;
%         if x(i) < 0 %|| y(i) < 0
%             h_g=h_g*-1;
%         end
    end
%     x=x*1.2*2.54;
%     y=y*1.2*2.54;
%     z=z*1.2*2.54;
    d_g=sqrt(x.^2+y.^2+z.^2);
   
    h = sqrt(x_sp.^2+y_sp.^2);
    %% rssi dropout
    d_rssi=zeros(1,4);
    j=1;
    for i=2:1:mrf
        if rf_time(i) - rf_time(i-1) > 3.0
            k=sync_time(rf_time,pose_time,i-1);
            d_rssi(j,1) = d_g(k);
            d_rssi(j,2) = z(k);
            d_rssi(j,3) = h_g(k);
            d_rssi(j,4) = pose_time(k);
            j=j+1;
            k=sync_time(rf_time,pose_time,i);
            d_rssi(j,1) = d_g(k);
            d_rssi(j,2) = z(k);
            d_rssi(j,3) = h_g(k);
            d_rssi(j,4) = pose_time(k);
            j=j+1;
        end
    end
    
    k=sync_time(rf_time,pose_time,mrf);
    d_rssi(j,1) = d_g(k);
    d_rssi(j,2) = z(k);
    d_rssi(j,3) = h_g(k);
    d_rssi(j,4) = pose_time(k);
    
    %% position Plot
    if view_plot == 1
        % 1-D position with respect to time
        figure
        %subplot(2,1,1),
        plot(gx,gy,'b.') 
        hold on
        grid on
        grid minor
        title('pose')
        xlabel('Pose East of spot in green space (meters)')
        ylabel('Pose North of spot in green space (meters)')
        axis([min(gx)-0.01 max(gx)+0.01 min(gy)-0.01 max(gy)+0.01])
        %legend('X Position','Y Position','Z Position')
        hold off
        
        if save_plot == 1
            print(sprintf('../../plot/%s/%s%i/reach_gps%s%i',...
                vehicle_type,vehicle,test,vehicle,test),'-djpeg')
        end
        
        figure
        plot3(gx,gy,gz,'r.') % elipsoidal
        hold on 
        plot3(gx,gy,gz2,'b.') % above mean sea level
        %plot3(x(mp),y(mp),z(mp),'k<')
        grid on
        grid minor
        axis([min(gx)-0.01 max(gx)+0.01 min(gy)-0.01 max(gy)+0.01 min(gz)-0.01 max(gz2)+0.01])
        title(sprintf('Pose estimate of %s%i',vehicle,test))
        xlabel('Pose East of spot in green space (meters)')
        ylabel('Pose North of spot in green space (meters)')
        zlabel('Height above ellipsoid (meters)')
        
        if save_plot == 1
            print(sprintf('../../plot/%s/%s%i/reach_ellipsoid%s%i',...
                vehicle_type,vehicle,test,vehicle,test),'-djpeg')
        end
        
        figure
        plot3(gx,gy,gt,'r.') % subject gps
        hold on 
        %plot3(reach_long,reach_lat,gt,'b.') % task waypose
        %plot3(x(mp),y(mp),z(mp),'k<')
        grid on
        grid minor
        axis([min(gx)-0.01 max(gx)+0.01 min(gy)-0.01 max(gy)+0.01 min(gt)-0.01 max(gt)+0.01])
        title(sprintf('Pose estimate of %s%i',vehicle,test))
        xlabel(sprintf('Pose East of %f (meters)',ref_x))
        ylabel(sprintf('Pose North of %f (meters)',ref_y))
        zlabel('time (seconds)')
        
        if save_plot == 1
            print(sprintf('../../plot/%s/%s%i/reach_llt%i',...
                vehicle_type,vehicle,test,test),'-djpeg')
        end

    %     %[msp,mrf]
    % 3-D position

        figure
        plot3(x,y,z,'r>') % subject gps
        hold on 
        plot3(x(1),y(1),z(1),'b<') % task waypose
        plot3(x(mp),y(mp),z(mp),'k<')
        grid on
        grid minor
        axis([min(x)-0.01 max(x)+0.01 min(y)-0.01 max(y)+0.01 min(z)-0.01 max(z)+0.01])
        title(sprintf('Path of %s%i',vehicle,test))
        xlabel('Position in X Direction (meters)')
        ylabel('Position in Y Direction (meters)')
        zlabel('Position in Z Direction (meters)')
        %legend('Actual Path', 'Ideal Path')

        % print
        if save_plot == 1
            print(sprintf('../../plot/%s/%s%i/path_%s%i',...
                vehicle_type,vehicle,test,vehicle,test),'-djpeg')
        end

        % rssi
        figure
        %subplot(2,1,1),
        plot(rf_time,-rssi,'b.') 
        hold on
        grid on
        grid minor
        title('RSSI')
        xlabel('Time (seconds)')
        ylabel('RSSI (dBm)')
        %legend('X Position','Y Position','Z Position')
        hold off
        %subplot(2,1,2),plot(seq,-rssi,'r.')
        
        % print figure
        if save_plot == 1
            print(sprintf('../../plot/%s/%s%i/rssi_%s%i',...
                vehicle_type,vehicle,test,vehicle,test),'-djpeg')
        end

        % 1-D position with respect to time
        figure
        %subplot(2,1,1),
        hold on
        [hAx,h1,h2] = plotyy(pose_time,z,rf_time,-rssi);
        [hAx2,h3,h4] = plotyy(d_rssi(:,4),d_rssi(:,2),mean(rf_time),mean(-rssi));
        grid on
        grid minor
        axis(hAx(1),[pose_time(1)-0.01 pose_time(mp)+0.01 min(z)-0.1 max(z)+0.1])
        axis(hAx(2),[pose_time(1)-0.01 pose_time(mp)+0.01 min(-rssi)-0.1 max(-rssi)+0.1])
        axis(hAx2(1),[pose_time(1)-0.01 pose_time(mp)+0.01 min(z)-0.1 max(z)+0.1])
        axis(hAx2(2),[pose_time(1)-0.01 pose_time(mp)+0.01 min(-rssi)-0.1 max(-rssi)+0.1])
        h2.Marker = '.';
        h1.Color = 'red';
        h1.Marker = '.';
        h2.Color = 'blue';
        h2.LineStyle = 'none';
        h4.Marker = '.';
        h4.Color = 'white';
        h4.LineStyle = 'none';
        h3.Marker = 'x';
        h3.Color = 'black';
        h3.LineStyle = 'none';
        ylabel(hAx(1),'meters') % left y-axis
        ylabel(hAx(2),'RSSI') % right y-axis
        
        title('RSSI vs Position')
        xlabel('Time (seconds)')

        % print figure
        if save_plot == 1
            print(sprintf('../../plot/%s/%s%i/altitude_%s%i',...
                vehicle_type,vehicle,test,vehicle,test),'-djpeg')
        end
        
%         % 1-D position with respect to time
        figure
        %subplot(2,1,1),
        hold on
        grid on
        grid minor
%         %[size(sp_time) size(z_sp) size(h)]
        plot(h_g,z,'r>')
        j=1;
        for i=1:1:mp
            if j < mrf + 1 && rf_time(j) - pose_time(i) < 0.3
                plot(h_g(i),z(i),'ko')
                j=j+1;
            end
        end
        title('RSSI vs Position')
        xlabel('Horizontal Distance(meters)')
        ylabel('Vertical Distance (meters)')
        legend('UAV Path', 'Packet Received')
% 
%         % print figure
        if save_plot == 1
            print(sprintf('../../plot/%s/%s%i/hvsv_%s%i',...
                vehicle_type,vehicle,test,vehicle,test),'-djpeg')
        end
        
        % 1-D position with respect to time
        figure
        %subplot(2,1,1),
        hold on
        [hAx,h1,h2] = plotyy(pose_time,h_g,rf_time,-rssi);
        [hAx2,h3,h4] = plotyy(d_rssi(:,4),d_rssi(:,3),mean(rf_time),mean(-rssi));
        h4.Marker = '.';
        h4.Color = 'white';
        h4.LineStyle = 'none';
        h3.Marker = 'x';
        h3.Color = 'black';
        h3.LineStyle = 'none';
        grid on
        
        axis(hAx(1),[pose_time(1)-0.01 pose_time(mp)+0.01 min(h_g)-0.1 max(h_g)+0.1])
        axis(hAx(2),[pose_time(1)-0.01 pose_time(mp)+0.01 min(-rssi)-0.1 max(-rssi)+0.1])
        axis(hAx2(1),[pose_time(1)-0.01 pose_time(mp)+0.01 min(h_g)-0.1 max(h_g)+0.1])
        axis(hAx2(2),[pose_time(1)-0.01 pose_time(mp)+0.01 min(-rssi)-0.1 max(-rssi)+0.1])
        grid minor
        h2.Marker = '.';
        h1.Color = 'red';
        h1.Marker = '.';
        h2.Color = 'blue';
        h2.LineStyle = 'none';
        ylabel(hAx(1),'meters') % left y-axis
        ylabel(hAx(2),'RSSI') % right y-axis
        
        title('RSSI vs Position')
        xlabel('Time (seconds)')

        % print figure
        if save_plot == 1
            print(sprintf('../../plot/%s/%s%i/horizontal_gps_%s%i',...
                vehicle_type,vehicle,test,vehicle,test),'-djpeg')
        end
        
        % 1-D position with respect to time
        figure
        hold on
        [hAx,h1,h2] = plotyy(pose_time,d_g,rf_time,-rssi);
        [hAx2,h3,h4] = plotyy(d_rssi(:,4),d_rssi(:,1),mean(rf_time),mean(-rssi));
        axis(hAx2(1),[pose_time(1)-0.01 pose_time(mp)+0.01 min(d_g)-0.1 max(d_g)+0.1])
        axis(hAx2(2),[pose_time(1)-0.01 pose_time(mp)+0.01 min(-rssi)-0.1 max(-rssi)+0.1])
        h4.Marker = '.';
        h4.Color = 'white';
        h4.LineStyle = 'none';
        h3.Marker = 'x';
        h3.Color = 'black';
        h3.LineStyle = 'none';
        grid on
        axis(hAx(1),[pose_time(1)-0.01 pose_time(mp)+0.01 min(d_g)-0.1 max(d_g)+0.1])
        grid minor
        axis(hAx(2),[pose_time(1)-0.01 pose_time(mp)+0.01 min(-rssi)-0.1 max(-rssi)+0.1])
        h2.Marker = '.';
        h1.Color = 'red';
        h1.Marker = '.';
        h2.Color = 'blue';
        h2.LineStyle = 'none';
        ylabel(hAx(1),'meters') % left y-axis
        ylabel(hAx(2),'RSSI') % right y-axis
        grid on
        grid minor
        title('RSSI vs Position')
        xlabel('Time (seconds)')

        % print figure
        if save_plot == 1
            print(sprintf('../../plot/%s/%s%i/distance_gps_%s%i',...
                vehicle_type,vehicle,test,vehicle,test),'-djpeg')
        end
    end
end
% [ttime/3600 ftime/3600]