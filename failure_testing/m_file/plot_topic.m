function [flight_stats,fail_stats]=plot_topic(test_set,vehicle,start_test,end_test,save_plot,view_plot)
%% Plot Flight Testing UAV Statistics
if save_plot == 1
    mkdir(sprintf('../plot/%s/',test_set))
end

% max topic data
nsp = 40000;
nscs = nsp/100;
nss = nscs;
nv = nsp;
nqci = nsp/10;
nse = nsp/10;

% results matrix
flight_stats = zeros(14,4);
fail_stats = zeros(6,4);

% intermediate matrices
failed = 0;
fail_type = zeros(5,4);
n_test = end_test - start_test + 1;
oap = zeros(n_test,3);
task_power = zeros(n_test,2);
pmax = oap;
ot = zeros(n_test,3);
precision = zeros(n_test,5);
packet_delay = zeros(2,6);
mtbf = zeros(3,n_test);
control_packet = zeros(1,6);
fprintf('%s\n',vehicle);
for test = start_test:1:end_test
    test_index = test - start_test + 1;
    fail = 0;
   
    close all
    if save_plot == 1
        mkdir(sprintf('../plot/%s/%s%i/',test_set,vehicle,test))
    end
    % data version difference
    if (strcmp(vehicle,'herbie') == 1 && test > 20) || strcmp(vehicle,'morpheus') == 1
        v2 = 1;
        begin_task2 = zeros(1,2);
    elseif (strcmp(vehicle,'hulk_smash') == 1 && test > 3) || (strcmp(vehicle,'jenny') == 1 && test > 110)
        v2 = 2;
    else
        v2 = 0;
    end
    % import data from csv files
    [status_s,status_ns,voltage,motors_status,serial_mode,...% subject status
        waypose_status]= importfile_csv_multiple(sprintf('../csv/%s/%s%i/status_%s%i.csv',...
        test_set,vehicle,test,vehicle,test),2,nss,3);
%     [r_status_s,r_status_ns,r_voltage,r_motors_status,r_serial_mode,...% subject status
%         r_waypose_status]= importfile_csv_multiple(sprintf('../csv/%s/%s%i/robot_status_%s%i.csv',...
%         test_set,vehicle,test,vehicle,test),2,nss,3);
    [sp_s,sp_ns,x,y,z,rotation_sp]= importfile_csv_multiple(sprintf('../csv/%s/%s%i/position_%s%i.csv',...
        test_set,vehicle,test,vehicle,test),2,nsp,1); % subject pose
    [task_s,task_ns,x_t,y_t,z_t,r_t]= importfile_csv_multiple(sprintf('../csv/%s/%s%i/task_%s%i.csv',...
        test_set,vehicle,test,vehicle,test),2,nscs,1); % task waypose
    [vicon_s,vicon_ns,r_x,r_y,r_z,x_v,y_v,... % vicon
        z_v,r_w]= importfile_csv_multiple(sprintf('../csv/%s/%s%i/vicon_%s%i.csv',...
        test_set,vehicle,test,vehicle,test),2,nv,4);
    [qci_s,qci_ns,pitch,roll,thrust,rotation_qci,flying,... % Quad Control Input
        control_qci] = importfile_csv_multiple(sprintf('../csv/%s/%s%i/control_input_%s%i.csv',...
        test_set,vehicle,test,vehicle,test),2,nqci,5);
    [state_s,state_ns,state] = importfile_csv_multiple(sprintf('../csv/%s/%s%i/state_%s%i.csv',...
        test_set,vehicle,test,vehicle,test),2,nscs,2); % state
    if strcmp(vehicle,'morpheus') == 0 || (strcmp(vehicle,'morpheus') == 1 && test > 17)
        [command_state_s,command_state_ns,command_state] = importfile_csv_multiple(sprintf('../csv/%s/%s%i/command_state_%s%i.csv',...
            test_set,vehicle,test,vehicle,test),2,nscs,2); % state
    else
        command_state_s = 0; 
        command_state_ns = 0;
        command_state = 0;
    end
    if v2 == 1 % voltage and current
        [volt1_s,volt1_ns,voltage1,power_task,volt1_flying] =...
            importfile_csv_multiple(sprintf('../csv/%s/%s%i/sensor1_%s%i.csv',...
            test_set,vehicle,test,vehicle,test),2,nse,6);
        [volt2_s,volt2_ns,voltage2,power_task,volt2_flying] =...
            importfile_csv_multiple(sprintf('../csv/%s/%s%i/sensor3_%s%i.csv',...
            test_set,vehicle,test,vehicle,test),2,nse,6);
        [current_s,current_ns,current1,power_task,current1_flying] =...
            importfile_csv_multiple(sprintf('../csv/%s/%s%i/sensor2_%s%i.csv',...
            test_set,vehicle,test,vehicle,test),2,nse,6);
        [current_s,current_ns,current2,power_task,...
            current2_flying] = importfile_csv_multiple(sprintf('../csv/%s/%s%i/sensor4_%s%i.csv',...
            test_set,vehicle,test,vehicle,test),2,nse,6);
    elseif v2 == 2
        [power_s,power_ns,voltage1,voltage2,current1,current2,...
            power_task,power_flying] = importfile_csv_multiple(sprintf('../csv/%s/%s%i/sensor_%s%i.csv',...
            test_set,vehicle,test,vehicle,test),2,nse,7);
    else
        voltage1 = importfile_csv_multiple(sprintf('../csv/%s/%s%i/sensor1_%s%i.csv',...
            test_set,vehicle,test,vehicle,test),2,nse,0);
        voltage2 = importfile_csv_multiple(sprintf('../csv/%s/%s%i/sensor3_%s%i.csv',...
            test_set,vehicle,test,vehicle,test),2,nse,0);
        current1 = importfile_csv_multiple(sprintf('../csv/%s/%s%i/sensor2_%s%i.csv',...
            test_set,vehicle,test,vehicle,test),2,nse,0);
        current2 = importfile_csv_multiple(sprintf('../csv/%s/%s%i/sensor4_%s%i.csv',...
            test_set,vehicle,test,vehicle,test),2,nse,0);
    end

    %% topic sizes and time conversion
    no_data = 0;
    % blocked cmmunication, outgoing communication was blocked from 
    % controlling laptop by other device 
    if size(status_s) == [0,1]
        no_data = 1;
        fail = 1;
        status_s = zeros(1,1);
        status_ns = zeros(1,1);
    end
    if size(vicon_s) == [0,1]
        vicon_s = zeros(1,1);
        vicon_ns = zeros(1,1);
    end
    if size(sp_s) == [0,1]
        sp_s = zeros(1,1);
        sp_ns = zeros(1,1);
    end
    if size(state_s) == [0,1]
        state_s = zeros(1,1);
        state_ns = zeros(1,1);
    end
    if size(task_s) == [0,1]
        task_s = zeros(1,1);
        task_ns = zeros(1,1);
    end
    if size(qci_s) == [0,1]
        qci_s = zeros(1,1);
        qci_ns = zeros(1,1);
    end
    if size(command_state_s) == 0
        command_state_s = zeros(1,1);
        command_state_ns = zeros(1,1);
    elseif size(command_state_s) == [0,1]
        command_state_s = zeros(1,1);
        command_state_ns = zeros(1,1);
    end
    
    % time conversion and sizes
    % find first time as reference 0    
    [start_s,start_ns] = minTime(vicon_s(1,1),vicon_ns(1,1),status_s(1,1),status_ns(1,1));
    [start_s,start_ns] = minTime(start_s,start_ns,sp_s(1,1),sp_ns(1,1));
    [start_s,start_ns] = minTime(start_s,start_ns,state_s(1,1),state_ns(1,1));
    [start_s,start_ns] = minTime(start_s,start_ns,task_s(1,1),task_ns(1,1));
    if strcmp(vehicle,'morpheus') == 1
        [start_s,start_ns] = minTime(start_s,start_ns,qci_s(1,1),qci_ns(1,1));
    end
    % convert time vectors from unix time to time from start
    if start_s == 0
        [ms,state_time] = unixTime2TestTime(state_s(1,1),state_ns(1,1),state_s',state_ns');
        [mcs,command_state_time] = unixTime2TestTime(command_state_s(1,1),...
            command_state_ns(1,1),command_state_s',command_state_ns');
        [mss,status_time] = unixTime2TestTime(status_s(1,1),status_ns(1,1),...
            status_s',status_ns');
        [mx,sp_time] = unixTime2TestTime(sp_s(1,1),sp_ns(1,1),sp_s',sp_ns');
        [mxt,task_time] = unixTime2TestTime(task_s(1,1),task_ns(1,1),task_s',task_ns');
        [mv,vicon_time] = unixTime2TestTime(vicon_s(1,1),vicon_ns(1,1),vicon_s',vicon_ns');
        [mq,qci_time] = unixTime2TestTime(qci_s(1,1),qci_ns(1,1),qci_s',qci_ns');
    else
        [ms,state_time] = unixTime2TestTime(start_s,start_ns,state_s',state_ns');
        [mcs,command_state_time] = unixTime2TestTime(start_s,start_ns,...
            command_state_s',command_state_ns');
        [mss,status_time] = unixTime2TestTime(start_s,start_ns,status_s',status_ns');
        [mx,sp_time] = unixTime2TestTime(start_s,start_ns,sp_s',sp_ns');
        [mxt,task_time] = unixTime2TestTime(start_s,start_ns,task_s',task_ns');
        [mv,vicon_time] = unixTime2TestTime(start_s,start_ns,vicon_s',vicon_ns');
        [mq,qci_time] = unixTime2TestTime(start_s,start_ns,qci_s',qci_ns');
    end
    % crop bag
    cut_time = state_time(ms);
    % state time & find end of test time for bags that continued
    if ms > 70
        k = 50;
    else
        k = floor(ms*0.75);
    end
    for i=k:1:ms % crop state
        if state(i) == 0 && state(i-1) == 0 && state(i-2) == 0
            cut_time = state_time(i);
            state = state(1:ms);
            state_time = state_time(1:ms);
            ms = i;            
            break;
        end
    end
    if ms == 1
        state = 0;
    end
    % command state
    flight_begin_cs = 0; flight_end_cs = mcs;
    %if strcmp(vehicle,'hulk_smash') == 0 || (strcmp(vehicle,'hulk_smash') == 1 && test > 3)
        if mcs > 1
            for j = mcs:-1:1 % crop command state
                if abs(command_state_time(j) - cut_time) < 1.0  
                    command_state = command_state(1:j);
                    command_state_time = command_state_time(1:j);
                    mcs = j;
                    break;
                end
            end
            for j = mcs:-1:1 % determine end of flight
                if command_state(j) == 7
                    flight_end_cs = command_state_time(j);
                    break;
                end
            end
            for j = 1:1:mcs % determine start of flight
                if command_state(j) == 8
                    flight_begin_cs = command_state_time(j);
                    break;
                end
            end
        end
    %end
    if mcs == 1
        command_state = 0;
    end

    % status
    for j = mss-1:-1:1 % crop status
        if status_time(j) < cut_time
            mss = j + 1;
            break;
        end
    end
    if mss > 1
        voltage = voltage(1:mss);
        motors_status = motors_status(1:mss);
        serial_mode = serial_mode(1:mss);
        waypose_status = waypose_status(1:mss);
    else
        voltage = 0;
        motors_status = 0;
        serial_mode = 0;
        waypose_status = 0;
    end
    status_time = status_time(1:mss);
    
    % subject pose
    for j = mx-1:-1:1 % crop subject pose 
        if sp_time(j) < cut_time
            mx = j + 1;
            break;
        end
    end
    sp_time = sp_time(1:mx);
    if mx == 1
        x = 0;
        y = 0;
        z = 0;
        rotation_sp = 0;
    else
        x = x(1:mx);
        y = y(1:mx);
        z = z(1:mx);
        rotation_sp = rotation_sp(1:mx);
    end
    
    % task
    if mxt == 1
        x_t = 0;
        y_t = 0;
        z_t = 0;
        r_t = 0;
    end

    % vicon
    for j = mv-1:-1:1 % crop vicon
        if vicon_time(j) < cut_time
            mv = j + 1;
            break;
        end
    end
    vicon_time = vicon_time(1:mv);
    if mv == 1
        r_x = 0;
        r_y = 0;
        r_z = 0;
        r_w = 0;
        x_v = 0;
        y_v = 0;
        z_v = 0;
    else
        r_x = r_x(1:mv);
        r_y = r_y(1:mv);
        r_z = r_z(1:mv);
        r_w = r_w(1:mv);
        x_v = x_v(1:mv);
        y_v = y_v(1:mv);
        z_v = z_v(1:mv);
    end

    % power
    if v2 == 1
        if size(volt1_s) == [0,1]
            volt1_s = 0;
            volt1_ns = 0;
        end
        [mv1,sensor1_time] = unixTime2TestTime(start_s,start_ns,volt1_s',volt1_ns');
        [mv2,sensor2_time] = unixTime2TestTime(start_s,start_ns,volt2_s',volt2_ns');
    elseif v2 == 2
        if size(power_s) == [0,1]
            power_s = 0;
            power_ns = 0;
        end
        [mv1,sensor1_time] = unixTime2TestTime(start_s,start_ns,power_s',power_ns');
        mv2 = mv1;
        sensor2_time = sensor1_time;
    else
        mv1 = length(voltage1');
        mv2 = length(voltage2');
        if mv1 == 0
            mv1 = 1;
            mv2 = mv1;
        end
        if mv1 < 590
            v_freq = 0.2;
        elseif mv1 < 1190 && mv1 > 590
            v_freq = 0.1;
        else
            v_freq = 0.05;
        end
        sensor1_time = 0.0:v_freq:mv1*v_freq;
        sensor2_time = 0.0:v_freq:mv2*v_freq;
    end
    for j = mv1-1:-1:1 % crop sensor values 
        if sensor1_time(j) < cut_time
            mv1 = j + 1;
            break;
        end
    end
    mv1 = min([mv1,mv2]) ;
    mv2 = mv1;
    if mv1 == 1
        voltage1 = 0;
        voltage2 = 0;
        current1 = 0;
        current2 = 0;
    else
        voltage1 = voltage1(1:mv1);
        voltage2 = voltage2(1:mv2);
    end
    if v2 == 0
        sensor1_time = 0.0:v_freq:mv1*v_freq;
        sensor2_time = 0.0:v_freq:mv2*v_freq;
    else
        sensor1_time = sensor1_time(1:mv1);
        sensor2_time = sensor2_time(1:mv2);
    end
%         if max(voltage1) > 13 || max(voltage2) > 13
%             fprintf('high voltage on test: %i\n',test)
%         end
%         if min(voltage1) < 9 || min(voltage2) < 9
%             fprintf('low voltage on test: %i\n',test)
%         end
%         if max(current1) > 30 || max(current2) > 30
%             fprintf('high current on test: %i\n',test)
%         end
%         if min(current1) < 9 || min(current2) < 9
%             fprintf('low voltage on test: %i\n',test)
%         end

    % remove time travel            
    % Quad Control Input Time
    ek = 1;bk=mq;
    if mq > 1 %&& min(qci_time) <= 0.0
        for i = 1:1:mq % determine start of pitch and roll test
            if qci_time(i) > 0.0
                c_time = qci_time(i);
                k = i; 
                break;
            end
        end
        for i = mq:-1:1 % find end of pid input control
            if qci_time(i) > 30.0
                ek = i; 
                break;
            end
        end
        for i = ek:-1:1 % find start of pid input control
            if qci_time(i) < 0.0
                bk = i+1; 
                break;
            end
        end
        % approximate control rate used
        ctrl_qci = qci_time(k+1:ek) - qci_time(k:ek-1);
        ctrl_rate = median(ctrl_qci);

        % change qci time prior to dual publishers and during dual
        % publishing
        for i = k:-1:1 % fix front end negative values
            qci_time(i) = c_time - (k-i)*ctrl_rate;
        end
        for i = k+1:1:bk % fix dual publishing values
            if qci_time(i) <= 0.0
                if i < mq && qci_time(i-1) > 0.0 && qci_time(i+1) > 0.0
                    qci_time(i) = qci_time(i-1) + 0.0007;
                else
                    qci_time(i) = qci_time(i-1)+ctrl_rate;
                end
            end
        end
        for i = 2:1:mq % fix all other static times
            if qci_time(i) - qci_time(i-1) <= 0.0
                qci_time(i) = qci_time(i-1)+ctrl_rate;
            end
        end
        %[k min(qci_time)]  
        for j = mq-1:-1:1 % crop qci
            if qci_time(j) < cut_time
                mq = j + 1;
                break;
            end
        end
        qci_time = qci_time(1:mq);
        if qci_time(1,1) < 0 % is this still a valid case
            qci_time = qci_time - qci_time(1,1);
        end
    end
    if mq == 1
        pitch = 0;
        roll = 0;
        thrust = 0;
        rotation_qci = 0;
        flying = 0;
        control_qci = 0;
    else
        pitch = pitch(1:mq);
        roll = roll(1:mq);
        thrust = thrust(1:mq);
        rotation_qci = rotation_qci(1:mq);
        flying = flying(1:mq);
        control_qci = control_qci(1:mq);
    end
%         [qci_time(flight_begin_qci) qci_time(flight_begin_pose) qci_time(flight_end_qci) qci_time(flight_end_pose)] 
%         [vicon_time(1) vicon_time(ma) sp_time(1) sp_time(mx) state_time(1) state_time(ms) command_state_time(1) command_state_time(mcs)]
%         [status_time(1) status_time(mss) sensor1_time(1) sensor1_time(mv1) task_time(1) task_time(mxt)]
    
    %% Failure cases
    fail_flag = zeros(5,4);
    
    % if communication blocked
    if no_data == 1
        failed = failed + fail;
        if view_plot == 1
            fprintf('communication from controlling station was blocked')
            fprintf(' by external device\n')
            fprintf('test: %i failed\n',test)
        end
    else % if communication established proceed
        % object has flipped
        gap = 0.6;
        d = 200;
        for i = 1:1:mq % determine if vehicle flipped
             if (abs(pitch(i,1)) > 0.3 && abs(pitch(i,1)) < 1.0) || (abs(roll(i,1)) > 0.3 && abs(roll(i,1)) < 1.0)
                if (max(r_x) > gap && min(r_x) < -gap) || (max(r_y) > gap && min(r_y) < -gap)
                    for i = 1:1:mv - d
                        if r_x(i) > gap
                            for j = 1:1:d
                                if r_x(i+j) < gap
                                    break;
                                elseif j == d           
                                    fail_flag(4,1) = 1;
                                    fail = 1;
                                end
                            end
                        elseif r_y(i) > gap
                            for j = 1:1:d
                                if r_y(i+j) < gap
                                    break;
                                elseif j == d           
                                    fail_flag(4,1) = 1;
                                    fail = 1;
                                end
                            end
                        end
                        if fail_flag(4,1) == 1
                            break;
                        end
                    end
                end
             end
         end

        % momentary and sustained extreme rotation around x or y axis
        % rotation failure indicator for misinterpretation of object
        gap = 0.5;
        if (max(r_x) > gap || max(r_y) > gap || min(r_x) < -gap || min(r_y) < -gap)
            pass_count = zeros(4,1);
            fail_flag(1,4) = 1;
            for i = 1500:1:mv - 1500 % almost flipping
                if r_x(i,1) > gap
                    pass_count(1,1) = pass_count(1,1) + 1;
                elseif r_x(i,1) < -gap
                    pass_count(2,1) = pass_count(2,1) + 1;
                elseif r_y(i,1) > gap
                    pass_count(3,1) = pass_count(3,1) + 1;
                elseif r_y(i,1) < -gap
                    pass_count(4,1) = pass_count(4,1) + 1;
                else
                    pass_count = zeros(4,1);
                end
                if max(pass_count) == 50;
                    fail = 1;
                    break;
                end
            end
        end
        %--
        
        %--TASK Propagation
        % fail indicator for incorrect task propagation
        begin_task = 1;
        end_task = mxt;
        % Missing and extra tasks
        for j = 1:1:mxt
            if r_t(j) > 4.0
                if strcmp(vehicle,'morpheus') == 1 && j > 4
                    begin_task = j-4;
                    break;
                end
                for i = j-1:-1:1 % find pre-launch task
                    if r_t(i) == 0
                        if i > 2 && r_t(i-1) > 3 && r_t(i-2) == 0 && r_t(i+1) < 3
                            begin_task = i-2;% single zero task inserted
                        else
                            begin_task = i;
                        end
                        break;
                    end
                end
                break;
            end
        end
        for k = mxt:-1:6 % find land task
            if r_t(k-4) ~= 0.0 && r_t(k) == 0.0 && r_t(k-5) ~= 0.0              
                end_task = k;
                break;
            end
        end
        tasks = end_task - begin_task + 1;
        
        % failure indicator for extra and missing tasks
        if tasks > 36
            fail_flag(2,1) = 1;
            fail = 1;
        elseif tasks < 36
            fail_flag(2,2) = 1;
            fail = 1;
        end
        
        % failure indicator for an extra and a missing task on one run
        xyzr_change = zeros(4,2);
        if strcmp(vehicle,'morpheus') == 0
            xyzr_task = [4 4; 6 4; 6 5; 6 4];
        else
            xyzr_task = [4 4; 6 4; 32 4; 4 4];
        end
        for j = begin_task:1:end_task
            % check for missing x change
            if x_t(j) > x_t(begin_task)
                xyzr_change(1,1) = xyzr_change(1,1) + 1;
            elseif x_t(j) < x_t(begin_task)
                xyzr_change(1,2) = xyzr_change(1,2) + 1;
            end
            % check for missing y change
            if y_t(j) > y_t(begin_task)
                xyzr_change(2,1) = xyzr_change(2,1) + 1;
            elseif y_t(j) < y_t(begin_task)
                xyzr_change(2,2) = xyzr_change(2,2) + 1;
            end
            % check for missing z change
            if z_t(j) > z_t(begin_task)
                xyzr_change(3,1) = xyzr_change(3,1) + 1;
            end
            % check for missing r change
            if r_t(j) < 2 && r_t(j) > 1
                xyzr_change(4,1) = xyzr_change(4,1) + 1;
            elseif r_t(j) < 5 && r_t(j) > 4
                xyzr_change(4,2) = xyzr_change(4,2) + 1;
            elseif r_t(j) < 4 && r_t(j) > 3
                xyzr_change(3,2) = xyzr_change(3,2) + 1;
            end
        end
        
        %[xyzr_change - xyzr_task xyzr_change xyzr_task]
        if sum(sum(abs(xyzr_change - xyzr_task))) ~= 0  && tasks == 36
            fail = 1;
            for i = 1:1:4
                for j = 1:1:2
                    if xyzr_change(i,j) - xyzr_task(i,j) > 0
                        fail_flag(2,1) = 1;
                    elseif xyzr_change(i,j) - xyzr_task(i,j) < 0
                        fail_flag(2,2) = 1;
                    end
                end
            end
        end        
        %--
     
        
        %-- SIGNAL DELAY
        loss = zeros(1,2);max_delay=zeros(2,7);
        %[qci_time(flight_begin_qci) qci_time(flight_end_qci) qci_time(flight_begin_pose) qci_time(flight_end_pose) task_time(begin_task) task_time(end_task)]
        
        % Vicon packet delay
        if strcmp(vehicle,'morpheus') == 1
            [loss,packet_delay(:,6),flags,max_delay(:,6)] = delayed_packet(vicon_time,...
                flight_begin_cs-1.000,flight_end_cs+3.000,mv,0.05,0,0);
        else
            [loss,packet_delay(:,6),flags,max_delay(:,6)] = delayed_packet(vicon_time,...
                flight_begin_cs-1.000,flight_end_cs+3.000,mv,0.05,0,0);
        end
        fail_flag(1,3) = flags;
      
        % state packet delay
        [loss,packet_delay(:,4),flags,max_delay(:,4)] = delayed_packet(state_time,...
            flight_begin_cs-1.000,flight_end_cs+3.000,ms,0.2,1,0);
        %fail_flag(3,4) = flags;
        
        % command state packet delay
        if strcmp(vehicle,'morpheus') == 1
            [loss,packet_delay(:,3),flags,max_delay(:,3)] = delayed_packet(command_state_time,...
                flight_begin_cs-1.000,flight_end_cs+3.000,mcs,2.3,0,1);
        elseif strcmp(vehicle,'hulk_smash') == 0 || (strcmp(vehicle,'hulk_smash') == 1 && test < 4)
            [loss,packet_delay(:,3),flags,max_delay(:,3)] = delayed_packet(command_state_time,...
                flight_begin_cs-1.000,flight_end_cs+3.000,mcs,1.3,0,1);
        end
        fail_flag(3,4) = flags;
        
        % task packet delay
        if strcmp(vehicle,'morpheus') == 1
            [loss_task,packet_delay(:,2),flags,max_delay(:,2)] = delayed_packet(task_time,...
                flight_begin_cs-1.000,flight_end_cs+3.000,mxt,2.3,0,1);
        else
            [loss_task,packet_delay(:,2),flags,max_delay(:,2)] = delayed_packet(task_time,...
                flight_begin_cs-1.000,flight_end_cs+3.000,mxt,1.3,0,1);
        end
        fail_flag(4,4) = flags;
        
        % subject status packet delay
        if strcmp(vehicle,'morpheus') == 1
            [loss,packet_delay(:,5),flags,max_delay(:,5)] = delayed_packet(status_time,...
                flight_begin_cs-1.000,flight_end_cs+3.000,mss,2.3,0,0);%0.15
        else
            [loss,packet_delay(:,5),flags,max_delay(:,5)] = delayed_packet(status_time,...
                flight_begin_cs-1.000,flight_end_cs+3.000,mss,1.3,0,0);%0.15
        end
        fail_flag(3,3) = flags;
                
        if strcmp(vehicle,'morpheus') == 0
            % subject pose packet delay
            [loss,packet_delay(:,7),flags,max_delay(:,7)] = delayed_packet(sp_time,...
                flight_begin_cs-1.000,flight_end_cs+3.000,mx,...
                ctrl_rate*10,0,0);
        else
            [loss,packet_delay(:,7),flags,max_delay(:,7)] = delayed_packet(sp_time,...
                flight_begin_cs-1.000,flight_end_cs+3.000,mx,...
                ctrl_rate*10,0,0);
        end
        fail_flag(4,3) = flags;
        
        if (strcmp(vehicle,'morpheus') == 0 && strcmp(vehicle,'hulk_smash') == 0) || (strcmp(vehicle,'hulk_smash') == 1 && test < 4)
            % quad control input packet delay
            [loss,packet_delay(:,1),flags,max_delay(:,1)] = delayed_packet(qci_time,...
                flight_begin_cs-1.000,flight_end_cs+3.000,mq,...
                ctrl_rate*12.25,0,0);
            %flight_end_cs+1.000
        elseif strcmp(vehicle,'morpheus') == 0
            [loss,packet_delay(:,1),flags,max_delay(:,1)] = delayed_packet(qci_time,...
                flight_begin_cs-1.000,flight_end_cs+3.000,mq,...
                ctrl_rate*12.25,0,0);
            %flight_end_cs
        else
            flags = 0;
        end
        fail_flag(4,2) = flags;
        % packet delay failure based on task and quad control input
        if fail_flag(4,2) == 1 || fail_flag(4,4) == 1 || fail_flag(3,4) == 1
            control_packet(1,1) = control_packet(1,1) + 1;
            fail = 1;
        end
        if view_plot == 1 && sum(sum(packet_delay)) > 0.0
            fprintf('test: %i qci \t task \t ctrl state \t state \t status \t vicon \t pose\n',test); 
            max_delay
            packet_delay
        end
        %--
 
        %--SUBJECT STATUS
        % fail indicator for lost of serial communication, manual
        % control taken and motors fail to turn off
        state_end = ms; state_begin = 1;
        for j = 2:1:ms
            if state(j) > 5
                state_begin = j;
                break;
            end
        end
        for j = ms:-1:state_begin
            if state(j) == 8
                if ms > j+3
                    state_end = j+3;
                    break;
                end
            end
        end
        if strcmp(vehicle,'morpheus') == 0
            serial_flag = 0;
            motors_flag = 0;
            begin_flight = state_time(state_begin);
            end_flight = state_time(state_end);
            %[test begin_flight end_flight end_flight - begin_flight]
            
            if sum(serial_mode) < 37
                mc_time = 0.0;
                fail = 1;
                fail_flag(1,1) = 1; % manual control taken
            end

            for i = 1:1:mss
                % Loss of serial communication
                if serial_mode(i,1) == 1 && serial_flag == 0
                    serial_flag = 1; % serial comms established
                end
                if serial_flag == 1 && serial_mode(i,1) == 0
                    serial_flag = 2; % serial comms lost
                    mc_time = status_time(i);
                    for j = i:1:mss
                        if serial_mode(j,1) == 1
                            if status_time(i) >= begin_flight && status_time(i) <= end_flight
                                fail = 1;
                            end                        
                            fail_flag(1,2) = 1; % Loss of serial communication
                            break;
                        end 
                    end
                    % manual control taken
                    if fail_flag(1,2) == 0 && motors_status(i,1) == 1
                        fail = 1;
                        fail_flag(1,1) = 1; % manual control taken
                    end
                end
         
                % motors turned off momentarily
                if motors_status(i,1) == 1 && motors_flag == 0
                    motors_flag = 1; % motors on
                end
                if motors_flag == 1 && motors_status(i,1) == 0
                    motors_flag = 2;
                    for j = i:1:mss
                        if motors_status(j,1) == 1
                            fail = 1;
                            fail_flag(3,1) = 1; % motors turned off momentarily
                            break;
                        end 
                    end
                end
            end
        end
        % motors did not turn off
        if motors_status(mss,1) ~= 0 && state(ms,1) == 0
%             fail = 1;
            fail_flag(3,1) = 1; 
        end
        
        % fail indicator for loss of GPS control
        for i = 2:1:mq-5
            if flying(i-1,1) == 1 && flying(i+1,1) == 1 && flying(i,1) == 0
                fail = 1;
                fail_flag(3,2) = 1;
                break;
            end
        end
                  
        % subject status waypose status propagation failure
        for i=2:1:mss-5
            if waypose_status(i,1) == 0 && waypose_status(i+1,1) == 0 
                if waypose_status(i+2,1) == 1 && waypose_status(i+3,1) == 1 && waypose_status(i+4,1) == 0
                    fail_flag(3,3) = 1;
                end
            elseif waypose_status(i,1) == 1 && waypose_status(i+1,1) == 1 
                if waypose_status(i+2,1) == 0 && waypose_status(i+3,1) == 0 && waypose_status(i+4,1) == 1
                    fail_flag(3,3) = 1;
                end
            end        
        end        
        %--
        
        %--STATE Propagation
        % fail indicator for incorrect state propagation
        if state_end - state_begin > 36 % extra state
            fail = 1;
            fail_flag(2,3) = 1;
        elseif state_end - state_begin < 36 % missing state
            fail = 1;
            fail_flag(2,4) = 1;
        end
        
        for j = state_begin:1:state_end
            if state(j,1) == 8 && state_end < ms % Two move states back to back
                if state(j+1,1) == 8
                    fail = 1;
                    fail_flag(5,1) = 1;
                end
            elseif state(j,1) == 7 && state_end < ms - 2 % Two hover states back to back
                if (fail_flag(1,1) == 1 && state_time(j) < mc_time) || fail_flag(1,1) == 0
                    if state(j+1,1) == 7 
                        if state(j+2,1) == 8 || (state(j+2,1) == 7 && state(j+3,1) == 7 )
                            fail = 1;
                            fail_flag(5,2) = 1;
                        end
                    end
                end
            end
        end
        %--
        
%         excess = 0;
%         for i = 3:1:ms
%             if state(i) == 2
%                 excess = excess + 1;
%             elseif state(i) >= 4
%                 break;
%             end
%         end
%         if excess > 5
%             [test excess]
%         end
        if sum(fail_flag(2,:)) > 0
            control_packet(1,2) = control_packet(1,2) + 1;
        end
        control_packet(1,3) = control_packet(1,3) + fail_flag(2,1);
        control_packet(1,4) = control_packet(1,4) + fail_flag(2,2);
        control_packet(1,5) = control_packet(1,5) + fail_flag(2,3);
        control_packet(1,6) = control_packet(1,6) + fail_flag(2,4);

        % update fail type count
        fail_type = fail_type + fail_flag;
        % update total failed
        failed = failed + fail;
        % print fail types on test
        if sum(sum(fail_flag)) > 0
            if view_plot == 1
                if fail_flag(1,1) == 1
                    fprintf('manual control taken on test: %i at time: %f\n',test, mc_time) 
                end
                if fail_flag(1,2) == 1
                    fprintf(' loss of serial communication on test: %i\n',test)
                end
                if fail_flag(1,3) == 1
                    fprintf('  loss of vicon on test: %i\n',test)
                end
                if fail_flag(1,4) == 1
                    fprintf('    Vicon lost object momentarily on test: %i\n',test)
                end
                if fail_flag(2,1) == 1
                    fprintf('   %i Extra task(s) added on test: %i\n',tasks-36,test)
                end
                if fail_flag(2,2) == 1
                    fprintf('   %i missing task on test: %i\n',36-tasks,test)
                end
                if fail_flag(2,3) == 1
                    fprintf('   %i Extra state(s) on test: %i\n',state_end-state_begin-36,test)
                end
                if fail_flag(2,4) == 1
                    fprintf('   %i missing state(s) on test: %i\n',36-(state_end-state_begin),test)
                end
                if fail_flag(3,1) == 1
                    fprintf('  motor response incorrect while on ground on test: %i\n',test)
                end
                if fail_flag(3,2) == 1 % 3,2
                    fprintf('Competing PID controllers on test: %i\n',test)
                end
                if fail_flag(3,3) == 1 % 3,3
                    fprintf('  Subject status task propagation inaccurate on test: %i\n',test)
                end
                if fail_flag(3,4) == 1 % 3,4
                    fprintf('     command state messages abnormal delay on test: %i\n',test)
                end
                if fail_flag(4,1) == 1
                    fprintf('vehicle may have flipped on test: %i\n',test)
                end
                if fail_flag(4,2) == 1 % 3,2
                    fprintf('  quad control input messages abnormal delay on test: %i\n',test)
                end
                if fail_flag(4,3) == 1 % 3,3
                    fprintf('     Subject pose messages dropped on test: %i\n',test)
                end
                if fail_flag(4,4) == 1 % 3,3
                    fprintf('     task waypose messages abnormal delay on test: %i\n',test)
                end
                if fail_flag(5,2) == 1
                    fprintf('Two hover states back to back on test: %i\n',test)
                end
                if fail_flag(5,1) == 1
                    fprintf('Two Move states back to back on test: %i\n',test)
                end
                if fail == 1
                    fprintf('test: %i failed\n',test)
                    %[flight_begin_cs flight_end_cs]
                end
            end
        end
    end
    if fail == 1
        if failed == 1
            mtbf(1,failed) = test;
        else
            mtbf(2,failed-1) = test;
            mtbf(1,failed) = test;
            mtbf(3,failed-1) = mtbf(2,failed-1) - mtbf(1,failed-1);
        end
    end
    %% time calculation
    ot(test_index,2) = fail;
    if no_data == 0%
        ot(test_index,1) = max([vicon_time(mv) sp_time(mx) state_time(ms) status_time(mss) task_time(mxt)]);
        ot(test_index,3) = flight_end_cs - flight_begin_cs + 2.0;
        %         if ot(test_index,1) >= 70 && view_plot == 1
%             [test ot(test_index,1)]
%         end
    end
    
%     % calculate time spent in each state
%     if test_index == 1
%         ti = 0;
%     end
% 
%     if fail == 0
%         current_state_index = 1;
%         transition = 0;
%         ti = ti +1;
%         for i = 2:1:ms
%             if state(i,1) ~= state(i-1,1) 
%                 transition = transition + 1;
%                 trans_time(ti,transition,1) = state(i-1);
%                 trans_time(ti,transition,2) = state_time(i) - state_time(current_state_index);
%                 trans_time(ti,transition,3) = state(i);
%                 current_state_index = i;
%             end
%         end
%     end
        
    %% Power Calculation Totals
    oap(test_index,3) = fail;
    pmax(test_index,3) = fail;

%     if fail == 1
%         fprintf('Voltage extrema')
%         [max(voltage1) max(voltage2);min(voltage1) min(voltage2)]
%         fprintf('Current extrema')
%         [max(current1) max(current2);min(current1) min(current2)]
%     end
    if no_data == 0
        % power sensor 1
        [voltage1, current1, mv1] = equalizePower(voltage1,current1,v2);
        mc1 = mv1;

        % power sensor 2
        [voltage2, current2, mv2] = equalizePower(voltage2,current2,v2);
        mc2 = mv2;
        m = min(mv1,mv2);
     
        % separate hover and move task operating average power
    %if fail == 0
        if strcmp(vehicle,'morpheus') == 0 
            move_time = 0.9;
        else
            move_time = 1.9;
        end
        begin_sensor_time = 1; end_sensor_time = m;
        for i = 1:1:m
            if sensor1_time(i) >= flight_begin_cs
                begin_sensor_time = i;
                break;
            end
        end
        for i = m:-1:1
            if sensor1_time(i) <= flight_end_cs
                end_sensor_time = i;
                break;
            end
        end
        toggle_time = sensor1_time(begin_sensor_time);
        toggle_flag = 1;
        power_task = zeros(2,2);
        task_count = zeros(1,2);
        
        for i = begin_sensor_time:1:end_sensor_time           
            if sensor1_time(i) - sensor1_time(begin_sensor_time) < 36.0 && sensor1_time(i) - toggle_time > move_time
                if toggle_flag == 1
                    toggle_flag = 2;
                    move_time = 0.9;
                else
                    toggle_flag = 1;
                    if strcmp(vehicle,'morpheus') == 0 
                        move_time = 0.9;
                    else
                        move_time = 1.9;
                    end
                end
                toggle_time = sensor1_time(i);
            elseif sensor1_time(i) - toggle_time > 0.9
                toggle_flag = 2;
            end
            power_task(toggle_flag,1) = power_task(toggle_flag,1) + current1(i)*voltage1(i);
            power_task(toggle_flag,2) = power_task(toggle_flag,2) + current2(i)*voltage2(i);
            task_count(toggle_flag) = task_count(toggle_flag) + 2;            
        end
        task_power(test_index,1) = (power_task(1,1) + power_task(1,2)) / task_count(1);
        task_power(test_index,2) = (power_task(2,1) + power_task(2,2)) / task_count(2);
        
        L=begin_sensor_time;
        R=end_sensor_time;
        %[test L R]
        % estimate operating average power
        oav = sum(voltage1(L:R))/(R-L+1);
        oac = sum(current1(L:R))/(R-L+1);
        oap(test_index,1) = oav.*oac;
        oav = sum(voltage2(L:R))/(R-L+1);
        oac = sum(current2(L:R))/(R-L+1);
        oap(test_index,2) = oav.*oac;

        % find maximum power
        pmax(test_index,1) = max(voltage1.*current1);
        pmax(test_index,2) = max(voltage2.*current2);
        %--
    else
        oav = -1;
        oac = -1;
        oap(test_index,1) = -1;
        oap(test_index,2) = -1;
        pmax(test_index,1) = -1;
        pmax(test_index,2) = -1;
    end

    %% Position Calculation Totals
    precision(test_index,5) = fail;
    if test_index == 1
        max_x = 0;max_y = 0;max_r = 0;
    end
    if no_data == 0
        se_dif_r = rotation_sp(1) - rotation_sp(mx);
        if se_dif_r > pi()
            se_dif_r = se_dif_r - 2*pi();
        elseif se_dif_r < -pi()
            se_dif_r = se_dif_r + 2*pi();
        end
    %     if abs(x_v(1) - x_v(ma)) > max_x
    %         max_x = abs(x_v(1) - x_v(ma));
    %         %[test max_x max_y max_r]
    %     end
    %     if abs(y_v(1) - y_v(ma)) > max_y
    %         max_y = abs(y_v(1) - y_v(ma));
    %         %[test max_x max_y max_r]
    %     end
        if abs(se_dif_r) > max_r
            max_r = abs(se_dif_r);
            %[test max_x max_y max_r]
        end
        %[test x_v(1) - x_v(ma) y_v(1) - y_v(ma) se_dif_r;] 
       
        % reset off zero
        for k = 1:1:mxt
            if x_t(k,1) ~= 0.0
                x_t(1:k-1,1) = x_t(k,1);
                y_t(1:k-1,1) = y_t(k,1);
                z_t(1:k-1,1) = 0.0; %
                z_t(end_task + 2:mxt)=0.0;%
                break;
            end
        end
        if z_t(1,1) ~= 0.0 % reset failed, retry
            for j = 1:1:mv
                if z_v(j) > 0.1
                    launch_time = vicon_time(j);
                    break;
                end
            end
            for k = 1:1:mxt
                if task_time(k) > launch_time
                    z_t(1:k-1,1) = 0.0; %
                    break;
                end
            end
        end

        % time synchronization
        k = 1;
        pos_sync = zeros(mxt,2);
        for i = begin_task-1:1:end_task+1
            for j = k:1:mx
                if i > 1 && i < mxt && task_time(i) < sp_time(j)
                    pos_sync(i,1) = i;
                    pos_sync(i,2) = j;
                    k = j;
                    break;
                end    
            end
        end

        % generate task array same length as position array for each
        % dimension
        start = pos_sync(begin_task,2);
        stop = pos_sync(end_task,2);
        move_count = start;
        prev_count = 0;    
        
        %ma=mx;vicon_time=sp_time;x_v=x;y_v=y;z_v=z; r_v = rotation_sp;
        % initialize task arrays
        xyzr_f = zeros(mx,4);
        xyzr_f(1:mx,1) = x_t(1,1);
        xyzr_f(1:mx,2) = y_t(1,1);
        xyzr_f(1:mx,3) = z_t(1,1);
        if rotation_sp(1,1) > 6
            xyzr_f(1:mx,4) = 2*pi();
        else
            xyzr_f(1:mx,4) = 0.0;
        end
        % construct linear task arrays 
        for j = begin_task:1:end_task - 1
            d_x = x_t(j+1,1) - x_t(j,1);
            d_y = y_t(j+1,1) - y_t(j,1);
            d_z = z_t(j+1,1) - z_t(j,1);
            if j == begin_task && j > 1
                d_r = r_t(j+1,1) - r_t(j-1,1);
            else
                d_r = r_t(j+1,1) - r_t(j,1);
            end
            sync_diff = pos_sync(j+1,2) - pos_sync(j,2)-0;
            if sync_diff < 0
                [test begin_task end_task j sync_diff]
            end
            while move_count < prev_count + sync_diff + 0
                move_count = move_count + 1;
                xyzr_f(move_count,1) = x_t(j,1)...
                    + d_x*((move_count-prev_count)/sync_diff);
                xyzr_f(move_count,2) = y_t(j,1)...
                    + d_y*((move_count-prev_count)/sync_diff);
                xyzr_f(move_count,3) = z_t(j,1)...
                    + d_z*((move_count-prev_count)/sync_diff);
                if j ~= begin_task && j ~= begin_task + 1
                    xyzr_f(move_count,4) = r_t(j,1)...
                        + d_r*((move_count-prev_count)/sync_diff);
                end
            end
            prev_count = move_count;
        end

        % time syncronization offset
        offset = 901;

        % Subtract actual from ideal position
        fs = zeros(mx-offset+1,4);                 
        fs(:,1) = xyzr_f(1:mx-offset+1,1) - x(offset:mx,1);
        fs(:,2) = xyzr_f(1:mx-offset+1,2) - y(offset:mx,1);
        fs(:,3) = xyzr_f(1:mx-offset+1,3) - z(offset:mx,1);

        % subtract actual from ideal rotation
        r_task = xyzr_f(1:mx,4);
        r_actual = rotation_sp(offset:mx);
        i_r = min(length(r_task),length(r_actual));
        mf = length(fs');
        for i = 1:1:i_r
            if r_task(i,1) == 0 && r_actual(i,1) > 6
                fs(i,4) = 2*pi() - r_actual(i,1);
            elseif r_task(i,1) > 6 && r_actual(i,1) < 1
                fs(i,4) = r_actual(i,1) - 0.0;
            else
                fs(i,4) = r_task(i,1) - r_actual(i,1);
            end
            if fs(i,4) > pi()
                if r_task(i,1) > 6
                    fs(i,4) = r_actual(i,1) - 0.0;
                end
            end  
        end

        % calculate error measure, MAE
        precision(test_index,1) = sum(abs(fs(:,1)))/mf;%sqrt(sum(fs(:,1).*fs(:,1)))/mf;
        precision(test_index,2) = sum(abs(fs(:,2)))/mf;%sqrt(sum(fs(:,2).*fs(:,2)))/mf;
        precision(test_index,3) = sum(abs(fs(:,3)))/mf;%sqrt(sum(fs(:,3).*fs(:,3)))/mf;
        precision(test_index,4) = sum(abs(fs(:,4)))/mf;%sqrt(sum(fs(:,4).*fs(:,4)))/mf;          

        % plot time synchronization position graphs
        if view_plot == 1
            figure
            plot(fs(:,1),'k.')
            hold on
            plot(fs(:,2),'cx')
            plot(fs(:,3),'b+')
%             if loss_vicon(1) ~= 0
%                 for i=1:1:length(loss_vicon)
%                     %
%                     plot(loss_vicon(i),min(min(fs)):max(max(fs)))
%                 end
%             end

            figure
            plot(fs(:,4),'k*')

            % rotation actual versus tasked
            figure,%[length(r_task) length(r_actual) length(sp_time(offset:mx)) length(vicon_time)]
            plot_line(loss_task,r_task)
            hold on
            plot(sp_time,r_task,'bx')
            grid on
            grid minor
            plot(sp_time(1:mx-offset+1),r_actual,'c')
            title(sprintf('Rotation Error of %s%i',vehicle,test))
            hold off

            % print figure
            if save_plot == 1
                print(sprintf('../plot/%s/%s%i/rotation_error_%s%i',...
                    test_set,vehicle,test,vehicle,test),'-djpeg')
            end

            % spatial actual versus tasked
            figure
            subplot(3,1,1) % x
            plot_line(loss_task,x)
            hold on
            plot(sp_time,x,'c')
            grid on
            grid minor
            plot(sp_time(offset:mx),xyzr_f(1:mx-offset+1,1),'b')
            title(sprintf('Position Error of %s%i',vehicle,test))
            hold off

            subplot(3,1,2) % y
            plot_line(loss_task,xyzr_f(:,2))
            hold on
            plot(sp_time,y,'c')
            grid on
            grid minor
            plot(sp_time(offset:mx),xyzr_f(1:mx-offset+1,2),'b')
            hold off

            subplot(3,1,3) % z
            plot_line(loss_task,xyzr_f(:,3))
            hold on
            plot(sp_time,z,'c')
            grid on
            grid minor
            plot(sp_time(offset:mx),xyzr_f(1:mx-offset+1,3),'b')
            hold off

            % print figure
            if save_plot == 1
                print(sprintf('../plot/%s/%s%i/position_error_%s%i',...
                    test_set,vehicle,test,vehicle,test),'-djpeg')
            end
        end
    end

    %% control plots
    if view_plot == 1 
        if no_data == 1
            loss_task = 0;
        end
        figure
%         plot_line(loss_task,control_qci)
        hold on
        if size(qci_time) == [0,1]
            plot(control_qci,'b.')
        else
            plot(qci_time,control_qci,'b.')
        end
        grid on
        grid minor
        title(sprintf('Quad Control Input Control Sequence of %s%i',vehicle,test))
        xlabel('Time (seconds)')
        ylabel('Sequence Number')
        % print figure
        if save_plot == 1
            print(sprintf('../plot/%s/%s%i/control_%s%i',...
                test_set,vehicle,test,vehicle,test),'-djpeg')
        end
    end

    %% voltage, current, power 

    % voltage
    if view_plot == 1
        % empty plot
        if no_data == 1
            loss_task = 0;
            L = 0;
            R = 0;
        end
        
        figure
        subplot(2,2,2), 
%         if v2 == 1
%             plot_line(loss_task,voltage1)
            plot(sensor1_time(1:mv1),voltage1,'k+') 
%         else
%             plot(voltage1,'k+')
%         end
        hold on
%         if v2 == 1
            plot(sensor2_time(1:mv2),voltage2,'cx')
            xlabel('Time (seconds)')
%         else
%             plot(voltage2,'cx')
%             xlabel('Discrete Time Sample (n)')
%         end     
        grid on
        grid minor
        title('Sensor Voltage')            
        ylabel('Voltage (volts)')
        %legend('Sensor 1','Sensor 2')
        hold off

        % current
        subplot(2,2,3), 
%         if v2 == 1
%             plot_line(loss_task,current1)
            plot(sensor1_time(1:mv1),current1,'k+')
%         else
%             plot(current1,'k+')
%         end
        hold on
%         if v2 == 1
             plot(sensor2_time(1:mv2),current2,'cx')
            xlabel('Time (seconds)')
%         else
%             plot(current2,'cx')
%             xlabel('Discrete Time Sample (n)')
%         end

        grid on
        grid minor
        title('Sensor Current')
        ylabel('Current (Amperes)')
        % legend('Sensor 1','Sensor 2')
        hold off

        %[L, LP1,R]
        % plot power figure
        subplot(2,2,1), 
%         if v2 == 1
            plot(sensor1_time(1:mv1),current1.*voltage1,'k+')
%         else
%             plot(current1.*voltage1,'k+')
%         end
        hold on
%         if v2 == 1
            plot(sensor2_time(1:mv2),current2.*voltage2,'cx')
            xlabel('Time (seconds)')
%         else
%             plot(current2.*voltage2,'cx')
%             xlabel('Discrete Time Sample (n)')
%         end
        if fail == 0
            plot(sensor1_time(L),100,'r^')
            %plot(LP1,100,'b^')
            plot(sensor1_time(R),100,'y^')
        end
        grid on
        grid minor
        title(sprintf('Apparent Power of %s%i',vehicle,test))
        ylabel('Power (Watts)')
        %legend('Sensor 1','Sensor 2')
        hold off

        % "battery" voltage
        subplot(2,2,4), 
%         plot_line(loss_task,voltage)
        hold on
        plot(status_time,voltage,'b-')
        grid on
        grid minor
        title('Voltage (via Status Node)')
        xlabel('Time (seconds)')
        ylabel('Voltage (volts)')
        %legend('Status Voltage')

        % print figure
        if save_plot == 1
            print(sprintf('../plot/%s/%s%i/power_%s%i',...
                test_set,vehicle,test,vehicle,test),'-djpeg')
        end
    end

    %% position Plot

    % 3-D position
    if view_plot == 1
        % empty plot
        if no_data == 1
            loss_task = 0;
        end
        
        figure
        plot3(x,y,z,'c.') % subject pose
        %plot3(x_v,y_v,z_v,'c.') % Vicon
        hold on 
        plot3(x_t,y_t,z_t,'b-') % task waypose
        grid on
        grid minor
        title(sprintf('Path of %s%i',vehicle,test))
        xlabel('Position in X Direction (meters)')
        ylabel('Position in Y Direction (meters)')
        zlabel('Position in Z Direction (meters)')
        %legend('Actual Path', 'Ideal Path')

        % print
        if save_plot == 1
            print(sprintf('../plot/%s/%s%i/task_path_%s%i',...
                test_set,vehicle,test,vehicle,test),'-djpeg')
        end

        % 1-D position with respect to time
        figure
        m = size(x);
        subplot(2,2,2), 
%         plot_line(loss_task,z)
        hold on
        plot(sp_time,z,'b.')
        plot(sp_time,y,'c.')
        plot(sp_time,x,'k.')
        grid on
        grid minor
        title('Subject Pose')
        xlabel('Time (seconds)')
        ylabel('Position in X, Y, and Z Direction (meters)')
%         axis([0 sp_time(mx) + 0.001 min(min(min(z,y),x)) - .1 max(max(max(z,y),x)) + .1])
        %legend('X Position','Y Position','Z Position')
        hold off

        % pitch roll and thrust
        subplot(2,2,4), 
%         plot_line(loss_task,thrust) 
        hold on
        plot(qci_time,thrust,'b.')
        plot(qci_time,pitch,'c.')
        plot(qci_time,roll,'k.')
        %axis([0 max(qci_time) + 0.001 -1.1 1.1])
        grid on
        grid minor
        title('Quad Control Input Pitch, Roll, and Thrust')
        xlabel('Time (seconds)')
        ylabel('Controller Control Rates (unit normalized)')
        % legend('Thrust','Pitch','Roll')
        hold off

        % vicon position
        subplot(2,2,3), %plot_line(loss_task,z_v)
        hold on
        plot(vicon_time,y_v,'c.')
        plot(vicon_time,x_v,'k.')
        plot(vicon_time,z_v,'b.')
%         axis([0 vicon_time(mv) + 0.001 min(min(min(z_v,y_v),x_v)) - .1 max(max(max(z_v,y_v),x_v)) + .1])
        grid on
        grid minor
        title('Vicon Position')
        xlabel('Time (seconds)')
        ylabel('Position in X, Y, and Z Direction (meters)')
        %legend('Position in X','Position in Y','Position in Z')
        hold off

        % Task 1-D position with respect to time
        subplot(2,2,1), %plot_line(loss_task,z_t)
        hold on
        plot(task_time,y_t,'cx')
        plot(task_time,x_t,'k.')   
        plot(task_time,z_t,'b+')
%         axis([0 task_time(mxt) + 0.001 min(min(min(z_t,y_t),x_t)) - .1 max(max(max(z_t,y_t),x_t)) + .1])
        grid on
        grid minor
        title(sprintf('Task Waypose of %s%i',vehicle,test))
        xlabel('Time (seconds)')
        ylabel('Position in X, Y, and Z Direction (meters)')
        %legend('X Position','Y Position','Z Position')
        hold off

        % print figure
        if save_plot == 1
            print(sprintf('../plot/%s/%s%i/position_%s%i',...
                test_set,vehicle,test,vehicle,test),'-djpeg')
        end
    end

    %% rotation

    % rotation (subject_pose)
    if view_plot == 1
        % empty plot
        if no_data == 1
            loss_task = 0;
        end
        
        figure
        subplot(2,2,2), %plot_line(loss_task,rotation_sp)
        hold on
        plot(sp_time,rotation_sp,'k.')
%         axis([0 sp_time(mx) + 0.001 min(rotation_sp) - .1 max(rotation_sp) + .1])
        grid on
        grid minor
        title('Subject Pose Rotation')
        xlabel('Time (seconds)')
        ylabel('Rotation Around Z Axis (radians)')
        %legend('Rotation')

        % rotation control input
        subplot(2,2,4), %plot_line(loss_task,rotation_qci)
        hold on
        plot(qci_time,rotation_qci,'cx')            
        grid on
        grid minor
        %axis([0 max(qci_time) + 0.001 min(rotation_qci) - .1 max(rotation_qci) + .1])
        title('Quad Control Input Rotation')
        xlabel('Time (seconds)')
        ylabel('Rotation Around Z Axis (turning rate)')
        %legend('Rotation')

        % rotation task
        subplot(2,2,1), %plot_line(loss_task,r_t)
        hold on
        plot(task_time,r_t,'b+')
%         axis([0 task_time(mxt) + 0.001 min(r_t) - .1 max(r_t) + .1])
        grid on
        grid minor
        title(sprintf('Task Rotation of %s%i',vehicle,test))
        xlabel('Time (seconds)')
        ylabel('Rotation Around Z Axis (radians)')
        %legend('Rotation')

        % vicon rotation
        subplot(2,2,3), %plot_line(loss_task,r_z)
        hold on
        plot(vicon_time,r_w,'r.')
%         axis([0 vicon_time(mv) + 0.001 min(min(min(min(r_z,r_y),r_x),r_w)) - .1 max(max(max(max(r_z,r_y),r_x),r_w)) + .1])
        plot(vicon_time,r_z,'b.')
        plot(vicon_time,r_y,'c.')
        plot(vicon_time,r_x,'k.')
        grid on
        grid minor
        title('Vicon Rotation')
        xlabel('Time (seconds)')
        ylabel('Rotation Around an Axis (unit quaternion)')
        %legend('Rotation Around X','Rotation Around Y','Rotation Around Z')
        hold off
        % print figure
        if save_plot == 1
            print(sprintf('../plot/%s/%s%i/rotation_%s%i',...
                test_set,vehicle,test,vehicle,test),'-djpeg')
        end
    end

    %% state & status
    if view_plot == 1 
        % empty plot
        if no_data == 1
            loss_task = 0;
        end
        
        figure
        subplot(3,1,1), %plot_line(loss_task,state)
        hold on
        plot(command_state_time,command_state,'b+')
        plot(state_time,state,'k.')
        grid on
        grid minor
        axis([0 80 + 0.001 0 - .5 8 + .5])
%         axis([0 state_time(ms) + 0.001 0 - .5 8 + .5])
        title(sprintf('State Sequence of %s%i',vehicle,test))
        xlabel('Time (seconds)')
        ylabel('Current State')
        %legend('State')

        % binary status
        subplot(3,1,2), %plot_line(loss_task,motors_status)
        hold on
        plot(status_time,motors_status,'k-')
        grid on
        grid minor
        plot(status_time,serial_mode,'c-')
        plot(status_time,waypose_status,'bx')
%         axis([0 status_time(mss) + 0.001 -0.3 1.3])
        title('Binary Status Flags')
        xlabel('Time (seconds)')
        ylabel('Boolean Value')
        %legend('Motors','Serial Mode','Waypose')
        hold off

        % gps control Status
        subplot(3,1,3), %plot_line(loss_task,flying)
        hold on
        plot(qci_time,flying,'b.')
        grid on
        grid minor
%         axis([0 max(qci_time) + 0.001 -0.3 1.3])
        title('Quad Control Input GPS Control Status')
        ylabel('Currently Flying')
        xlabel('Time (seconds)')
        %legend('GPS Control Status')
        
        % print figure
        if save_plot == 1
            print(sprintf('../plot/%s/%s%i/state_status_%s%i',...
                test_set,vehicle,test,vehicle,test),'-djpeg')
        end  
    end

      
end

%% failure
fail_type(5,2) = failed;
fail_type(5,3) = n_test;
fail_type(5,4) = (failed/n_test)*100;
[fail_type(5,2) fail_type(5,4)]
flight_stats(1:5,:) = fail_type;
if failed > 1
    mtbf = mtbf(:,1:failed-1);
    [mean(mtbf(3,:)) median(mtbf(3,:))];
end
control_packet
% if view_plot == 1
%     figure
%     bar(fail_type(1:2,:))
%     grid on
%     grid minor
%     title('Failure Types')
%     ylabel('Number of Failures')
%     xlabel('ManContr LostSeri LstVicon SlwStart TaskSent TaskRece NRecTask NSendTsk')
%     
%     % print figure
%     if save_plot == 1
%         print(sprintf('plot/%s/failure_%s',test_set,vehicle),'-djpeg')
%     end
% end
%% Power
% average operating power
figure
subplot(2,1,1)

% segment pass and fail test runs and plot data
pass_count = 0;
fail_count = 0;
pass = zeros(n_test,4);
fail= zeros(n_test,4);
for i = 1:1:n_test
    if oap(i,3) == 0
        plot(i,oap(i,1),'k+')
        if i == 1
            hold on
        end
        plot(i,oap(i,2),'cx')
        pass_count = pass_count + 1;
        pass(pass_count,1:2) = oap(i,1:2);
        pass(pass_count,3:4) = task_power(i,:);
%         if oap(i,1) == 0 || oap(i,2) == 0
%             [vehicle]
%             [i]
%         end
    else
        if oap(i,1) > median(oap(:,1)) - 20
            plot(i,oap(i,1),'r+')
            if i == 1
                hold on
            end
            plot(i,oap(i,2),'rx')
            fail_count = fail_count + 1;
            fail(fail_count,1:2) = oap(i,1:2);
            fail(fail_count,3:4) = task_power(i,:);
        end
    end
end
hold off

pass = pass(1:pass_count,:);
fail = fail(1:fail_count,:);
if fail_count > 0
    if fail_count > 1
        fail_stats(1,:) = mean(fail);
        fail_stats(2,:) = sqrt(var(fail));
    else
        fail_stats(1,:) = fail;
        fail_stats(2,:) = sqrt(fail);
    end
%     fprintf('Average Power: Fail');
%     [mean(fail) sqrt(var(fail))]
end

% record power related statistics
power_stats = zeros(4,4);
if pass_count > 0
    power_stats(1,:) = mean(pass);
    power_stats(2,:) = var(pass);
    power_stats(4,1:2) = [min(min(pass(:,1:2))) max(max(pass(:,1:2)))];
%     fprintf('Pass');
%     [mean(pass) sqrt(var(pass))]
end
% add details to plot
grid on
grid minor
title('Average Operating Apparent Power')
xlabel('Test run')
ylabel('Power (Watts)')

subplot(2,1,2)
% maximum power
% segment pass and fail test runs and plot data
pass_count = 0;
fail_count = 0;
pass = zeros(n_test,2);
fail= zeros(n_test,2);
for i = 1:1:n_test
    if pmax(i,3) == 0
        plot(i,pmax(i,1),'k+')
        if i == 1
            hold on
        end
        plot(i,pmax(i,2),'cx')
        pass_count = pass_count + 1;
        pass(pass_count,:) = pmax(i,1:2);
    else
        if pmax(i,1) > 0 
            plot(i,pmax(i,1),'r+')
            if i == 1
                hold on
            end
            plot(i,pmax(i,2),'rx')
            fail_count = fail_count + 1;
            fail(fail_count,:) = pmax(i,1:2);
        end
    end
end
hold off
pass = pass(1:pass_count,:);
fail = fail(1:fail_count,:);
if fail_count > 0
    if fail_count > 1
        fail_stats(3,:) = [mean(fail) sqrt(var(fail))];
    else
        fail_stats(3,:) = [fail sqrt(fail)];
    end
%     fprintf('Maximum Power: Fail');
%     [mean(fail) sqrt(var(fail))]
end

% record power related statistics
if pass_count > 0
    power_stats(3,:) = [mean(pass) min(min(pass)) max(max(pass))];
    power_stats(4,3:4) = var(pass);
    flight_stats(6:9,:) = power_stats;
%     fprintf('Pass');
%     [mean(pass) sqrt(var(pass))]
end
% text(1, min(pmax(:,1)) + 0.7,...
%     sprintf('Maximum Power: Average = [%f %f],Range = [%f,%f]',...
%     power_stats(3,1),power_stats(3,2),power_stats(3,3),power_stats(3,4)))
grid on
grid minor
title('Maximum Apparent Power')
xlabel('Test run')
ylabel('Power (Watts)')

% print figure
if save_plot == 1
    print(sprintf('../plot/%s/maximum_power_%s',test_set,vehicle),'-djpeg')
end

%% time
% plot and segment pass and fail tests prior to generating statistics
figure
pass_count = 0;
fail_count = 0;
pass = zeros(n_test,1);
fail= zeros(n_test,1);
for i = 1:1:n_test
    if ot(i,2) == 0
        plot(i,ot(i,1),'kx')
        if i == 1
            hold on
        end
%         if ot(i,1) < 62.5 || ot(i,1) > 68.0
%             [i ot(i,1)]
%          end
        pass_count = pass_count + 1;
        pass(pass_count,1) = ot(i,1);       
    else
        if ot(i,1) > 0
            plot(i,ot(i,1),'rx')
            if i == 1
                hold on
            end
            fail_count = fail_count + 1;
            fail(fail_count,1) = ot(i,1);
        end
    end
    
end
pass = pass(1:pass_count,:);
fail = fail(1:fail_count,:);
if fail_count > 0
    if fail_count > 1
        fail_stats(6,:) = [mean(fail) sqrt(var(fail)) 0 0];
    else
        fail_stats(6,:) = [fail sqrt(fail) 0 0];
    end
end

% calculate and store time
time_stats = zeros(1,4);
if pass_count > 0
    time_stats(1,:) = [mean(pass) var(pass) min(min(pass)) max(max(pass))];
end

flight_stats(14,:) = time_stats;

% format plot
text(1, min(ot(:,1)) + 0.1,sprintf('Average = %f, Range = [%f %f]',...
    time_stats(1,1),time_stats(1,3),time_stats(1,4)))

% edit plot
grid on
grid minor
title('Time to Perform Task Set')
xlabel('Test run')
ylabel('Time (secs)')
if save_plot == 1
    print(sprintf('../plot/%s/total_time_%s',test_set,vehicle),'-djpeg')
end

%trans_time
%[max(trans_time(:,:, 2)); min(trans_time(:,:, 2));mean(trans_time(:,:, 2));mode(round(trans_time(:,:, 2)))]

% plot and segment pass and fail tests prior to generating statistics
figure
pass_count = 0;
fail_count = 0;
pass = zeros(n_test,1);
fail= zeros(n_test,1);
for i = 1:1:n_test
    if ot(i,2) == 0
        plot(i,ot(i,3),'kx')
        if i == 1
            hold on
        end
        pass_count = pass_count + 1;
        pass(pass_count,1) = ot(i,3);       
    else
        if ot(i,1) > 0
            plot(i,ot(i,3),'rx')
            if i == 1
                hold on
            end
            fail_count = fail_count + 1;
            fail(fail_count,1) = ot(i,3);
        end
    end
    
end
% edit plot
grid on
grid minor
title('Time to Perform Task Set')
xlabel('Test run')
ylabel('Time (secs)')

% print figure
if save_plot == 1
    print(sprintf('../plot/%s/flight_time_%s',test_set,vehicle),'-djpeg')
end
%% position
% [max_x max_y max_r]
% plot and segment pass and fail tests prior to generating statistics
figure
pass_count = 0;
fail_count = 0;
pass = zeros(n_test,4);
fail= zeros(n_test,4);
most_precise = zeros(1,2);
most_precise(1,1) = 1000;
total_precision = 0;
for i = 1:1:n_test
    if precision(i,5) == 0
        plot(i,precision(i,1)/ot(i,1)*1000,'k.')
        if i == 1
            hold on
        end
        plot(i,precision(i,2)/ot(i,1)*1000,'cx')
        plot(i,precision(i,3)/ot(i,1)*1000,'b+')
        pass_count = pass_count + 1;
        pass(pass_count,:) = precision(i,1:4);
        total_precision = (sqrt(precision(i,1)^2 + precision(i,2)^2 +...
            precision(i,3)^2))/ot(i,1)*1000; 
        if total_precision < most_precise(1,1)
            most_precise(1,1) = total_precision;
            most_precise(1,2) = i;
        end
            
        %[i precision(i,1:3)/ot(i,1)*1000 precision(i,4)/ot(i,1)]
    else
        if precision(i,1) > 0
            plot(i,precision(i,1)/ot(i,1)*1000,'r.')
            if i == 1
                hold on
            end
            plot(i,precision(i,2)/ot(i,1)*1000,'rx')
            plot(i,precision(i,3)/ot(i,1)*1000,'r+')
            fail_count = fail_count + 1;
            fail(fail_count,:) = precision(i,1:4);
        end
    end
end
[most_precise(1,2) most_precise(1,1)]
pass = pass(1:pass_count,:);
fail = fail(1:fail_count,:);
if fail_count > 0
    if fail_count > 1
        fail_stats(4,:) = mean(fail)/fail_stats(6,1);
        fail_stats(5,:) = sqrt(var(fail));
    else
        fail_stats(4,:) = fail/fail_stats(6,1);
        fail_stats(5,:) = sqrt(fail);
    end
    fail_stats(4,1:3) = fail_stats(4,1:3)*1000    
end

% calculate and store position statistics
error_stats = zeros(4,4);
if pass_count > 0
    error_stats(1,:) = mean(pass);%pass
    error_stats(1,1:3) = error_stats(1,1:3)*100;
    error_stats(4,1:3) = error_stats(1,1:3)*10/time_stats(1,1);
    error_stats(4,4) = error_stats(1,4)/time_stats(1,1);
    error_stats(2,:) = var(pass);
    total_error = pass(:,1).*pass(:,1) + pass(:,2).*pass(:,2) + pass(:,3).*pass(:,3);
    error_stats(3,:) = [min(min(pass(:,1:3)))*100 max(max(pass(:,1:3)))*100 0 mean(sqrt(total_error))*100];    
end
flight_stats(10:13,:) = error_stats(1:4,:);
flight_stats(12,3) = error_stats(3,4)*10/time_stats(1,1)

grid on
grid minor
title('Position Precision Estimate')
xlabel('Test run')
ylabel('Position (Millimeters)')

% print figure
if save_plot == 1
    print(sprintf('../plot/%s/position_error_%s',test_set,vehicle),'-djpeg')
end
