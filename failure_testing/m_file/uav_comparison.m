function uav_comparison(select_vehicle,save_plot,view_plot,fname)
%uav_comparison returns UAV stats
%   Output: Excel file with failure, power, position, and time statistics
%   of the selected unmanned aerial systems tested
n = length(select_vehicle');
comparison =  zeros(n,14,4);
fail_stats = zeros(n,6,4);

% non volatile column values
vehicles = char({'CRASH'; 'HERBIE'; 'JENNY'; 'UNL_COMP_SCI'; 'HULK_SMASH'; 'MORPHEUS'});
propellers = char({'AscTec Safety Propellers'; 'ARDrone Propellers';...
    'AscTec Normal Propellers'; 'AscTec Safety Propellers';...
    'AscTec Safety Propellers'; 'ARDrone Propellers'});
firmwares = char({'AutoPilot LowLevel #20636 V3.12, AutoPilot HighLevel V3.11';...
    'AutoPilot LowLevel #20632 V3.12, AutoPilot HighLevel V3.0';...
    'AutoPilot LowLevel #20633 V2.14, AutoPilot HighLevel V3.0';...
    'AutoPilot LowLevel #20637 V3.12, AutoPilot HighLevel V3.0';...
    'AutoPilot LowLevel #20502 V3.12, AutoPilot HighLevel V3.11';...
    'ARDrone'});
costs = [inf; inf; inf; inf; inf; inf];

% variable row selection matrices 
vehicle=char(zeros(n,length(vehicles(1,:))));
propeller=char(zeros(n,length(propellers(1,:))));
firmware=char(zeros(n,length(firmwares(1,:))));
cost=zeros(n,1);

% determine flight characteristics of selected vehicles
for j = 1:1:n 
    i = select_vehicle(j);
    vehicle(j,:) = vehicles(i,:);
    propeller(j,:) = propellers(i,:);
    firmware(j,:) = firmwares(i,:);
    cost(j,:) = costs(i,1);

    % determine flight characteristics of selected vehicle for a specific range of tests 
    if i == 1
        [comparison(j,:,:), fail_stats(j,:,:)]=plot_topic('crash','crash',167,168,save_plot,view_plot);
    elseif i == 2
        [comparison(j,:,:)]=plot_topic('herbie','herbie',1,100,save_plot,view_plot);
    elseif i == 3
        [comparison(j,:,:)]=plot_topic('jenny','jenny',87,88,save_plot,view_plot);
    elseif i == 4
        [comparison(j,:,:)]=plot_topic('unl_comp_sci','unl_comp_sci',1,100,save_plot,view_plot);
    elseif i == 5
        [comparison(j,:,:)]=plot_topic('hulk_smash','hulk_smash',1,50,save_plot,view_plot);
    elseif i == 6
        [comparison(j,:,:)]=plot_topic('morpheus','morpheus',1,16,save_plot,view_plot);
    end
end

%% Failure Statistics Arrays
reliability = comparison(:,5,4);
failedTests = comparison(:,5,2);
totalTests = comparison(:,5,3);
manualControl = comparison(:,1,1);
serialCommunicationLost = comparison(:,1,2);
viconDelay = comparison(:,1,3);
viconLostObject = comparison(:,1,4);
extraTask = comparison(:,2,1);
missingTask = comparison(:,2,2);
missingState = comparison(:,2,4);
extraState = comparison(:,2,3);
motorsStayedOn = comparison(:,3,1);
%competingPIDControllers = comparison(:,3,2);
subjectStatusDelay = comparison(:,3,3);
commandStateDelay = comparison(:,3,4);
vehicleFlipped = comparison(:,4,1);
quadControlInputDelay = comparison(:,4,2);
subjectPoseDelay = comparison(:,4,3);
taskedPoseDelay = comparison(:,4,4);

 title_fail = {'Vehicle Type and Failure Statistics'};
 units_fail = {'(type)' '(version)' '(U.S. Dollars)' '(%)' '(count)' ...
     '(count)' '(count)' '(count)' '(count)' '(count)' '(count)' '(count)'...
     '(count)' '(count)' '(count)' '(count)' '(count)' '(count)' '(count)'...
     '(count)' '(count)' '(count)'};
 uas_fail = table(vehicle,propeller,firmware,cost,reliability,failedTests,...
     totalTests,manualControl,serialCommunicationLost,...
     extraTask,missingTask,extraState,missingState,...
     quadControlInputDelay,commandStateDelay,taskedPoseDelay,...
     subjectPoseDelay,viconDelay,viconLostObject,motorsStayedOn,...
     vehicleFlipped,subjectStatusDelay);
 
% Power Statistics Arrays
meanPower = (comparison(:,6,1)+comparison(:,6,2))/2;
moveMean = comparison(:,6,3);
hoverMean = comparison(:,6,4);
meanPowerRange = comparison(:,9,1:2);
meanPowerVariance = (comparison(:,7,1)+comparison(:,7,2))/2;
movePowerVariance = comparison(:,7,3);
hoverPowerVariance = comparison(:,7,4);
maximumPower = (comparison(:,8,1)+comparison(:,8,2))/2;
maximunPowerRange = comparison(:,8,3:4);
maximumPowerVariance = (comparison(:,9,3)+comparison(:,9,4))/2;

 title_power = {'Power Statistics'};
 units_power = {'(Watts)' '(Watts)' '(Watts)' '(Watts)' '(Watts)' ' '...
     ' ' ' ' '(Watts)' '(Watts)' '(Watts)' ' '};
 uas_power = table(vehicle,meanPower,moveMean,hoverMean,meanPowerRange,...
     meanPowerVariance,movePowerVariance,hoverPowerVariance,maximumPower,...
     maximunPowerRange,maximumPowerVariance);

 % Position Statistics Arrays
 totalSpatialError = comparison(:,12,4);
totalMeanError_x = comparison(:,10,1);
totalMeanError_y = comparison(:,10,2); 
totalMeanError_z = comparison(:,10,3);
totalMeanError_rotation = comparison(:,10,4);
spatialErrorPerTask = comparison(:,12,3);
meanErrorPerTask_x = comparison(:,13,1);
meanErrorPerTask_y = comparison(:,13,2);
meanErrorPerTask_z = comparison(:,13,3);
meanErrorPerTask_rotation = comparison(:,13,4);
varianceErrorPerTask_x = comparison(:,11,1);
varianceErrorPerTask_y = comparison(:,11,2); 
varianceErrorPerTask_z = comparison(:,11,3);
varianceErrorPerTask_rotation = comparison(:,11,4);

 title_position = {'Position Error Statistics'};
 units_position = {'(centimeters - cm)' '(cm)' '(cm)' '(cm)' '(radians)'...
     '(millimeters - mm)' '(mm)' '(mm)' '(mm)' '(radians)' ' ' ' ' ' '};
 uas_position = table(vehicle,totalSpatialError,...
     totalMeanError_x,totalMeanError_y,totalMeanError_z,...
     totalMeanError_rotation,spatialErrorPerTask,meanErrorPerTask_x,...
     meanErrorPerTask_y,meanErrorPerTask_z,meanErrorPerTask_rotation,...
     varianceErrorPerTask_x,varianceErrorPerTask_y,varianceErrorPerTask_z,...
     varianceErrorPerTask_rotation);
 
 % Time Statistics Arrays
timeMean = comparison(:,14,1);
minimumTime = comparison(:,14,3);
maximumTime = comparison(:,14,4);
timeVariance = comparison(:,14,2);

 title_time = {'Test Execution Time Statistics'};
 units_time = {'(seconds - s)' '(s)' '(s)' ' '};
 uas_time = table(vehicle,timeMean,...
     minimumTime,maximumTime,...
     timeVariance);

%% write to excel file: loop possibly
base=3;
xlswrite(fname,title_fail,1,sprintf('G%i',0*n+base-2))
xlswrite(fname,units_fail,1,sprintf('B%i',0*n+base-1))
writetable(uas_fail,fname,'Sheet',1,'Range',...
    sprintf('A%i:V%i',base,n+base))

space = 4;
base = base + space;
xlswrite(fname,title_power,1,sprintf('G%i',1*n+base-2))
xlswrite(fname,units_power,1,sprintf('B%i',1*n+base-1))
writetable(uas_power,fname,'Sheet',1,'Range',...
    sprintf('A%i:M%i',n+base,2*n+base))

base = base + space;
xlswrite(fname,title_position,1,sprintf('G%i',2*n+base-2))
xlswrite(fname,units_position,1,sprintf('B%i',2*n+base-1))
writetable(uas_position,fname,'Sheet',1,'Range',...
    sprintf('A%i:O%i',2*n+base,3*n+base))

base = base + space;
xlswrite(fname,title_time,1,sprintf('C%i',3*n+base-2))
xlswrite(fname,units_time,1,sprintf('B%i',3*n+base-1))
writetable(uas_time,fname,'Sheet',1,'Range',...
    sprintf('A%i:E%i',3*n+base,4*n+base))

% output "finish" executing in French
fprintf('fini\n')
end
