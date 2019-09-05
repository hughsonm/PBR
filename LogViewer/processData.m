function [data,names]   = processData(dataFile)

rawData = csvread(dataFile,1,0);
fid = fopen(dataFile);
names = fgetl(fid);
fclose(fid);
qIdcs = [0,find(names == ','),length(names)];
for ii = 1:(length(qIdcs)-1)
    start = qIdcs(ii)+1;
    finish = qIdcs(ii+1)-1;
    headers{ii} = names(start:finish);    
end

data = rawData;
names = headers;


% rawData = import(dataFile,1,0);
%get gyro units correct

%find last GPS entry and stop data there or errors occur
% dataEnd = find(rawData(:,8),1,'last');
% 
% Time = 0.0000001*( rawData([dataDelay:dataEnd], 1) - rawData(dataDelay, 1));
% % XAccel = rawData([dataDelay:dataEnd], 31)/9.81;
% % YAccel = rawData([dataDelay:dataEnd], 32)/9.81;
% % XYAccel = sqrt(XAccel.^2 + YAccel.^2);
% % ZGyro = rawData([dataDelay:dataEnd], 35); %unknown units
% % Steering = rawData([dataDelay:dataEnd], 5);
% Gas = rawData([dataDelay:dataEnd], 2)*0.01;
% 
% Brake1 = rawData([dataDelay:dataEnd], 28);
% Brake2 = rawData([dataDelay:dataEnd], 29)-90;
% Brake = [Brake1 Brake2];
% % Lat = rawData([dataDelay:dataEnd], 8)*0.0000001;
% % Long = rawData([dataDelay:dataEnd], 9)*0.0000001;
% % GPSAccuracy = rawData([dataDelay:dataEnd], 10)*0.001;
% % TimeOfWeek = rawData([dataDelay:dataEnd], 11)*1;
% CTemp = rawData([dataDelay:dataEnd], 17)-273;
% BTemp = rawData([dataDelay:dataEnd], 30:31);
% speed = [rawData([dataDelay:dataEnd], 5)*.001436*.84*1.6 rawData([dataDelay:dataEnd], 6:8)*.001436*1.6]; 
% oilPress = rawData([dataDelay:dataEnd], 4);
% RPM = sgolayfilt(rawData([dataDelay:dataEnd], 3), 1, 5);
% lambda = rawData([dataDelay:dataEnd], 11);
% 
% 
% %filter the raw accelerometer values
% %2
%         xaccelFilt = designfilt('lowpassiir', 'PassbandFrequency', 6, ...
%             'StopbandFrequency', 10, 'PassbandRipple', 1, ...
%             'StopbandAttenuation', 60, 'SampleRate', 100, ...
%             'DesignMethod', 'butter', 'MatchExactly', ...
%             'passband');
% %3
%         yaccelFilt = designfilt('lowpassiir', 'PassbandFrequency', 6, ...
%             'StopbandFrequency', 10, 'PassbandRipple', 1, ...
%             'StopbandAttenuation', 60, 'SampleRate', 100, ...
%             'DesignMethod', 'butter', 'MatchExactly', ...
%             'passband');
% %12
%         combinedFilt = designfilt('lowpassiir', 'PassbandFrequency', 7, ...
%             'StopbandFrequency', 10, 'PassbandRipple', 1, ...
%             'StopbandAttenuation', 60, 'SampleRate', 100, ...
%             'DesignMethod', 'butter', 'MatchExactly', ...
%             'passband');
% % 1:time
% 2:X accel 
% 3:Y accel
% 4:combined accel
% 5:Z gyro
% filteredAccel = [Time  filtfilt(xaccelFilt, XAccel), filtfilt(yaccelFilt, YAccel),...
%     filtfilt(combinedFilt, XYAccel) sgolayfilt(ZGyro,1,5)];

% Pedals position
% 1:time
% 2:throttle
% 3:brake
% brake = 0.0001159*exp(8*Brake/2863);
% throttle = 0.398*exp(.00003743*Gas) + -72760*exp(-0.0009159*Gas);
% filteredPedals = [Time, throttle(:,1), brake;];
% filteredPedals = [filteredPedals(:,1), sgolayfilt(filteredPedals(:,2:3),1,9)];
% 
% % See cosworth pdf for details
% % 562 is 9.81m/s * 57deg/rad
% % .9 is my own adjustment factor
% % oversteer pos understeer neg
% attitudeVelocity = [ZGyro, (YAccel ./ interpolatedVelocity)*562];
% attitudeVelocity = [abs(attitudeVelocity(:,1)) - abs(attitudeVelocity(:,2)) + 1.6];
% 
% % Steering Position
% % 1:time
% % 2:filtered steering position
% % 3: understeer angle (oversteer pos understeer neg)
% % 4: attitude Velocity
% % understeer angle multiplication factors:
% % 14.9 steering ratio
% % 57 rads to degrees
% % 2.819 wheel base
% % 9.8m/s^2
% filteredSteering = [filteredAccel(:,1) Steering, sgolayfilt(abs(Steering) - abs((23462 * 1.3 * filteredAccel(:,3) ./ bsxfun(@power,interpolatedVelocity,2))),1,5)];
% 
% filteredSteering = [filteredSteering(:,:), sgolayfilt(attitudeVelocity,1,31)];
% filteredSteering(a,[3 4]) = 0;
% 
% 
% % distance travelled interpolated to 100hz with start distance interpolated
% distance = [Time interp1(GPSPos(:,1), dist, Time)];
% nans = sum(isnan(distance(:,2)));
% distanceadd = [distance(1:nans,1) interp1([0; distance(nans+1,1)], [0; dist(1)], distance(1:nans,1))];
% distance = [distanceadd; distance(nans+1:end,:)];

end