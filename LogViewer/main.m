close all;
clear;

% Grouping Explanation:
% The matrix 'grouping' determines what gets initially displayed.
% The indices in column 1 will be plotted in subplot 1.
% The indices in column 2 will be plotted in subplot 2.
% etc.

% If this is your first time here, then just leave grouping as is.

grouping = [
    1 2 3;
    4 5 6
];

[log_file,log_path] = uigetfile('*.csv');
[data,names] = processData([log_path,log_file]);

% Uncomment these lines if you don't want to use the GUI to find your log
% file.
% log_filename = './Logs/GimliAutoXAug2018/Gimli7.csv';
% [data,names] = processData(log_filename);

% Show the indices and colum names
for nn = 1:length(names)
    disp(strcat(num2str(nn),': ',names(nn)));
    disp(' ');
end

% Plotting Explanation:
% The plotting function will scale different-sized data sets so
% they all fit on the same plot. It does this by dividing by a 
% power of ten. For example, if TPS and RPM are shown on the 
% same plot, TPS will be divided by 100 and rpm by 10000.

% EGT data are in column 43,44,45,and 46. Give me all the rows in the
% columns 43,44,45,and 46
% egts = data(:,43:46); 
% % Any values in egts greater than 1000 get replaced with zero.
% egts(egts>1000) = 0;
% % put the modified egt values back into data so that they will be used by
% % runGraphsTime
% data(:,43:46) = egts;

runGraphsTime(data, names, grouping);