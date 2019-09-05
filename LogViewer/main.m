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
% 
% ypr = data(:,30:32);
% ypr = ypr/16;
% 
% ypr = ypr-min(ypr);
% scale_factor = max(ypr);
% ypr = ypr./scale_factor;
% ypr = ypr*2*pi;
% ypr = unwrap(ypr);
% ypr = ypr/2/pi;
% ypr = ypr.*scale_factor;
% ypr = ypr-mean(ypr);
% data(:,30:32) = ypr;
% 
% 
% 
% 
% 
% 
% tt = data(:,1)/1e7;
% ts = mean(tt(2:end)-tt(1:end-1));
% fs = 1/ts;
% fc = 1;
% Wn = fc/(fs/2);
% 
% % Make a filter with cutoff freq fc
% [B,A] = butter(3,Wn);
% 
% dp = data(:,26:29);
% dp = dp-mean(dp);
% dp = filter(B,A,dp);
% data(:,26:29) = dp;
% 
% ws = data(:,36:39);
% ws(ws>1300) = 0;
% ws = filter(B,A,ws);
% ws = ws/60*2*pi*9*2.54/100*3.6;
% data(:,36:39) = ws;
% 
% acc = data(:,33:35);
% acc = filter(B,A,acc);
% data(:,33:35) = acc;
% 
% sr = data(:,11);
% sr = filter(B,A,sr);
% data(:,11) = sr;
% 
% data(:,13) = filter(B,A,data(:,13));
% data(:,5) = filter(B,A,data(:,5));


runGraphsTime(data, names, grouping);


normalized_data = data-min(data);
normalized_data = normalized_data./max(normalized_data);
normalized_data = normalized_data - mean(normalized_data);

Q = 0;
for ii = 1:size(dd,2)
    Pi = normalized_data(ii,:)';
    Q = Q + Pi*(Pi');
end

[V,D] = eig(Q);
% The vector with the biggest eigenvalue is the first principal component
P1 = V(:,end);
[~,I] = sort(abs(P1));
sP1 = P1(I);
snames = names(I);
for ii = 1:length(sP1)
    disp(snames(ii));
    disp(sP1(ii));
    disp(' ');
end