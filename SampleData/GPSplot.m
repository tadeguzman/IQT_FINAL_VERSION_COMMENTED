% // Theo DeGuzman
% // Johns Hopkins University
% // IQT24 Senior Design Project
% // V2 Prototype Code
% // Not for production use -- end user assumes responsibility for usage and testing of this code

% Sample analysis code for the visualization of CSV results

clear; close all;

% [~,~,data] = xlsread('.\EclipseReturn\CL040821.CSV');
% [~,~,data1] = xlsread('.\EclipseReturn\CL040822.CSV');
% [~,~,data2] = xlsread('.\EclipseReturn\CL040823.CSV');
% [~,~,data3] = xlsread('.\EclipseReturn\CL040900.CSV');
% [~,~,data4] = xlsread('.\EclipseReturn\CL040911.CSV');
% [~,~,data5] = xlsread('.\EclipseReturn\CL040912.CSV');
% [~,~,data6] = xlsread('.\EclipseReturn\CL040913.CSV');
% [~,~,data7] = xlsread('.\EclipseReturn\CL040914.CSV');
% [~,~,data8] = xlsread('.\EclipseReturn\CL040915.CSV');
% [~,~,data9] = xlsread('.\EclipseReturn\CL040916.CSV');
% 
% [~,~,data_b] = xlsread('.\EclipseReturn\BL040821.CSV');
% [~,~,data_b1] = xlsread('.\EclipseReturn\BL040822.CSV');
% [~,~,data_b2] = xlsread('.\EclipseReturn\BL040823.CSV');
% [~,~,data_b3] = xlsread('.\EclipseReturn\BL040900.CSV');
% [~,~,data_b4] = xlsread('.\EclipseReturn\BL040911.CSV');
% [~,~,data_b5] = xlsread('.\EclipseReturn\BL040912.CSV');
% [~,~,data_b6] = xlsread('.\EclipseReturn\BL040913.CSV');
% [~,~,data_b7] = xlsread('.\EclipseReturn\BL040914.CSV');
% [~,~,data_b8] = xlsread('.\EclipseReturn\BL040915.CSV');
% [~,~,data_b9] = xlsread('.\EclipseReturn\BL040916.CSV');

%% Collect all the files to pull from 
[~,~,data] = xlsread('.\pull3\CL041416.CSV');
[~,~,data1] = xlsread('.\pull3\CL041417.CSV');
[~,~,data2] = xlsread('.\pull3\CL041418.CSV');

[~,~,data_b] = xlsread('.\pull3\BL041416.CSV');
[~,~,data_b1] = xlsread('.\pull3\BL041417.CSV');
[~,~,data_b2] = xlsread('.\pull3\BL041418.CSV');


%% aggregate the files
% data = [data; data1; data2; data3; data4; data5; data6; data7; data8; data9];
% data_b = [data_b; data_b1; data_b2; data_b3; data_b4; data_b5; data_b6; data_b7; data_b8; data_b9];

data = [data; data1; data2];
data_b = [data_b; data_b1; data_b2];

% Set start and end
sd = size(data);
sb = size(data_b);
% num = min(sd, sb);
num = 5000;
start = 1;

%% Extract data from CSVs
lats = cell2mat(data(start:num, 7))./1E7;
lons = cell2mat(data(start:num, 8))./1E7;
speed = cell2mat(data(start:num, 9))./1E3;
heading = cell2mat(data(start:num, 4))./1E5;
heading2 = cell2mat(data(start:num, 10))./1E5;
target_heading = cell2mat(data_b(start:num, 12))./1E5;
err_heading = cell2mat(data_b(start:num, 14))./1E5;
steering = cell2mat(data_b(start:num, 15));
t_lat = cell2mat(data_b(start:num, 3))./1E7;
t_lon = cell2mat(data_b(start:num, 4))./1E7;

nav_data = [heading, target_heading, err_heading, steering];


%% Calculate derived points
test_h = zeros(size(heading));
corr_h = zeros(size(heading));
test_d = zeros(size(heading));

for i = 1:size(heading)
    test_h(i) = test_heading([lats(i) lons(i)], [39.2531741 -76.4894198]);
end
for i = 1:size(heading)
    test_d(i) = test_dist([lats(i) lons(i)], [t_lat(i) t_lon(i)])/100;
end
for i = 2:size(heading)
    corr_h(i) = test_heading([lats(i-1) lons(i-1)], [lats(i) lons(i)]);
end

num = size(lats);

steer = cell2mat(data_b(1:num, 15));
trim = cell2mat(data_b(1:num, 17));
err_bng = cell2mat(data_b(1:num, 14))./1E5;

PI = pi;

% for derived heading vector 
vlats = (asin(sin(lats*PI/180).*cos(speed./6378100) + cos(lats*PI/180).*sin(speed./6378100).*cos(heading*PI/180)))*180/PI;
vlons = (lons*PI/180+atan2(sin(heading*PI/180).*sin(speed./6378100).*cos(lats*PI/180), cos(speed./6378100)-sin(lats*PI/180).*sin(vlats*PI/180)))*180/PI;

% for gps heading vector
glats = (asin(sin(lats*PI/180).*cos(speed./6378100) + cos(lats*PI/180).*sin(speed./6378100).*cos(heading2*PI/180)))*180/PI;
glons = (lons*PI/180+atan2(sin(heading2*PI/180).*sin(speed./6378100).*cos(lats*PI/180), cos(speed./6378100)-sin(lats*PI/180).*sin(glats*PI/180)))*180/PI;

% for target heading vector
tlats = (asin(sin(lats*PI/180).*cos(test_d./6378100) + cos(lats*PI/180).*sin(test_d./6378100).*cos(target_heading*PI/180)))*180/PI;
tlons = (lons*PI/180+atan2(sin(target_heading*PI/180).*sin(test_d./6378100).*cos(lats*PI/180), cos(test_d./6378100)-sin(lats*PI/180).*sin(tlats*PI/180)))*180/PI;

% for steering vector
slats = (asin(sin(lats*PI/180).*cos(speed./6378100) + cos(lats*PI/180).*sin(speed./6378100).*cos((heading+steer)*PI/180)))*180/PI;
slons = (lons*PI/180+atan2(sin((heading+steer)*PI/180).*sin(speed./6378100).*cos(lats*PI/180), cos(speed./6378100)-sin(lats*PI/180).*sin(slats*PI/180)))*180/PI;


% Trim down to useful data
lats = lats(lons < -1);
t_lat = t_lat(lons < -1);
t_lon = t_lon(lons < -1);
tlats = tlats(lons < -1);
tlons = tlons(lons < -1);
glats = glats(lons < -1);
glons = glons(lons < -1);
vlats = vlats(lons < -1);
vlons = vlons(lons < -1);
slats = slats(lons < -1);
slons = slons(lons < -1);
steer = steer(lons < -1);
trim = trim(lons < -1);
err_bng = err_bng(lons < -1);
heading = heading(lons < -1);
target_heading = target_heading(lons < -1);
lons = lons(lons < -1);


%% Plot visuals of interest
figure(1);
for i = 1:length(lons)
    geoplot([lats(i) tlats(i)], [lons(i) tlons(i)],'LineWidth',1,'Color','c'); % target heading vector
    hold on;    
    geoplot(t_lat(i), t_lon(i),"X",'Color','r'); % Aim points
    geoplot([lats(i) vlats(i)], [lons(i) vlons(i)],'LineWidth',1,'Color','b'); % current velocity direction vector
    geoplot([lats(i) glats(i)], [lons(i) glons(i)],'LineWidth',1,'Color','g'); % GPS heading vector
    geoplot([lats(i) slats(i)], [lons(i) slons(i)],'LineWidth',1,'Color','r'); % steering output vector
end

geoplot(lats, lons, "k--o"); % Current positions


geobasemap topographic;


figure(2);
scatter(err_bng, steer+trim);
xlabel('Bearing Error Value') 
ylabel('Steer + Trim Value')

figure(3);
plot(1:length(heading), heading);
hold on;
plot(1:length(heading), target_heading);
plot(1:length(heading), err_bng);
xlabel('Time') 
ylabel('Degrees')

figure(4);
plot(1:length(speed), speed);
xlabel('Time') 
ylabel('Speed')



