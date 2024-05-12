function [target] = test_heading(wp_from, wp_to)
%

PI = pi;
lat1 = (wp_from(1))*(PI/180);
lon1 = (wp_from(2))*(PI/180);
lat2 = (wp_to(1))*(PI/180);
lon2 = (wp_to(2))*(PI/180);


y = sin(lon2-lon1) * cos(lat2);
x = cos(lat1).*sin(lat2) - sin(lat1).*cos(lat2).*cos(lon2-lon1);
theta = atan2(y, x);
target =  mod(((theta.*180/PI + 360)* 1E5), 360E5) ;

if (target < (-180E5)) 
    target = target + 360E5;
elseif (target > (180E5))
    target = target - 360E5;
end 
target = target/1E5;
end