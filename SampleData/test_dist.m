function [d] = test_dist(wp_from, wp_to)
%

PI = pi;
lat1 = (wp_from(1))*(PI/180);
lon1 = (wp_from(2))*(PI/180);
lat2 = (wp_to(1))*(PI/180);
lon2 = (wp_to(2))*(PI/180);


R = 6371E3;
d = (R * acos(sin(lat1)*sin(lat2)+cos(lat1)*cos(lat2)*cos(lon1-lon2))); % in m

end