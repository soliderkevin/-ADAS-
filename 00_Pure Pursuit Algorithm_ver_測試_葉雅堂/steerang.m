function delta_steer = steerang( current_map_location, target_map_location, L )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%   delta_steer 順時為正，逆時針為負
[alpha, delta_location_map] = ppalpha(current_map_location,target_map_location);
temp1=2*L*sin(alpha);
temp2=norm(delta_location_map);
delta_steer = atan2(temp1,temp2);
% temp=[temp2, temp1];
% delta_steer = my_yaw_angle( temp );





function  [alpha, delta_location_map] = ppalpha(current_map_location,target_map_location)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%   current_map_location = [x; y; heading]
%   target_map_location = [x; y]
%   alpha 順時為正，逆時針為負

theta = current_map_location(1,3);  % heading angle
RM_map2car = [cos(theta), sin(theta);-sin(theta), cos(theta)];
delta_location_map = target_map_location-current_map_location(1,1:2);
target_car_location = RM_map2car*delta_location_map';
alpha = atan2(target_car_location(2),target_car_location(1));
% alpha = my_yaw_angle( target_car_location );
