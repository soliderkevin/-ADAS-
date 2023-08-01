% This program is licensed under the author Prof. Chien-Feng Wu. See the LICENSE file for details.
% Unless required by applicable law or agreed to in writing, this software is provided on an "AS IS" BASIS, WITHOUT 
% WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied, including, without limitation, any warranties or 
% conditions of TITLE, NON-INFRINGEMENT, MERCHANTABILITY, or FITNESS FOR A PARTICULAR PURPOSE.
% In no event shall the copyright owner or contributors be liable for any direct, indirect, incidental, special, 
% exemplary, or consequential damages (including, but not limited to, procurement of substitute goods or services; 
% loss of use, data, or profits; or business interruption) however caused and on any theory of liability, whether 
% in contract, strict liability, or tort (including negligence or otherwise) arising in any way out of the use of this 
% software, even if advised of the possibility of such damage.
% This software is subject to the terms and conditions of the license.
% This program is just for academic purposes. Not for commercial research and development purposes
% [Owner name]: Chien-Feng Wu, Assistant Professor of Tamkang University
% http://www.ee.tku.edu.tw/%e5%b0%88%e4%bb%bb%e5%b8%ab%e8%b3%87/#1490079920986-11f49204-8df1
% Copyright (C). All rights reserved.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc
close all
global DT

%%%%%%%%%%%%%%%%%%%%%% load Racing Loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% load SpaFrancorchamps
load Monza

[m,~]=size(X);
DT=0.1;
k=1;
path_length(1)=0;
for i=2:m
    temp_y=Y(i,1)-Y(i-1,1);
    temp_x=X(i,1)-X(i-1,1);
    temp_phi=atan2(temp_y,temp_x);
    phi(k,1)=mod(temp_phi,2*pi);
    temp_s=temp_x^2+temp_y^2;
    s_arc(k,1)=sqrt(temp_s);
    path_length(i)=path_length(i-1)+s_arc(k,1);
    k=k+1;
end
remainingdist=flip(path_length);
X_map(:,1)=X(2:end,1)'; 
X_map(:,2)=Y(2:end,1)'; 
X_map(:,3)=phi;
X_map(:,4)=remainingdist(1:end-1)';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

L=4.5;
velocity=55;
X_now=X_map(1,1:3);   
X_car=X_now;
Steer_Angle_deg=0;
DT=0.05;
figure(1)
hold on
plot(X_map(:,1),X_map(:,2),'ro');
plot(X_now(1,1),X_now(1,2),'ko');
givenpath=X_map;
last_closest_index=0;
max_index=size(givenpath,1);
i=1;
loop=0;
desired_loop=1;
while  loop<desired_loop  
    tk(i,1)=i*DT;
    [heading_point,closest_index] = lookahead(velocity, X_now, givenpath); % 決定預視點 heading point [x_map,y_map]^T
    delta_steer = steerang( X_now, heading_point, L );                      % 算輪胎轉角 delta
    X_next = vehicle_dynamic(X_now,delta_steer,velocity,DT);                % 更新車輛動態 X_now := [x,y, heading]^T
    X_car=[X_car;X_next];
    X_now = X_next;
    
    %  跳脫設定 %%  currentlt location after start-line and last location before finish-line
   if closest_index<=15&&last_closest_index>=0.95*max_index
      disp("Finish one loop") 
      loop=loop+1;
%       break;
   end
        last_closest_index=closest_index;
        i=i+1;
end

total_time=tk(end,1);

scenario = drivingScenario;
roadCenters = [X, Y];
roadWidth = 10;
road(scenario,roadCenters,roadWidth);
rbdry=roadBoundaries(scenario);
ego_car=vehicle(scenario);
waypoints=roadCenters;
speed=45;
chasePlot(ego_car);
car_path=X_car(:,1:2);
trajectory(ego_car, car_path, speed);
scenario.SampleTime = 0.01;
scenario.StopTime = total_time;


while advance(scenario)
    pause(0.001)
end


function X = vehicle_dynamic(X_now,delta,velocity,DT)
  L=4.5;
  X(1,1) = X_now(1,1)+velocity*cos(X_now(1,3))*DT; % car_in_map _x
  X(1,2) = X_now(1,2)+velocity*sin(X_now(1,3))*DT; % car_in_map_y
  X(1,3) = X_now(1,3)+velocity/L*(tan(delta))*DT;  % vehicle currentlt heading angle in map
  X(1,3) = mod(X(1,3),2*pi);
end

function delta_steer = steerang( current_map_location, target_map_location, L )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%   delta_steer 順時為正，逆時針為負
[alpha, delta_location_map] = ppalpha(current_map_location,target_map_location);
temp1=2*L*sin(alpha);
temp2=norm(delta_location_map);
delta_steer = atan2(temp1,temp2);

end


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

end
