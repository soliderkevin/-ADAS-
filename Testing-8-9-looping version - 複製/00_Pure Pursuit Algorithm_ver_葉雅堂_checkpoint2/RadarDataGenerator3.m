clear
clc
close all
 
X_map = getloopmap();
 

simTime=0.1;


%demo of curvative file
load Monza.mat %x,y 
TX = [X Y];
[L2,R2,K2] = curvature(TX);
Z=0;


scenario = drivingScenario('SampleTime',simTime);
%%
%Monza X,Y and Z(direction angle)
roadCenters = [X_map(:,1), X_map(:,2), X_map(:,5)] ;
% roadRights = [X_map(:,1), X_map(:,2), X_map(:,5];
% %%
%Lanemark
lm = [laneMarking('Solid','Color','w'); ...
    laneMarking('Dashed','Color','y'); ...
    laneMarking('Dashed','Color','y'); ...
    laneMarking('Solid','Color','w')];
ls = lanespec(3,'Marking',lm);    %3 Lanes Generated
road(scenario,roadCenters,'Lanes',ls); %ls 三線道把標線標上去

%%
%simulating Car本車狀態
ego_car = vehicle(scenario, ...   
        'ClassID',1, ... %轎車ID=1
        'Position',[X_map(1,1), X_map(1,2), X_map(1,5)]); %X,Y,Z的位置
ego_car.Yaw = X_map(1,3)*180/pi; %Yaw就是heading angle，在Matlab吃角度不是rad
 
%%
%simulating Car對手車狀態
car_1 = vehicle(scenario,'ClassID',1,'Position',[ego_car.Position(1)+10 200 0]);  % +10 -1.8
car_1_Waypoints = roadCenters(20:end,:);
      carSpeed = 20;

smoothTrajectory(car_1,car_1_Waypoints,carSpeed)
 
 %%
 %Front Mirror為前車雷達
maxRange = 100; % 
frontMirror = [ego_car.FrontOverhang 0 (ego_car.Height-0.1)]; %這行不用動
 
% Register actor profiles with the sensors.
profiles = actorProfiles(scenario);   %腳本的演員是誰?  不用動
targetProfiles = profiles(2:end);     %車子            不用動
 
update_rate = 1/scenario.SampleTime;  %更新速率        不用動
id = 1;                               %雷達在哪台車身上?
rdr_front = drivingRadarDataGenerator(id,'UpdateRate',update_rate, ... %現在用一隻雷達、更新率?
 'MountingLocation',frontMirror, ...                                   %雷達位置? 
 'RangeLimits',[0 maxRange], ...                                       %雷達範圍? 
 'Profiles',targetProfiles);                                           %偵測目標? 
 
%%
%鳥看圖設定伏試圖
bep = birdsEyePlot('XLim',[-20 50],'YLim',[-35 35]);
 
lmPlotter = laneMarkingPlotter(bep,'Tag','lm','DisplayName','Lane markings');
olPlotter = outlinePlotter(bep,'Tag','ol');
caPlotter = coverageAreaPlotter(bep, ...
 'Tag','ca', ...
 'DisplayName','Radar coverage area', ...
 'FaceColor','red','EdgeColor','red');
 
helperPlotScenario(bep,rdr_front,ego_car)

%%
 
chasePlot(ego_car)  %劃一個方塊圖

%%
%畫邊線，左邊和右邊
olPlotter = outlinePlotter(bep); %查
lblPlotter = laneBoundaryPlotter(bep,'Color','r','LineStyle','-');
lbrPlotter = laneBoundaryPlotter(bep,'Color','g','LineStyle','-');
rbsEdgePlotter = laneBoundaryPlotter(bep);
 %%
 
%% Generate Clustered Detections
% Use the radar to generate clustered detections of the target vehicles.
% Visualize these detections on the bird's-eye plot. At each simulation
% time step, the radar generates only one detection per target.

%雷達打到的車子中心聚集點
clusterDetPlotter = detectionPlotter(bep, ...
 'DisplayName','Clustered detections', ...    %將聚集點畫出來
 'MarkerEdgeColor','red', ...                 %使用紅色來畫
 'MarkerFaceColor','red');                   %使用紅色來畫

%global variable
L=4.7;
velocity = 0;

%User global variable
rdr_sample_time=0.001; %可以換SAMPLE_TIME
Car_Velocity = 30;  %- ego dynamic
map_loop_amount = 3; %looping 3 times depends on map_looping
givenpath=X_map; %做自己車子的
phy = 0;   %路面得高度
iy = 1;    %路面的摩嚓係數
g = 9.8;   %重力加速度

%inner global default
count= 0;     % waiting count
count2 = 0;    % Boosting time count
Switching_Lane_Index = 0;
given_i = 1;
map_looping = 0; %inner looping default 0
switchinglane_index = 0;
curve_speed = 0;
LOOP = 1;
curve_velocity = 0;

while (map_looping < map_loop_amount)
restart(scenario); % The instruction code inside advance driving model
while advance(scenario)
 
 %% sensor data



 
  
 targets = targetPoses(ego_car);
 [dets, numDets, isValidTime] = rdr_front(targets, rdr_sample_time); %rdr_front 就是drivingRadarDataGenerator的一個，輸出Dectector動dets(偵測到的數量),num和validtime(是否掛掉)
 [T1,T2] = helperPlotScenario(bep, rdr_front, ego_car)                         %協助畫圖，用 = [a,b] 得出位置
 if isValidTime && numDets > 0
 
 detPos = cell2mat(cellfun(@(d)d.Measurement(1:2),dets, ...
 'UniformOutput',false)')';
 plotDetection(clusterDetPlotter,detPos);
 
 end 
 


 
 %% ego dynamic
 %車輛自己追機的動態  %車子速度
 a = 4.3;   %剎車和加速 depending on the country, japan 4.6, taiwan 4.3 or so.
 t_rule = 3;        %安全距離 3 秒原則   
 Waiting_Time = 4; % seconds
 BackTrack_Time = 4;
 
 Rule_Distances = Car_Velocity*(t_rule) - (1/2)*a*(t_rule)^2;  %安全距離 3 秒原則 希望保留空間
 
 D_vector=T1(1,:)-T1(2,:);  % radar distance
 D_ForR =  norm(D_vector,2) - Rule_Distances ; %誤差希望達到距離去改變velocity值 
 

 display (norm(D_vector,2));
 display (D_ForR);

 
  curve_velocity = sqrt((phy+iy)/(1-phy*iy)*g*R2(LOOP)); %The maximum speed from curvative formula
  display(curve_velocity);

  if R2(LOOP) < 1000 % Checker determine curvative < 1000 as the curve.
      curve_index = 1;
      curve_speed = curve_velocity;
  else
      curve_index = 0;
  end
             %following the forward car track

  if(curve_index == 0)
      if(velocity <100) %the curvative maximum speed
        if norm(D_vector,2) > Car_Velocity*3
         velocity = Car_Velocity;

        else
         velocity=sqrt(abs(Car_Velocity*Car_Velocity + 2*a*D_ForR)); % v^2 =  v0^2 +2*a*s -> v = sqrt(v0^2 +2*a*s); abs function to make it positive.
%          Waypoint可以拉多遠  -新成的軌跡上，stash-feedback，     
%          display("Inside");
%          display(D_ForR);
%          display(2*a*D_ForR);
%          display(Car_Velocity*Car_Velocity);
%          display(Car_Velocity*Car_Velocity + 2*a*D_ForR);

         end 
        
      else

      end
  else
       switchinglane_index = 0;
      

  end


  
  %ego car 
  if R2(LOOP)<1000 %heading point(位置)
     velocity = curve_velocity;
  end



display(velocity);

 
%When car in curve and radar couldn't hit the car, it goes on the same
%speed.


 display(R2(LOOP));
 Z = R2(LOOP);
 display(Z);

 display(carSpeed);

 temp1=ego_car.Position(1,1:2);
 temp2=deg2rad(ego_car.Yaw);
 ego_now=[temp1,temp2];
 ego_next = getnextegostate(velocity, ego_now, givenpath,simTime);
 helperMoveEgoToState(ego_car, ego_next, velocity);
 
 LOOP = LOOP+1;
 end
map_looping = map_looping + 1;
end
 
 
 
 
 
function [position,yaw] = helperPlotScenario(bep,radar,ego)
 
 % Plot lane markings
 lmPlotter = findPlotter(bep,'Tag','lm');
 [lmv,lmf] = laneMarkingVertices(ego);
 plotLaneMarking(lmPlotter,lmv,lmf)
 
 % Plot vehicle outlines
 olPlotter = findPlotter(bep,'Tag','ol');
 [position,yaw,length,width,originOffset,color] = targetOutlines(ego);
 plotOutline(olPlotter,position,yaw,length,width, ...
 'OriginOffset',originOffset,'Color',color)
 
 % Plot radar coverage area
 caPlotter = findPlotter(bep,'Tag','ca');
 plotCoverageArea(caPlotter,radar.MountingLocation(1:2), ...
 radar.RangeLimits(2),radar.MountingAngles(1), ...
 radar.FieldOfView(1))
 
end
 
 
function ego_next = getnextegostate(velocity, ego_now, givenpath,DT,R2) %動態方程式經過此函數會變成matlab裡的腳本
 
 L=4.7;
 
 [heading_point] = lookahead4(velocity, ego_now, givenpath,DT,R2); % 決定預視點 heading point [x_map,y_map]^T
 
 delta_steer = steerang( ego_now, heading_point, L ); % 算輪胎轉角 delta
 
 ego_next = vehicle_dynamic(ego_now,delta_steer,velocity,DT); % 更新車輛動態 X_now := [x,y, heading]^T
 
 


 
 % velocity = R(i); %Now R location my car
 
 %sqrt((phy iy)/(1-phy*iy)*g*R2)

 

 %carspeed = 100; %Now R location ego car
end
 
 
function X = vehicle_dynamic(X_now,delta,velocity,DT)
 L=4.5;
 X(1,1) = X_now(1,1)+velocity*cos(X_now(1,3))*DT; % car_in_map _x
 X(1,2) = X_now(1,2)+velocity*sin(X_now(1,3))*DT; % car_in_map_y
 X(1,3) = X_now(1,3)+velocity/L*(tan(delta))*DT; % vehicle currentlt heading angle in map
 X(1,3) = mod(X(1,3),2*pi);
end

function [L,R,k] = curvature(X)
% Radius of curvature and curvature vector for 2D or 3D curve
%  [L,R,k] = curvature(X)
%   X:   2 or 3 column array of x, y (and possibly z) coordiates
%   L:   Cumulative arc length
%   R:   Radius of curvature
%   k:   Curvature vector
% The scalar curvature value is 1./R
% Version 2.6: Calculates end point values for closed curve

  N = size(X,1);
  dims = size(X,2);
  if dims == 2
    X = [X,zeros(N,1)];  % Use 3D expressions for 2D as well
  end
  L = zeros(N,1);
  R = NaN(N,1);
  k = NaN(N,3);
  for i = 2:N-1
    [R(i),~,k(i,:)] = circumcenter(X(i,:)',X(i-1,:)',X(i+1,:)');
    L(i) = L(i-1)+norm(X(i,:)-X(i-1,:));
  end
  if norm(X(1,:)-X(end,:)) < 1e-10 % Closed curve. 
    [R(1),~,k(1,:)] = circumcenter(X(end-1,:)',X(1,:)',X(2,:)');
    R(end) = R(1);
    k(end,:) = k(1,:);
    L(end) = L(end-1) + norm(X(end,:)-X(end-1,:));
  end
  i = N;
  L(i) = L(i-1)+norm(X(i,:)-X(i-1,:));
  if dims == 2
    k = k(:,1:2);
  end
end


 
function helperMoveEgoToState(ego_car, ego_next,velocity)
ego_car.Position(1:2) = ego_next(1:2);
ego_car.Velocity(1:2) = [cos(ego_next(3)) sin(ego_next(3))]*velocity;
ego_car.Yaw = rad2deg(ego_next(3));
end