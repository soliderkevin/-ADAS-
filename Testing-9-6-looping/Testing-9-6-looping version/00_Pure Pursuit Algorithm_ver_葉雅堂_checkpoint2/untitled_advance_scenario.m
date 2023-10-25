clear
close all
clc
X_map = getloopmap();

DT=0.1;
scenario = drivingScenario('SampleTime',DT,'StopTime',500);
roadcenters = [X_map(:,1), X_map(:,2), X_map(:,5)];
lm = [laneMarking('Solid','Color','w'); ...
    laneMarking('Dashed','Color','y'); ...
    laneMarking('Dashed','Color','y'); ...
    laneMarking('Solid','Color','w')];
ls = lanespec(3,'Marking',lm);
road(scenario,roadcenters,'Lanes',ls);

ego_car = vehicle(scenario, ...
    'ClassID',1, ...
    'Position',[X_map(1,1), X_map(1,2), X_map(1,5)]);
ego_car.Yaw = X_map(1,3)*180/pi;

% plot(scenario)
chasePlot(ego_car)
bep = birdsEyePlot('XLim',[-15 35],'YLim',[-15 15]);
olPlotter = outlinePlotter(bep);
lblPlotter = laneBoundaryPlotter(bep,'Color','r','LineStyle','-');
lbrPlotter = laneBoundaryPlotter(bep,'Color','g','LineStyle','-');
rbsEdgePlotter = laneBoundaryPlotter(bep);
legend('off');
givenpath=X_map;
L=4.7;

while advance(scenario)
    
    velocity=15;
    DT=0.1;
    temp1=ego_car.Position(1,1:2);
    temp2=deg2rad(ego_car.Yaw);
    ego_now=[temp1,temp2];
    rbs = roadBoundaries(ego_car);
    [position,yaw,length,width,originOffset,color] = targetOutlines(ego_car);  % 車座標俯視圖參數
    lb = laneBoundaries(ego_car,'XDistance',0:5:30,'LocationType','Center', ...
        'AllBoundaries',false);
    plotLaneBoundary(rbsEdgePlotter,rbs)
    plotLaneBoundary(lblPlotter,{lb(1).Coordinates})
    plotLaneBoundary(lbrPlotter,{lb(2).Coordinates})
    plotOutline(olPlotter,position,yaw,length,width, ...
        'OriginOffset',originOffset,'Color',color)   % 車座標俯視圖作圖
    ego_next = getnextegostate(velocity, ego_now, givenpath,DT);
    helperMoveEgoToState(ego_car, ego_next,velocity);
    pause(0.001)
      
end


function ego_next = getnextegostate(velocity, ego_now, givenpath,DT)

    L=4.7;
    [heading_point] = lookahead4(velocity, ego_now, givenpath,DT);            % 決定預視點 heading point [x_map,y_map]^T
    delta_steer = steerang( ego_now, heading_point, L );                      % 算輪胎轉角 delta
    ego_next = vehicle_dynamic(ego_now,delta_steer,velocity,DT);              % 更新車輛動態 X_now := [x,y, heading]^T
    
end



function X = vehicle_dynamic(X_now,delta,velocity,DT)
  L=4.5;
  X(1,1) = X_now(1,1)+velocity*cos(X_now(1,3))*DT; % car_in_map _x
  X(1,2) = X_now(1,2)+velocity*sin(X_now(1,3))*DT; % car_in_map_y
  X(1,3) = X_now(1,3)+velocity/L*(tan(delta))*DT;  % vehicle currentlt heading angle in map
  X(1,3) = mod(X(1,3),2*pi);
end


function helperMoveEgoToState(ego_car, ego_next,velocity)
ego_car.Position(1:2) = ego_next(1:2);
ego_car.Velocity(1:2) = [cos(ego_next(3)) sin(ego_next(3))]*velocity;
ego_car.Yaw = rad2deg(ego_next(3));
end

