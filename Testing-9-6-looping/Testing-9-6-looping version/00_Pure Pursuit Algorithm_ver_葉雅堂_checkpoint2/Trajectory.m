
%Required navigation toolbox for the connect command
function  [x,y]= Trajectory(x_position, y_position)

% waypoints = [0 0; ... 2*n matrixs  read the x,y of racetrack
%  50 20; ...
%  100 0; ...
%  150 60];  
waypoints = [x_position,y_position] %map X,Y coordination
refPath = referencePathFrenet(waypoints);
%%
% 
% * ITEM1
% 
% * ITEM1
% * ITEM2
% 
% * ITEM2
% 
connector = trajectoryGeneratorFrenet(refPath); %Expected waypoints rows to be an array with all of the values > 1.
 
 
 
initState = [0 0 0 0 0 0]; % [S ds ddS L dL ddL] trajglobal縱向位置；加速度、跳度| 路徑長、車向加速度、車向跳度
termState = [1 0 0 0 0 0]; % [S ds ddS L dL ddL] %Generate a five-second trajectory between the path origin and a point 30 m down the path as Frenet states.
  
termStateDeviated = termState + ([-1:1]' * [0 0 0 1 0 0]); %generating from -1~1 between dots.
[~,trajGlobal] = connect(connector,initState,termStateDeviated,6); % 8 是t時間，connect指令 ，Singular start pose with singular goal pose.% Multiple start pose with singular goal pose.% % Singular start pose with multiple goal pose.% % Multiple start pose with multiple goal pose.
 
% clf
% show(refPath);
% hold on
% axis equal
% for i = 1:length(trajGlobal)
%  plot(trajGlobal(i).Trajectory(:,1),trajGlobal(i).Trajectory(:,2),'g') % plot out trajGlobal(1)andtrajGlobal(3) also making trajGlobal(2) unable to  , Trajectory(:,1), Trajectory(:,2)
%  end


%Return the waypoints and insert into the original map for determination

x = trajGlobal(i).Trajectory(:,1);
y =  trajGlobal(i).Trajectory(:,2);
end
% 
% path = Trajectory_1(x_position, y_position);
%trajGlobal存入: x位置、Y位置、縱向加速度、側向加速度、縱向跳度、側向跳度。是給地圖的座標。去看connect輸出的資料結構
%(x(2)-x(1))/(sampling time)
%打到後有障礙物不能走，給定旁邊reference的距離。根據referencePathFrenet(waypoints); connector = trajectoryGeneratorFrenet(refPath);
%這三個，能生出一跳壁障用軌跡，切到trajGlobal就可以完成避障

%因跑道圖資的間隔都差不多是一米 這個是生成路徑的提示 給你下起點終點用
% PS資料每相鄰兩點約一米長