
%Required navigation toolbox for the connect command
clear
clc
close all
waypoints = [0 0; ...
 50 20; ...
 100 0; ...
 150 60];
refPath = referencePathFrenet(waypoints);
connector = trajectoryGeneratorFrenet(refPath);
 
 
 
initState = [0 0 0 0 0 0]; % [S ds ddS L dL ddL] trajglobal縱向位置；加速度、跳度| 路徑長、車向加速度、車向跳度
termState = [30 0 0 0 0 0]; % [S ds ddS L dL ddL]
 
termStateDeviated = termState + ([-3:3]' * [0 0 0 1 0 0]);
[~,trajGlobal] = connect(connector,initState,termStateDeviated,10); % 8 是t時間，connect指令
 
clf
show(refPath);
hold on
axis equal
for i = 1:length(trajGlobal)
 plot(trajGlobal(i).Trajectory(:,1),trajGlobal(i).Trajectory(:,2),'g')
end
legend(["Waypoints","Reference Path","Alternative Trajectories"])
hold off

%trajGlobal存入: x位置、Y位置、縱向加速度、側向加速度、縱向跳度、側向跳度。是給地圖的座標。去看connect輸出的資料結構
%(x(2)-x(1))/(sampling time)
%打到後有障礙物不能走，給定旁邊reference的距離。根據referencePathFrenet(waypoints); connector = trajectoryGeneratorFrenet(refPath);
%這三個，能生出一跳壁障用軌跡，切到trajGlobal就可以完成避障

