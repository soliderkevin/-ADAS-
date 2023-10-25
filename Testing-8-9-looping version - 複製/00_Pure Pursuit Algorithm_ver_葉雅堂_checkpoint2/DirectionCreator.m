clear 
close 
clc

waypoints = [0 0; ...
 50 20; ...
 100 0; ...
 150 10];
refPath = referencePathFrenet(waypoints);
connector = trajectoryGeneratorFrenet(refPath);
initState = [0 0 0 0 0 0]; % [S ds ddS L dL ddL]
termState = [30 0 0 0 0 0]; % [S ds ddS L dL ddL]
[~,trajGlobal] = connect(connector,initState,termState,5);
show(refPath);
hold on
axis equal
plot(trajGlobal.Trajectory(:,1),trajGlobal.Trajectory(:,2),'b')
legend(["Waypoints","Reference Path","Trajectory to 30m"])

termStateDeviated = termState + ([-3:3]' * [0 0 0 1 0 0]);
[~,trajGlobal] = connect(connector,initState,termStateDeviated,10);

clf
show(refPath);
hold on
axis equal
for i = 1:length(trajGlobal)
 plot(trajGlobal(i).Trajectory(:,1),trajGlobal(i).Trajectory(:,2),'g')
end
legend(["Waypoints","Reference Path","Alternative Trajectories"])
hold off

newTermState = [5 10 0 5 0 0];
[~,newTrajGlobal] = connect(connector,initState,newTermState,3);



