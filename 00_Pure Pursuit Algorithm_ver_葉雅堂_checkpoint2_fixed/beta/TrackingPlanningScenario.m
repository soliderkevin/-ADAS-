function [scenario, egoVehicle, sensors] = TrackingPlanningScenario

% createDrivingScenario Returns the drivingScenario defined in the Designer

% Generated by MATLAB(R) 9.9 (R2020b) and Automated Driving Toolbox 3.2 (R2020b).
% Generated on: 28-Apr-2020 15:04:44

% Construct a drivingScenario object.
scenario = drivingScenario;
scenario.SampleTime = 0.1;

% Add all road segments
roadCenters = [0 50 0;
    150 50 0;
    300 75 0;
    310 75 0;
    400 0 0;
    300 -50 0;
    290 -50 0;
    0 -50 0;
    0 -50 0];
laneSpecification = lanespec(4);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [10.7 51.4 0], ...
    'Name', 'Car');


% Add the non-ego actors
car1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [34.7 49.3 0], ...
    'Name', 'Car1');
waypoints = [34.7 49.3 0;
    60.1 48.2 0;
    84.2 47.9 0;
    119 49.3 0;
    148.1 51.4 0;
    189.6 58.7 0;
    230.6 68 0;
    272.6 74.7 0;
    301.4 77.5 0;
    316.7 76.8 0;
    332.4 75.2 0;
    348.9 72.2 0;
    366.2 65.1 0;
    379.6 55.6 0];
speed = [10;10;10;10;10;10;10;10;10;10;10;10;10;10];
trajectory(car1, waypoints, speed);

car2 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [17.6 46.7 0], ...
    'Name', 'Car2');
waypoints = [17.6 46.7 0;
    43.4 45.5 0;
    71.3 43.8 0;
    102.7 44.2 0;
    123.5 45.5 0;
    143.6 47.4 0;
    162.4 50 0;
    198.5 61 0;
    241.1 70.1 0;
    272.3 74.1 0;
    292 76.6 0;
    312.8 77.2 0;
    350.3 75.2 0;
    362.5 70.4 0;
    375.9 63.3 0;
    390.7 49.9 0;
    401.3 33 0];
speed = [9;9;9;9;9;9;9;9;9;9;9;9;9;9;9;9;9];
trajectory(car2, waypoints, speed);

car3 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [62.6 51.9 0], ...
    'Name', 'Car3');
waypoints = [62.6 51.9 0;
    87.4 51.3 0;
    117.7 52.2 0;
    147.6 55 0;
    174.9 59.7 0;
    203.3 65.8 0;
    265 69.7 0;
    288.3 73.1 0;
    314.5 73.1 0;
    334.9 70.8 0;
    360 59.9 0];
speed = [6;6;6;6;6;6;6;6;6;6;6];
trajectory(car3, waypoints, speed);

car4 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [101.7 41.1 0], ...
    'Name', 'Car4');
waypoints = [101.7 41.1 0;
    124.6 42 0;
    148.5 43.9 0;
    171.9 48.2 0;
    197.1 52.8 0;
    222.3 58.5 0;
    252.4 64.4 0;
    281.4 68.5 0;
    307.7 69.5 0;
    329.9 68.2 0;
    352.7 62.8 0];
speed = [7;7;7;7;7;7;7;7;7;7;7];
trajectory(car4, waypoints, speed);

car5 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [251.3 75.6 0], ...
    'Name', 'Car5');
waypoints = [251.3 75.6 0;
    255.7 76.7 0];
speed = [0.01;0.01];
trajectory(car5, waypoints, speed);

sensors = createSensors(scenario);

end

function sensors = createSensors(scenario)

radarPosition = [3.7 0 0.2];
radarDetectionRanges = [1 160];
radarFieldOfView = [48 5];
radarRotation = [0 0 0];

cameraPositions = [2.95 0 1.1;...
                   2.0 0.9 0.7;...
                   2.8 0.9 0.7;...
                   2.0 -0.9 0.7;...
                   2.8 -0.9 0.7];

cameraRotation = [0 1 0;0 1 65;0 1 140;0 1 -65;0 1 -140];

cameraRanges = [250 80 100 80 100];

cameraImageSize = [600 800;800 1024;720 1280;800 1024;720 1280];

cameraPrincipalPoint = [400 300;512 400;640 360;512 400;640 360];

cameraFocalLength = [900 900;400 400;720 720;400 400;720 720];

numCameras = size(cameraPositions,1);
cameras = cell(numCameras,1);
numRadars = size(radarPosition,1);
radars = cell(numRadars,1);

profiles = actorProfiles(scenario);
visionProfiles = profiles;
for i = 1:numel(visionProfiles)
    visionProfiles(i).Length = 1;
    visionProfiles(i).OriginOffset = [0.5 0 0];
end

for i = 1:numRadars
    radars{i} = drivingRadarDataGenerator('SensorIndex',i);
    radars{i}.FieldOfView = radarFieldOfView(i,:);
    radars{i}.MountingLocation = radarPosition(i,:);
    radars{i}.MountingAngles = radarRotation(i,:);
    radars{i}.RangeLimits = radarDetectionRanges(i,:);
    radars{i}.Profiles = profiles;
    radars{i}.HasRangeRate = true;
    radars{i}.DetectionCoordinates = 'Sensor spherical';
    radars{i}.HasFalseAlarms = true;
    radars{i}.FalseAlarmRate = 1e-7;
    radars{i}.HasNoise = true;
end

for i=1:numCameras
    cameras{i} = visionDetectionGenerator('SensorIndex',numRadars + i);
    intrinsics = cameraIntrinsics(cameraFocalLength(i,:),cameraPrincipalPoint(i,:),cameraImageSize(i,:));
    cameras{i}.Intrinsics = intrinsics;
    cameras{i}.SensorLocation = cameraPositions(i,1:2);
    cameras{i}.Height = cameraPositions(i,3);
    cameras{i}.Roll = cameraRotation(i,1);
    cameras{i}.Pitch = cameraRotation(i,2);
    cameras{i}.Yaw = cameraRotation(i,3);
    cameras{i}.ActorProfiles = visionProfiles;
    cameras{i}.MaxRange = cameraRanges(i);
    cameras{i}.DetectionCoordinates = 'Ego Cartesian';
    cameras{i}.MinObjectImageSize = [1 1];
    cameras{i}.HasNoise = true;
end

sensors = [radars;cameras];

end