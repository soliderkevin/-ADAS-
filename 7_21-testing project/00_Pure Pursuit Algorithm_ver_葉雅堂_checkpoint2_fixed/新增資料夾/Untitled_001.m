clear
close all
clc
% Setup scenario and sensors

% [scenario, egoVehicle, sensors] = helperTrackingAndPlanningScenario();

[scenario, egoVehicle, sensors] = TrackingPlanningScenario();

tracker = trackerJPDA('FilterInitializationFcn',@helperInitRefPathFilter,...
    'AssignmentThreshold',[200 inf],...
    'ConfirmationThreshold',[8 10],...
    'DeletionThreshold',[5 5]);

% Collision check time stamps
tHorizon = 5; % seconds
deltaT = 0.5; % seconds
tSteps = deltaT:deltaT:tHorizon;

% Create the dynamicCapsuleList object for AEB
capList = dynamicCapsuleList;
capList.MaxNumSteps = numel(tSteps) + 1;

% Specify the ego vehicle geometry
carLen = 4.7;
carWidth = 1.8;
rearAxleRatio = 0.25;
egoID = 1;
[egoID, egoGeom] = egoGeometry(capList,egoID); % Geometric properties for set of ego bodies

% Inflate to allow uncertainty and safety gaps
egoGeom.Geometry.Length = 2*carLen; % in meters
egoGeom.Geometry.Radius = carWidth/2; % in meters
egoGeom.Geometry.FixedTransform(1,end) = -2*carLen*rearAxleRatio; % in meters
updateEgoGeometry(capList,egoID,egoGeom);

% Create display for visualizing results
display = HelperTrackingAndPlanningDisplay;

% Initial state of the ego vehicle
refPath = helperGetReferencePath;
egoState = frenet2global(refPath,[0 0 0 0.5*3.6 0 0]);
helperMoveEgoToState(egoVehicle, egoState);

while advance(scenario)

    % Current time
    time = scenario.SimulationTime;

    % Step 1. Collect data
    detections = helperGenerateDetections(sensors, egoVehicle, time);
    
    % Step 2. Feed detections to tracker
    tracks = tracker(detections, time);

    % Step 3. Predict tracks in planning horizon
    timesteps = time + tSteps;
    predictedTracks = repmat(tracks,[1 numel(timesteps)+1]);
    for i = 1:numel(timesteps)
        predictedTracks(:,i+1) = predictTracksToTime(tracker,'confirmed',timesteps(i));
    end
    
    % Step 4. Update capsule list and plan highway trajectory
    currActorState = helperUpdateCapsuleList(capList, predictedTracks);
    [optimalTrajectory, trajectoryList] = helperPlanHighwayTrajectory(capList, currActorState, egoState);

    % Visualize the results
    display(scenario, egoVehicle, sensors, detections, tracks, capList, trajectoryList);

    % Step 5. Move ego on planned trajectory
    egoState = optimalTrajectory(2,:);
    helperMoveEgoToState(egoVehicle,egoState);

end

showSnaps(display,2,4); % Shows snapshot while publishing

showSnaps(display,2,1); % Shows snapshot while publishing

showSnaps(display,2,2); % Shows snapshot while publishing

showSnaps(display,2,3); % Show snapshot while publishing


function detections = helperGenerateDetections(sensors, egoVehicle, time)
    detections = cell(0,1);
    for i = 1:numel(sensors)
        thisDetections = sensors{i}(targetPoses(egoVehicle),time);
        detections = [detections;thisDetections]; %#ok<AGROW> 
    end

    detections = helperAddEgoVehicleLocalization(detections,egoVehicle);
    detections = helperPreprocessDetections(detections);
end

function detectionsOut = helperAddEgoVehicleLocalization(detectionsIn, egoPose)

defaultParams = struct('Frame','Rectangular',...
    'OriginPosition',zeros(3,1),...
    'OriginVelocity',zeros(3,1),...
    'Orientation',eye(3),...
    'HasAzimuth',false,...
    'HasElevation',false,...
    'HasRange',false,...
    'HasVelocity',false);

fNames = fieldnames(defaultParams);

detectionsOut = cell(numel(detectionsIn),1);

for i = 1:numel(detectionsIn)
    thisDet = detectionsIn{i};
    if iscell(thisDet.MeasurementParameters)
        measParams = thisDet.MeasurementParameters{1};
    else
        measParams = thisDet.MeasurementParameters(1);
    end

    newParams = struct;
    for k = 1:numel(fNames)
        if isfield(measParams,fNames{k})
            newParams.(fNames{k}) = measParams.(fNames{k});
        else
            newParams.(fNames{k}) = defaultParams.(fNames{k});
        end
    end

    % Add parameters for ego vehicle
    thisDet.MeasurementParameters = [newParams;newParams];
    thisDet.MeasurementParameters(2).Frame = 'Rectangular';
    thisDet.MeasurementParameters(2).OriginPosition = egoPose.Position(:);
    thisDet.MeasurementParameters(2).OriginVelocity = egoPose.Velocity(:);
    thisDet.MeasurementParameters(2).Orientation = rotmat(quaternion([egoPose.Yaw egoPose.Pitch egoPose.Roll],'eulerd','ZYX','frame'),'frame')';
    
    
    % No information from object class and attributes
    thisDet.ObjectClassID = 0;
    thisDet.ObjectAttributes = struct;
    detectionsOut{i} = thisDet;
end

end

function detections = helperPreprocessDetections(detections)
    % This function pre-process the detections from radars and cameras to
    % fit the modeling assumptions used by the tracker

    % 1. It removes velocity information from camera detections. This is
    % because those are filtered estimates and the assumptions from camera
    % may not align with defined prior information for tracker.
    %
    % 2. It fixes the bias for camera sensors that arise due to camera
    % projections for cars just left or right to the ego vehicle.
    % 
    % 3. It inflates the measurement noise for range-rate reported by the
    % radars to match the range-rate resolution of the sensor
    for i = 1:numel(detections)
        if detections{i}.SensorIndex > 1 % Camera
            % Remove velocity
            detections{i}.Measurement = detections{i}.Measurement(1:3);
            detections{i}.MeasurementNoise = blkdiag(detections{i}.MeasurementNoise(1:2,1:2),25);
            detections{i}.MeasurementParameters(1).HasVelocity = false;

            % Fix bias
            pos = detections{i}.Measurement(1:2);
            if abs(pos(1)) < 5 && abs(pos(2)) < 5
                [az, ~, r] = cart2sph(pos(1),pos(2),0);
                [pos(1),pos(2)] = sph2cart(az, 0, r + 0.7); % Increase range
                detections{i}.Measurement(1:2) = pos;
                detections{i}.MeasurementNoise(2,2) = 0.25;
            end
        else % Radars
            detections{i}.MeasurementNoise(3,3) = 0.5^2/4;
        end
    end
end


function helperMoveEgoToState(egoVehicle, egoState)
egoVehicle.Position(1:2) = egoState(1:2);
egoVehicle.Velocity(1:2) = [cos(egoState(3)) sin(egoState(3))]*egoState(5);
egoVehicle.Yaw = egoState(3)*180/pi;
egoVehicle.AngularVelocity(3) = 180/pi*egoState(4)*egoState(5);
end





