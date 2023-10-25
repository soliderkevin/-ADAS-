classdef HelperTrackingAndPlanningChasePlot < matlab.System
    properties
        Parent
        ViewLocation = [-4.7*3 0 10];
        ViewAngles = [0 20 0];
    end

    properties
        % [pos, posCov] = TrackParsingFcn(tracks)
        % [pos, posCov, vel] = TrackParsingFcn(tracks);
        % [pos, posCov, vel, labels] = TrackParsingFcn(tracks);
        TrackParsingFcn = @TrackParsingFcn;
        % pos = DetectionParsingFcn(detections)
        % [pos, posCov] = DetectionParsingFcn(detections);
        % [pos, posCov, vel] = DetectionParsingFcn(detections);
        DetectionParsingFcn = @DetectionParsingFcn;
    end

    properties (Access = protected)
        TheaterPlot
        RadarDetectionPlotter
        VisionDetectionPlotter
        TrackPlotter
        OutlinePlotter
        LaneBoundaryPlotter
        LaneMarkingPlotter
        ExistingObstacles
        EgoPredictedTrajectoryPlotters
    end

    methods
        function obj = HelperTrackingAndPlanningChasePlot(varargin)
            setProperties(obj,nargin,varargin{:});
        end

        function deleteLegend(obj)
            legend(obj.Parent,'off');
        end
    end

    methods (Access = protected)
        function setupImpl(obj , scenario, egoVehicle, sensors, detections, tracks)

            obj.TheaterPlot = theaterPlot('Parent',obj.Parent);
            obj.TrackPlotter = trackPlotter(obj.TheaterPlot,'DisplayName','Tracks','HistoryDepth',7);

            sensorClass = cellfun(@(x)class(x),sensors,'UniformOutput',false);

            isRadar = strcmpi(sensorClass,'drivingRadarDataGenerator');
            isCamera = strcmpi(sensorClass,'visionDetectionGenerator');

            if any(isRadar)
                obj.RadarDetectionPlotter = detectionPlotter(obj.TheaterPlot,'DisplayName','Radar Detections',...
                    'Marker','o','MarkerFaceColor','r');

            end

            if any(isCamera)
                obj.VisionDetectionPlotter = detectionPlotter(obj.TheaterPlot,'DisplayName','Vision Detections',...
                    'Marker','^','MarkerFaceColor','b');
            end
            chasePlot(egoVehicle,'Parent',obj.Parent,...
                'ViewLocation',obj.ViewLocation(1:2),...
                'ViewHeight',obj.ViewLocation(3),...
                'ViewYaw',obj.ViewAngles(1),...
                'ViewPitch',obj.ViewAngles(2),...
                'ViewRoll',obj.ViewAngles(3));

            % Find all actor patches
            allPatches = findall(obj.Parent,'Type','Patch');
            allTags = arrayfun(@(x)x.Tag,allPatches,'UniformOutput',false);
            actorPatches = allPatches(startsWith(allTags,'ActorPatch'));

            for i = 1:numel(actorPatches)
                actorPatches(i).FaceAlpha = 0.3;
                actorPatches(i).EdgeAlpha = 1;
%                 actorPatches(i).Ed
            end

            obj.ExistingObstacles = zeros(0,1);

            hold(obj.Parent,'on');
            egoTrajectoryPlotters = cell(4,1);
            trajColors = [0 1 1;0.5 0.5 0.5;1 0 0;0 1 0];
            width = [1 2 2 2];
            for i = 1:4
                egoTrajectoryPlotters{i} = plot(obj.Parent,nan,nan,'LineWidth',width(i),'Color',trajColors(i,:));
            end
            obj.EgoPredictedTrajectoryPlotters = egoTrajectoryPlotters;

            hold(obj.Parent,'off');
        end

        function stepImpl(obj, scenario, egoVehicle, sensors, detections, tracks, capList, trajectoryList)
            [pos, vel, labels] = obj.TrackParsingFcn(tracks, egoVehicle);
            plotTrack(obj.TrackPlotter, pos, vel, labels);

            detectionSensorIndex = cellfun(@(x)x.SensorIndex,detections);
            inSensorIndex = cellfun(@(x)x.SensorIndex,sensors);
            isRadar = cellfun(@(x)isa(x,'drivingRadarDataGenerator'),sensors);
            isCamera = cellfun(@(x)isa(x,'visionDetectionGenerator'),sensors);

            radarSensorIndex = inSensorIndex(isRadar);
            cameraSensorIndex = inSensorIndex(isCamera);

            pos = obj.DetectionParsingFcn(detections,egoVehicle);

            posRadar = pos(ismember(detectionSensorIndex,radarSensorIndex),:);
            posCamera = pos(ismember(detectionSensorIndex,cameraSensorIndex),:);

            plotDetection(obj.RadarDetectionPlotter,posRadar);
            plotDetection(obj.VisionDetectionPlotter,posCamera);
            hold(obj.Parent,'on');
            show(capList,'Parent',obj.Parent,'FastUpdate',1,'TimeStep',1:capList.MaxNumSteps);
            hold(obj.Parent,'off');
%             if ~isequal(obj.ExistingObstacles,capList.ObstacleIDs)
%                 allPatches = findall(obj.Parent,'Type','Patch');
%                 isCapsulePatch = arrayfun(@(x)isa(x.Parent,'matlab.graphics.primitive.Transform'),allPatches);
%                 capsulePatches = allPatches(isCapsulePatch);
%                 for i = 1:numel(capsulePatches)
%                     capsulePatches(i).FaceColor = 0.3*lines(1);
%                     capsulePatches(i).EdgeColor = 0.3*lines(1);
%                 end
%                 obj.ExistingObstacles = capList.ObstacleIDs;
%             end
            [trajPos, trajClass] = HelperTrackingAndPlanningChasePlot.parseTrajectoryForPlotting(trajectoryList);
    
            for i = 1:numel(obj.EgoPredictedTrajectoryPlotters)
                thisIdx = trajClass == i;
                thisX = trajPos(thisIdx,1);
                thisY = trajPos(thisIdx,2);
                obj.EgoPredictedTrajectoryPlotters{i}.XData = thisX;
                obj.EgoPredictedTrajectoryPlotters{i}.YData = thisY;
            end
        end
    end

    methods (Static)
        function [trajectoryPositions, class] = parseTrajectoryForPlotting(trajectoryList)

            trajectoryPositions = zeros(0,2);
            class = zeros(0,1);

            for i = 1:numel(trajectoryList)
                numStates = size(trajectoryList(i).Trajectory,1);
                trajectoryPositions = [trajectoryPositions;trajectoryList(i).Trajectory(:,1:2);nan nan]; %#ok<AGROW>
                if ~trajectoryList(i).IsValid
                    thisClass = 1;
                elseif isnan(trajectoryList(i).Evaluation)
                    thisClass = 2;
                elseif trajectoryList(i).Evaluation == 0
                    thisClass = 3;
                else
                    thisClass = 4;
                end
                class = [class;repmat(thisClass,numStates + 1,1)]; %#ok<AGROW>
            end

        end
    end
end

function [pos, vel, labels] = TrackParsingFcn(tracks, egoVehicle)
if isempty(tracks)
    pos = zeros(0,3);
    vel = zeros(0,3);
    labels = string.empty(0,1);
    return
end
state = horzcat(tracks.State);
cartState = filterToCartState(state);
pos = cartState([1 3],:);
vel = cartState([2 4],:);
labels = string([tracks.TrackID]);
pos = [pos;0.5*ones(1,numel(tracks))]';
vel = [vel;zeros(1,numel(tracks))]';
end

function pos =  DetectionParsingFcn(detections, egoVehicle)
sIdx = cellfun(@(x)x.SensorIndex, detections);
uqIdx = unique(sIdx);
pos = zeros(3,numel(detections));
for i = 1:numel(uqIdx)
    thisIdx = sIdx == uqIdx(i);
    pos(:,thisIdx) = calculatePositionInScenarioFrame(detections(thisIdx), egoVehicle);
end
pos = pos(1:2,:)';
pos = [pos 0.5*ones(1,numel(detections))'];
end


function [posScene, velScene, posCovEgo] = calculatePositionInScenarioFrame(detections, egoVehicle)

% Calculate Cartesian positions for all detections in the "sensor"
% coordinate frame
allDets = [detections{:}];
meas = horzcat(allDets.Measurement);

if strcmpi(allDets(1).MeasurementParameters(1).Frame,'Spherical')
    az = meas(1,:);
    r = meas(2,:);
    el = zeros(1,numel(az));
    [x, y, z] = sph2cart(deg2rad(az),deg2rad(el),r);
    posSensor = [x;y;z];
    rr = meas(3,:);
    rVec = posSensor./sqrt(dot(posSensor,posSensor,1));
    velSensor = rr.*rVec;
else
    posSensor = meas;
    velSensor = zeros(3,size(meas,2));
end

% Transform parameters
sensorToEgo = detections{1}.MeasurementParameters(1);
R = sensorToEgo.Orientation;
T = sensorToEgo.OriginPosition;
if isfield(sensorToEgo,'OriginVelocity')
    Tdot = sensorToEgo.OriginVelocity;
else
    Tdot = zeros(3,1);
end

if isfield(sensorToEgo,'IsParentToChild') && sensorToEgo.IsParentToChild
    R = R';
end

% Position, velocity in ego frame
posEgo = T + R*posSensor;
velEgo = Tdot + R*velSensor; % Assume Rdot = 0;

% egoToScenario
R2 = rotmat(quaternion([egoVehicle.Yaw egoVehicle.Pitch egoVehicle.Roll],'eulerd','ZYX','frame'),'point');
T = egoVehicle.Position(:);
Tdot = egoVehicle.Velocity(:);
posScene = T + R2*posEgo;
velScene = Tdot + R2*velEgo;

if nargout > 2
    assert(~strcmpi(allDets(1).MeasurementParameters(1).Frame,'Spherical'),'Only cartesian measurements');
    measCov = cat(3,allDets.MeasurementNoise);
    posCovEgo = zeros(3,3,numel(allDets));
    for i = 1:numel(allDets)
        posCovEgo(:,:,i) = R2*R*measCov(:,:,i)*R'*R2';
    end
end
end

function cartState = filterToCartState(filterState)

% Assemble as Frenet state to use frenet2global
numStates = size(filterState,2);
s = filterState(1,:);
ds = filterState(2,:);
dds = zeros(1,numStates);
d = filterState(3,:);
dd = filterState(4,:);
ddbyds = zeros(1,numStates);
ddbyds(abs(ds) > 0.5) = dd(abs(ds) > 0.5)./ds(abs(ds) > 0.5);
dd2ds2 = zeros(1,numStates);
frenetState = [s;ds;dds;d;ddbyds;dd2ds2];

% Convert to global state
refPath = helperGetReferencePath;
globalState = frenet2global(refPath,frenetState')';

% Convert to cartesian state
x = globalState(1,:);
y = globalState(2,:);
speed = globalState(5,:);
theta = globalState(3,:);
vx = speed.*cos(theta);
vy = speed.*sin(theta);
cartState = [x;vx;y;vy];

end