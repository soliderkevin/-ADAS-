classdef HelperTrackingAndPlanningBirdsEyePlot < matlab.System
    properties
        Parent
        XLimits = [-40 80];
        YLimits = [-30 30];
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
        BirdsEyePlot
        RadarDetectionPlotter
        VisionDetectionPlotter
        TrackPlotter
        OutlinePlotter
        LaneBoundaryPlotter
        LaneMarkingPlotter
        PredictedTrajectoryPlotter;
        EgoPredictedTrajectoryPlotters
    end

    methods
        function obj = HelperTrackingAndPlanningBirdsEyePlot(varargin)
            setProperties(obj,nargin,varargin{:});
        end
        function deleteLegend(obj)
            legend(obj.Parent,'off');
        end
    end

    methods (Access = protected)
        function setupImpl(obj , scenario, egoVehicle, sensors, detections, tracks, capList, trajectoryList)
            obj.BirdsEyePlot = birdsEyePlot('Parent',obj.Parent,...
                'XLim',obj.XLimits,...
                'YLim',obj.YLimits);
            obj.TrackPlotter = trackPlotter(obj.BirdsEyePlot,'DisplayName','Tracks','HistoryDepth',7);

            sensorClass = cellfun(@(x)class(x),sensors,'UniformOutput',false);

            isRadar = strcmpi(sensorClass,'drivingRadarDataGenerator');
            isCamera = strcmpi(sensorClass,'visionDetectionGenerator');

            if any(isRadar)
                obj.RadarDetectionPlotter = detectionPlotter(obj.BirdsEyePlot,'DisplayName','Radar Detections',...
                    'Marker','o','MarkerFaceColor','r');

            end

            if any(isCamera)
                obj.VisionDetectionPlotter = detectionPlotter(obj.BirdsEyePlot,'DisplayName','Vision Detections',...
                    'Marker','^','MarkerFaceColor','b');
            end

            radars = sensors(isRadar);
            cameras = sensors(isCamera);

%             for i = 1:numel(radars)
%                 plotter = coverageAreaPlotter(obj.BirdsEyePlot,'FaceColor','r');
%                 plotCoverageArea(plotter,radars{i}.MountingLocation(1:2),radars{i}.RangeLimits(2),radars{i}.MountingAngles(3),radars{i}.FieldOfView(1));
%             end
% 
%             for i = 1:numel(cameras)
%                 plotter = coverageAreaPlotter(obj.BirdsEyePlot,'FaceColor','b');
%                 plotCoverageArea(plotter,cameras{i}.SensorLocation,cameras{i}.MaxRange,cameras{i}.Yaw,cameras{i}.FieldOfView(1));
%             end

            obj.OutlinePlotter = outlinePlotter(obj.BirdsEyePlot);
            obj.LaneMarkingPlotter = laneMarkingPlotter(obj.BirdsEyePlot);
            obj.LaneBoundaryPlotter = laneBoundaryPlotter(obj.BirdsEyePlot);

            % Create predicted trajectory plotter
            hold(obj.Parent,'on');
            obj.PredictedTrajectoryPlotter = plot(obj.Parent,nan,nan,'LineWidth',2,'Color',[0 0 0],'LineStyle','-.');

            % Create Ego trajectory plotters
            egoTrajectoryPlotters = cell(4,1);
            trajColors = [0 1 1;0.5 0.5 0.5;1 0 0;0 1 0];
            for i = 1:4
                egoTrajectoryPlotters{i} = plot(obj.Parent,nan,nan,'LineWidth',1,'Color',trajColors(i,:));
            end
            obj.EgoPredictedTrajectoryPlotters = egoTrajectoryPlotters;
            hold(obj.Parent,'off');
            obj.EgoPredictedTrajectoryPlotters = egoTrajectoryPlotters;
        end

        function stepImpl(obj, scenario, egoVehicle, sensors, detections, tracks, capList, trajectoryList)
            rb = roadBoundaries(egoVehicle);
            [position, yaw, length, width, originOffset, color] = targetOutlines(egoVehicle);
            % Get lane marking vertices and faces
            [lmv, lmf] = laneMarkingVertices(egoVehicle);
            % update the bird's-eye plotters with the road and actors
            plotLaneBoundary(obj.LaneBoundaryPlotter, rb);
            plotLaneMarking(obj.LaneMarkingPlotter, lmv, lmf);
            plotOutline(obj.OutlinePlotter, position, yaw, length, width, ...
                'OriginOffset', originOffset, 'Color', color);
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

            [~,poses] = obstaclePose(capList);
            posesXY = zeros(0,2);
            for i = 1:numel(poses)
                posesXY = [posesXY;poses(i).States(:,1:2);nan(1,2)]; %#ok<AGROW> 
            end
            R = rotmat(quaternion([egoVehicle.Yaw egoVehicle.Pitch egoVehicle.Roll],'eulerd','ZYX','frame'),'point');
            R = R(1:2,1:2);
            posesXYEgo = (R'*((posesXY - egoVehicle.Position(1:2))'))';
            obj.PredictedTrajectoryPlotter.XData = posesXYEgo(:,1);
            obj.PredictedTrajectoryPlotter.YData = posesXYEgo(:,2);

            [trajPos, trajClass] = HelperTrackingAndPlanningChasePlot.parseTrajectoryForPlotting(trajectoryList);
            trajPosEgo = (R'*((trajPos - egoVehicle.Position(1:2))'))';
    
            for i = 1:numel(obj.EgoPredictedTrajectoryPlotters)
                thisIdx = trajClass == i;
                thisX = trajPosEgo(thisIdx,1);
                thisY = trajPosEgo(thisIdx,2);
                obj.EgoPredictedTrajectoryPlotters{i}.XData = thisX;
                obj.EgoPredictedTrajectoryPlotters{i}.YData = thisY;
            end
        end
    end
end

function [pos, vel, labels] = TrackParsingFcn(tracks, egoVehicle)
if isempty(tracks)
    pos = zeros(0,2);
    vel = zeros(0,2);
    labels = string.empty(0,1);
    return
end
state = horzcat(tracks.State);
cartState = filterToCartState(state);
trackPos = cartState([1 3],:);
trackVel = cartState([2 4],:);
egoPos = egoVehicle.Position(1:2);
yaw = egoVehicle.Yaw;
R = [cosd(yaw) -sind(yaw);sind(yaw) cosd(yaw)];
pos = (R'*(trackPos - egoPos(:)))';
vel = (R'*(trackVel))';
labels = string([tracks.TrackID]);
end

function pos =  DetectionParsingFcn(detections, egoVehicle)
sIdx = cellfun(@(x)x.SensorIndex, detections);
uqIdx = unique(sIdx);
pos = zeros(3,numel(detections));
for i = 1:numel(uqIdx)
    thisIdx = sIdx == uqIdx(i);
    pos(:,thisIdx) = calculatePositionInEgoFrame(detections(thisIdx));
end
pos = pos(1:2,:)';
end


function [posEgo, velEgo, posCovEgo] = calculatePositionInEgoFrame(detections)

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

if nargout > 2
    assert(~strcmpi(allDets(1).MeasurementParameters(1).Frame,'Spherical'),'Only cartesian measurements');
    measCov = cat(3,allDets.MeasurementNoise);
    posCovEgo = zeros(3,3,numel(allDets));
    for i = 1:numel(allDets)
        posCovEgo(:,:,i) = R*measCov(:,:,i)*R';
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