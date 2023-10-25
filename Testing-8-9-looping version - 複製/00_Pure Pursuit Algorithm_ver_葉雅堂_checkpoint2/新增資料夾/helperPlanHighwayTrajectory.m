function [optimalTrajectory, trajectoryList] = helperPlanHighwayTrajectory(capList, curActorState, egoState)

% Define terminal state parameters
refPath = helperGetReferencePath;
laneWidth = 3.6;
speedLimit = 11;
timeHorizons = 1:5;
safetyGap = 10;
timResolution = 0.1;
collisionCheckResolution = 5;

% Define cost parameters.
latDevWeight    =  1;
timeWeight      = -1;
speedWeight     =  1;

% Define kinematic parameters
maxAcceleration =  15; % in meters/second^2
maxCurvature    =   1; % 1/meters, or radians/meter
minVelocity     =   0; % in meters/second

% Define connector as persistent variable
persistent connector
if isempty(connector)
    connector = trajectoryGeneratorFrenet(refPath,'TimeResolution',timResolution);
end

% Generate cruise control states.
[termStatesCC,timesCC] = exampleHelperBasicCruiseControl(...
    refPath,laneWidth,egoState,speedLimit,timeHorizons);

% Generate lane change states.
[termStatesLC,timesLC] = exampleHelperBasicLaneChange(...
    refPath,laneWidth,egoState,timeHorizons);

% Generate vehicle following states.
if ~isempty(curActorState)
    [termStatesF,timesF] = exampleHelperBasicLeadVehicleFollow(...
        refPath,laneWidth,safetyGap,egoState,curActorState,timeHorizons);
else
    termStatesF = zeros(0,6);
    timesF = zeros(0,1);
end

% Combine the terminal states and times.
allTS = [termStatesCC; termStatesLC; termStatesF];
allDT = [timesCC; timesLC; timesF];

costTS = exampleHelperEvaluateTSCost(allTS,allDT,laneWidth,speedLimit,...
    speedWeight, latDevWeight, timeWeight);

egoFrenetState = global2frenet(refPath,egoState);
[~,globalTraj] = connect(connector,egoFrenetState,allTS,allDT);

% Eliminate trajectories that violate constraints.
isValid = exampleHelperEvaluateTrajectory(globalTraj,maxAcceleration,maxCurvature,minVelocity);

% Determine evaluation order.
[~, idx] = sort(costTS);
optimalTrajectory = [];

trajectoryEvaluation = nan(numel(isValid),1);

% Check each trajectory for collisions starting with least cost.
for i = 1:numel(idx)
    if isValid(idx(i))
        % Update capsule list with the ego object's candidate trajectory.
        egoPoses.States = globalTraj(idx(i)).Trajectory(1:collisionCheckResolution:end,1:3);
        updateEgoPose(capList,1,egoPoses);

        % Check for collisions.
        isColliding = checkCollision(capList);

        if all(~isColliding)
            % If no collisions are found, this is the optimal.
            % trajectory.
            trajectoryEvaluation(idx(i)) = 1;
            optimalTrajectory = globalTraj(idx(i)).Trajectory;
            break;
        else
            trajectoryEvaluation(idx(i)) = 0;
        end
    end
end

trajectoryList = globalTraj;
for i = 1:numel(trajectoryList)
    trajectoryList(i).Evaluation = trajectoryEvaluation(i);
    trajectoryList(i).IsValid = isValid(i);
end

end

function laneNum = exampleHelperPredictLane(frenetState, laneWidth, dt)
%exampleHelperPredictLane Predicts a vehicles lane for a given set of times
%
%   This function is for internal use only. It may be removed in the future

%   LANENUM = exampleHelperPredictLane(FRENETSTATE, LANEWIDTH, DT) predicts
%   a vehicle's lane over one or more times, DT, in the future based on its
%   current FRENETSTATE, a 1-by-6 vector in Frenet coordinates [S dS ddS L dL ddL],
%   and the following assumptions:
%
%       1) Bang bang acceleration control
%       2) Constant change in lateral acceleration for two control sections
%   	3) Terminal lateral velocity = 0
%       4) Terminal lateral acceleration = 0
%       5) Scenario occurs on a 4-lane highway with fixed lane width, LANEWIDTH
%
%           NOTE: When DT is zero, no motion model assumptions are applied.
%
% Copyright 2020 The MathWorks, Inc.

narginchk(3,3);

laneBounds = [inf (2:-1:-2)*laneWidth -inf];
laneNum = zeros(numel(dt),1);

for i = 1:numel(dt)
    if dt(i) == 0
        dLaneEgo = laneBounds-frenetState(4);
        laneNum(i) = min(find(dLaneEgo(2:(end-1)) >= 0 & dLaneEgo(3:(end)) < 0,1),4);
    else
        % Retrieve current velocity/acceleration/time
        t  = dt(i);
        a0 = frenetState(6);
        v0 = frenetState(5);

        % Solve for the constant change in acceleration and time of
        % application that arrest the ego vehicle's lateral
        % velocity and acceleration over a given number of seconds.
        if abs(a0) < 1e-15
            avgAcc = -v0/t;
            Ldiff = v0*t + avgAcc/2*t^2;
        else
            a = a0;
            b = (-2*v0-2*a0*t);
            c = (v0*t+a0/2*t^2);

            % Possible time switches
            r = (-b+(sqrt(b^2-4*a*c).*[-1 1]))/(2*a);

            % Select the option that occurs in the future
            rS = r(r>0 & r <= t);

            % Calculate the constant change in acceleration
            da0 = a0/(t-2*rS);

            % Predict total distance traveled over t seconds
            Ldiff = v0*t + a0/2*t^2 + da0/6*t^3 - da0/6*(t-rS)^3;
        end
        % Find distance between predicted offset and each lane
        dLaneEgo = laneBounds-(frenetState(4)+Ldiff);
        
        % Determine future lane
        idx = min(find(dLaneEgo(2:(end-1)) >= 0 & dLaneEgo(3:(end)) < 0,1),4);
        if ~isempty(idx)
            laneNum(i) = idx;
        else
            laneNum(i) = 4;
        end
    end
end
end

function costs = exampleHelperEvaluateTSCost(terminalStates, times, laneWidth, speedLimit, speedWeight, latWeight, timeWeight)
%exampleHelperEvaluateTSCost Evaluate trajectory cost.
%
%   This function is for internal use only. It may be removed in the future

%   COSTS = exampleHelperEvaluateTSCost(TERMINALSTATES, times, laneWidth, speedLimit, speedWeight, latWeight, timeWeight)
%   Evaluates the cost of an N-by-6 matrix of Frenet states, TERMINALSTATES,
%   and corresponding N-by-1 vector of timespans, TIMES.
%
%   Cost is based on lateral deviation from a lane center, calculated using
%   LANEWIDTH, deviation from the desired velocity, SPEEDLIMIT, and the span
%   of time required by the trajectory, TIMES.
%
%   Each of these metrics is scaled by the accompanying weights,
%   LATWEIGHT, SPEEDWEIGHT, and TIMEWEIGHT, respectively.
%
% Copyright 2020 The MathWorks, Inc.

% Find lateral deviation from nearest lane center
laneCenters = (1.5:-1:-1.5)*laneWidth;
latDeviation = abs(laneCenters-terminalStates(:,4));

% Calculate lateral deviation cost
latCost = min(latDeviation,[],2)*latWeight;

% Calculate trajectory time cost
timeCost = times*timeWeight;

% Calculate terminal speed vs desired speed cost
speedCost = abs(terminalStates(:,2)-speedLimit)*speedWeight;

% Return cumulative cost
costs = latCost+timeCost+speedCost;
end

function isValid = exampleHelperEvaluateTrajectory(globalTrajectory, maxAcceleration, maxCurvature, minVelocity)
%exampleHelperEvaluateTrajectory Evaluate trajectory constraints.
%
%   This function is for internal use only. It may be removed in the future

%   ISVALID = exampleHelperEvaluateTrajectory(GLOBALTRAJECTORY, MAXACCELERATION, MAXCURVATURE, MINVELOCITY)
%   Takes in a N-element struct array of trajectories, GLOBALTRAJECTORY, and
%   checks whether each trajectory violates the MAXACCELERATION, MAXCURVATURE,
%   or MINVELOCITY constraints.
%
%   Each element of GLOBALTRAJECTORY contains the fields Trajectory, an
%   N-by-[x y theta kappa v a] matrix, and Times, an N-by-1 vector of timesteps.
%
%   Returns an N-by-1 vector of logicals, ISVALID, where an entry of false
%   means that the corresponding trajectory violated one or more constraints,
%   and true means that all constraints were met.
%
% Copyright 2020 The MathWorks, Inc.

isValid = true(numel(globalTrajectory),1);
for i = 1:numel(globalTrajectory)
    % Acceleration constraint
    accViolated  = any(abs(globalTrajectory(i).Trajectory(:,6)) > abs(maxAcceleration));

    % Curvature constraint
    curvViolated = any(abs(globalTrajectory(i).Trajectory(:,4)) > abs(maxCurvature));

    % Velocity constraint
    velViolated  = any(globalTrajectory(i).Trajectory(:,5) < minVelocity);

    isValid(i) = ~accViolated && ~curvViolated && ~velViolated;
end
end

function [terminalStates, times] = exampleHelperBasicLeadVehicleFollow(refPath, laneWidth, safetyGap, egoState, actorState, dt)
%exampleHelperBasicLeadVehicleFollow Generates terminal states for vehicle following behavior.
%
%   This function is for internal use only. It may be removed in the future

%   [TERMINALSTATES, TIMES] = exampleHelperBasicLeadVehicleFollow(REFPATH, LANEWIDTH, SAFETYGAP, EGOSTATE, ACTORSTATE, DT)
%   Generates terminal states that attempt to follow behind the nearest
%   vehicle in the ego vehicle's current lane over given spans of time, DT.
%
%   REFPATH is a referencePathFrenet object used to convert the ego and
%   actors' state, EGOSTATE/ACTORSTATE, respectively, from global coordinates,
%   [x y theta kappa v a], to Frenet coordinates, [S dS ddS L dL ddL].
%
%   Once in Frenet coordinates, exampleHelperPredictLane is used to predict
%   the future lanes of all actors over the given times, DT. The nearest
%   actor leading the ego vehicle in the same lane is chosen as the vehicle
%   to follow.
%
%   The function returns TERMINALSTATES, an N-by-6 matrix where each row
%   is a Frenet state defined as follows:
%   [(S_lead-SAFETYGAP) dS_lead 0 L_lead dL_lead 0], where *_lead denotes
%   the state of the nearest lead vehicle.
%
% Copyright 2020 The MathWorks, Inc.

% Convert ego state to Frenet coordinates
frenetStateEgo = global2frenet(refPath, egoState);

% Get current lane of ego vehicle
curEgoLane = exampleHelperPredictLane(frenetStateEgo, laneWidth, 0);

% Get current and predicted lanes for each actor
frenetStateActors = global2frenet(refPath, actorState);

predictedActorLanes = zeros(numel(dt),size(actorState,1));
for i = 1:size(actorState,1)
    predictedActorLanes(:,i) = exampleHelperPredictLane(frenetStateActors(i,:),laneWidth,dt);
end
% For each time horizon, find the closest car in the same lane as
% ego vehicle
terminalStates = zeros(numel(dt),6);
validTS = false(numel(dt),1);
for i = 1:numel(dt)
    % Find vehicles in same lane t seconds into the future
    laneMatch = curEgoLane == predictedActorLanes(i,:)';

    % Determine if they are ahead of the ego vehicle
    leadVehicle = frenetStateEgo(1) < frenetStateActors(:,1);

    % Of these, find the vehicle closest to the ego vehicle (assume
    % constant longitudinal velocity)
    future_S = frenetStateActors(:,1) + frenetStateActors(:,2)*dt(i);
    future_S(~leadVehicle | ~laneMatch) = inf;
    [actor_S1, idx] = min(future_S);

    % Check if any car meets the conditions
    if actor_S1 ~= inf
        % If distance is greater than safety gap, set the terminal
        % state behind this lead vehicle
        if frenetStateEgo(1)+safetyGap < actor_S1
            ego_S1 = actor_S1-safetyGap;
            terminalStates(i,:) = [ego_S1 frenetStateActors(idx,2) 0 frenetStateActors(idx,4:5) 0];
            validTS(i) = true;
        end
    end
end
% Remove any bad terminal states
terminalStates(~validTS,:) = [];
times = dt(validTS(:));
times = times(:);
end

function [terminalStates, times] = exampleHelperBasicCruiseControl(refPath, laneWidth, egoState, targetVelocity, dt)
%exampleHelperBasicCruiseControl Generates terminal states for cruise control behavior.
%
%   This function is for internal use only. It may be removed in the future

%   [TERMINALSTATES, TIMES] = exampleHelperBasicCruiseControl(REFPATH, LANEWIDTH, EGOSTATE, TARGETVELOCITY, DT)
%   Generates terminal states that attempt to obey the speed limit,
%   TARGETVELOCITY, while following a lane center over a given spans of time, DT.
%
%   REFPATH is a referencePathFrenet object used to convert the ego
%   vehicle's state, EGOSTATE, from global coordinates, [x y theta kappa v a],
%   to Frenet coordinates, [S dS ddS L dL ddL].
%
%   Once in Frenet coordinates, exampleHelperPredictLane is used to predict
%   the future lane that the vehicle would reside in based on the current
%   Frenet state and zero terminal velocity/acceleration over the given
%   span of time.
%
%   The function returns TERMINALSTATES, an N-by-6 matrix where each row
%   is a Frenet state defined as follows: [NaN TARGETVELOCITY 0 L_lane 0 0], where
%   L_lane is the lateral deviation of the predicted lane-center. TIMES is
%   also returned as an N-by-1 vector of doubles.
%
% Copyright 2020 The MathWorks, Inc.

% Convert ego state to Frenet coordinates
frenetState = global2frenet(refPath, egoState);

% Determine current and future lanes
futureLane = exampleHelperPredictLane(frenetState, laneWidth, dt);

% Convert future lanes to lateral offsets
lateralOffsets = (2-futureLane+.5)*laneWidth;

% Return terminal states
terminalStates      = zeros(numel(dt),6);
terminalStates(:,1) = nan;
terminalStates(:,2) = targetVelocity;
terminalStates(:,4) = lateralOffsets;
times = dt(:);
end

function [terminalStates, times] = exampleHelperBasicLaneChange(refPath, laneWidth, egoState, dt)
%exampleHelperBasicLaneChange Generates terminal states for lane change behavior.
%
%   This function is for internal use only. It may be removed in the future

%   [TERMINALSTATES, TIMES] = exampleHelperBasicLaneChange(REFPATH, LANEWIDTH, EGOSTATE, DT)
%   Generates terminal states, TERMINALSTATES, that transition the ego vehicle
%   from the current lane to adjacent lanes. REFPATH is a referencePathFrenet
%   object used to convert the ego vehicle's state, EGOSTATE, from global
%   coordinates, [x y theta kappa v a], to Frenet coordinates, [S dS ddS L dL ddL].
%
%   Once in Frenet coordinates, exampleHelperPredictLane returns the current
%   lane, and the centers of the adjacent lanes are then calculated relative to
%   REFPATH assuming a fixed LANEWIDTH and 4-lane road.
%
%   The function returns TERMINALSTATES, an N-by-6 matrix where each row
%   is a Frenet state defined as follows: [NaN dS_cur 0 L_lane 0 0], where
%   dS_cur is the ego vehicle's current longitudinal velocity and L_lane is
%   the lateral deviation of the lane-center. TIMES is also returned as an
%   N-by-1 vector of doubles.
%
% Copyright 2020 The MathWorks, Inc.

if egoState(5) == 0
    terminalStates = [];
    times = [];
else
    % Convert ego state to Frenet coordinates
    frenetState = global2frenet(refPath, egoState);

    % Get current lane
    curLane = exampleHelperPredictLane(frenetState, laneWidth, 0);

    % Determine if future lanes are available
    adjacentLanes = curLane+[-1 1];
    validLanes = adjacentLanes > 0 & adjacentLanes <= 4;

    % Calculate lateral deviation for adjacent lanes
    lateralOffset = (2-adjacentLanes(validLanes)+.5)*laneWidth;
    numLane = nnz(validLanes);

    % Calculate terminal states
    terminalStates = zeros(numLane*numel(dt),6);
    terminalStates(:,1) = nan;
    terminalStates(:,2) = egoState(5);
    terminalStates(:,4) = repelem(lateralOffset(:),numel(dt),1);
    times = repmat(dt(:),numLane,1);
end
end