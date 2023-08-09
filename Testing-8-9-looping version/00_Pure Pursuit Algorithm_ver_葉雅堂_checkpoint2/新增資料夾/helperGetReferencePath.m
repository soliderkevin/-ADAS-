function refPathOut = helperGetReferencePath()
persistent refPath
if isempty(refPath)
    waypoints = [0 50; 150 50; 300 75; 310 75; 400 0; 300 -50; 290 -50; 0 -50]; % in meters
    refPath = referencePathFrenet(waypoints);
end
refPathOut = refPath;
end