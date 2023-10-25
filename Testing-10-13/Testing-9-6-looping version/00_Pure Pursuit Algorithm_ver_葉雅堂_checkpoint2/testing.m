x = [0.012;
    4;
    6;
    8;
    ]
y = [5;1;5;3];


z = [0;0;0;0]
waypoints = [x,y,z];

wapoints_real = [waypoints(1:end) ;  waypoints(2:end)]'

refPath = referencePathFrenet(wapoints_real)


plot(waypoints)


