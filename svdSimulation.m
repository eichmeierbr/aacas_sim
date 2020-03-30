addpath Dependencies/intersections
addpath MotionControllers
addpath VectorField
addpath Objects
addpath Sensors

% Case 1
b_position = [-100,10,0];
% simulateSVD(b_position);

% Case 2
b_position = [-6,10,0];
% simulateSVD(b_position);

% Case 3
b_position = [-3,10,0];
simulateSVD(b_position);

% Case 4
b_position = [0,10,0];
simulateSVD(b_position);

% Case 5
b_position = [3,10,0];
simulateSVD(b_position);

function simulateSVD(position)
    a = Vehicle;
    a.sensor = PerfectSensor;
    a.controller = velocityFieldController;
    

    b = Obstacle(position);
    b.rad = 0.254;
%     b.rad = a.rad;
    b.controller = waypointG2G;
    b.controller.waypoints = b.state(1:3)';

    a.controller.safe_dist = 5 + a.rad + b.rad;
    
    world = World;
    world.addObject(a)
    world.addObject(b)

    for i=1:130
        world.tick()
    end
%     close all
end



