addpath Dependencies/intersections
addpath MotionControllers
addpath Objects
addpath Sensors


a = Vehicle;
% a.sensor = RangeSensor;
a.sensor = CamLidarFusion;
a.controller = waypointG2Gstraight;

b = Obstacle([20 5 0]);
b.rad = 2;
b.controller = waypointG2G;
b.controller.waypoints = [0 5 0; 20 5 0;20 10 0];


c = Obstacle([-20 5 0]);
c.rad = 1.5;
c.controller = waypointG2G;
c.controller.waypoints = [-20 0 0;-20 10 0];

world = World;
world.addObject(a)
world.addObject(b)
world.addObject(c)

for i=1:300
    world.tick()
end