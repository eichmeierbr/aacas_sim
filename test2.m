addpath Dependencies/intersections
addpath MotionControllers
addpath VectorField
addpath Objects
addpath Sensors


a = Vehicle;
% a.sensor = RangeSensor;
% a.sensor = CamLidarFusion;
a.sensor = PerfectSensor;
a.controller = velocityFieldController;


% b = Obstacle([20 5 0]);
% b.rad = 1.254;
% b.controller = waypointG2G;
% b.controller.waypoints = [0 5 0; 20 5 0;20 10 0];

b = Obstacle([10 10 0]);
b.rad = 1.254;
b.controller = waypointG2G;
b.controller.waypoints = [10 10 0; -10 10 0];


c = Obstacle([-20 5 0]);
c.rad = 0.254;
c.controller = waypointG2G;
c.controller.waypoints = [-20 0 0;-20 10 0];

a.controller.safe_dist = 8;

world = World;
world.addObject(a)
world.addObject(b)
% world.addObject(c)

for i=1:300
    world.tick()
end