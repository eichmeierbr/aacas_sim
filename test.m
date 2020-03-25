

a = Vehicle;
a.controller = waypointG2G;
% a.controller.vel = [1,2,0];
for i=1:300
    clf
    hold on
    a.tick()
    hold off
    pbaspect([1 1 1])
    daspect([1 1 1])
    pause(.1)
end