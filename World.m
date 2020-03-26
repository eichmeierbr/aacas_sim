classdef World < handle
    properties
        vehicles = Vehicle.empty();
        obstacles = Obstacle.empty();
        n_obstacles = 0;
        n_vehicles = 0;
        dt = 0.1;
        t = 0;
        axis_lims = 30;
    end
    methods
        function [] = addObject(obj, new_obj)
            if isa(new_obj,'Obstacle')
                shp = size(obj.obstacles);
%                 obj.obstacles{shp(1),shp(2)+1} = new_obj;
                obj.obstacles = [obj.obstacles, new_obj];
%                 obj.obstacles(shp(2)+1) = new_obj;
                obj.n_obstacles = obj.n_obstacles + 1;
            elseif isa(new_obj,'Vehicle')
                shp = size(obj.vehicles);
%                 obj.vehicles{shp(1),shp(2)+1} = new_obj;
                obj.vehicles= [obj.vehicles, new_obj];
%                 obj.vehicles(shp(2)+1) = new_obj;
                obj.n_vehicles = obj.n_vehicles + 1; 
            end
        end
        
        function [] = tick(obj)
            clf
            hold on
            for i=1:obj.n_obstacles
                obj.obstacles(i).tick(obj)
            end
            for i=1:obj.n_vehicles
                obj.vehicles(i).tick(obj)
            end
            pos = obj.vehicles(1).state(1:2);
            hold off
            
            xlim([pos(1)-obj.axis_lims, pos(1)+obj.axis_lims]);
            ylim([pos(2)-obj.axis_lims, pos(2)+obj.axis_lims]);
            pbaspect([1 1 1])
            daspect([1 1 1])
            pause(obj.dt)
            obj.t = obj.t + obj.dt;
        end
        
    end 
end