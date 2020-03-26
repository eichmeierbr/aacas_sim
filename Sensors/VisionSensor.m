classdef VisionSensor < handle
    %RangeSensor Creates a sensor based on a limited range
    
    properties (SetAccess = protected, GetAccess = public)
        n_lines = 30; % Number of range measurements
        max_dist = 30; % Max distance of the range measurements
        pos_bias = 0;
        pos_dev = 0;
        fov_h = 69.4*pi/180/2;
    end
    
    properties (SetAccess = protected, GetAccess = protected)
        orien_nom % Stores the nominal orientation of each of the range lines
        our_pos = 0;
        our_th = 0;
        
        % Obstacle plots
        h_obst_pos = []; % Handles to the obstacle posiiton plots
        h_obs_lines = [];% Handles to the ranging lines
    end
    
    methods (Access = public)
        function obj = VisionSensor()
            % Create the nominal orientations of the range lines
            if obj.n_lines <= 1 % Single line sensor directly out front
                obj.orien_nom = 0;
            else
                obj.orien_nom = linspace(0,2*pi,obj.n_lines+1);
                obj.orien_nom = obj.orien_nom(1:end-1); % Remove the duplicate at 2pi
            end
        end
        
        function [xo, yo, dist_o] = getObstacleDetections(obj,q, th, world)
            %getDistanceToObstacles Summary of this method goes here
            %   q: position of the robot
            %   th: orientation of the robot
            %   world: instantiation of the PolygonWorld class
            %
            % Return values:
            %   xo: x indices of the obstacles
            %   yo: y indices of the obstacles
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            xo = [];
            yo = [];
            dist_o = [];
            obj.our_pos = q;
            obj.our_th = th;
            for i=1:world.n_obstacles
                pos = world.obstacles(i).getPos(obj.pos_dev, obj.pos_bias);
                pos = pos(1:2);
                dist = norm(q-pos);
                
                angle = atan2(pos(2)-q(2),pos(1) - q(1));
                if and(dist < obj.max_dist,abs(angle-th) < obj.fov_h)
                    xo = [xo pos(1)];
                    yo = [yo pos(2)];
                    dist_o = [dist_o dist];
                end
            end
            
            obj.plotMeasurements(q, xo,yo);
        end
        
        function initializePlots(obj, ax)
            % create the obstacle position plots
            obj.h_obst_pos = plot(ax, 0, 0, 'ro', 'linewidth', 2); hold on;
            set(obj.h_obst_pos, 'xdata', [], 'ydata', []);
            
            % Create the lines plot
            obj.h_obs_lines = cell(obj.n_lines, 1);
            for k = 1:obj.n_lines
                % Create the plot
                obj.h_obs_lines{k} = plot(ax, 0, 0, ':r', 'linewidth', 1);
            end
        end
        
        function plotFOV(obj, q)
            % Plot FOV
            leftYs = [q(2), q(2)+obj.max_dist*sin(obj.our_th + obj.fov_h)];
            leftXs = [q(1), q(1)+obj.max_dist*cos(obj.our_th + obj.fov_h)];
                           
            rightYs = [q(2), q(2)+obj.max_dist*sin(obj.our_th - obj.fov_h)];
            rightXs = [q(1), q(1)+obj.max_dist*cos(obj.our_th - obj.fov_h)];
            
            plot(leftXs,leftYs,'-g');
            plot(rightXs,rightYs,'-g');
            
        end


        function plotMeasurements(obj, q, xo, yo)   
            obj.plotFOV(q);
            % Loop through and update the lines 
            for k = 1:length(xo)
                xdata = [q(1) xo(k)];
                ydata = [q(2) yo(k)];
%                 set(obj.h_obs_lines{k}, 'xdata', xdata, 'ydata', ydata);
                plot(xdata,ydata, '--g')
            end
            
        end
    end
    
end

