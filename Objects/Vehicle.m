classdef Vehicle < Objects
   properties
       sensor
       
       
       xo_latest
       yo_latest
       dist_latest
   end
    
   methods
       function obj = Vehicle(pos, vel, rad)
           % Set Shape and color properties
           obj.color = 'blue';
           obj.numSides = 6;
           obj.rad = 1.68;
           
           % Set Sensor
           obj.sensor = VisionSensor;
           
           % Set Controller
           Q = eye(12);
           Q(7,7) = 10;
           R = eye(6);
           obj.K = lqr(obj.A,obj.B,Q,R);
           
           % Read in arguments
          if nargin > 0
              obj.dims = length(pos);
              obj.pos = pos;
          end
          if nargin >1
              obj.vel = vel;
          end
          if nargin > 2
              obj.rad = rad;
          end
       end
       
       
       function [] = tick(obj,world)
           tick@Objects(obj,world);
           q = obj.state(1:2);
           th = obj.state(7);
           [xo, yo, dist_o] = obj.sensor.getObstacleDetections(q,th,world);
%            if(~isempty(xo))
%            obj.sensor.plotMeasurements(q, xo, yo);
%            end
       end
       
       function [] = draw(obj)
           draw@Objects(obj);
           xdata = [obj.state(1) obj.state(1)+obj.rad*cos(obj.state(7))];
           ydata = [obj.state(2) obj.state(2)+obj.rad*sin(obj.state(7))];
           plot(xdata,ydata,'r','LineWidth',3);
       end
       
   end
end