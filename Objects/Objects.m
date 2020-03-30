classdef Objects < handle
   properties
      state = zeros(12,1); % pos, vel, orient, rot
      state_hist = [];
      rad % Size of the object
      numSides 
      color
      poly
%       orien     
%       rot
%       maxVel

%       controller = dontMove;
      controller = MotionController;
   end
   
   methods
       function obj = Objects(pos, vel, rad)
           obj.color = 'green';
          if nargin > 0
              obj.state(1:3) = pos;
          end
          if nargin >1
              obj.state(4:6) = vel;
          end
          if nargin > 2
              obj.state(7:9) = rad;
          end
          obj.state_hist = [obj.state, obj.state];
%           obj.poly = nsidedpoly(obj.numSides,'center',obj.state(1:2)','radius',obj.rad);
       end
       
       function p = getPos(obj,dev,bias)
           p = obj.state(1:3);
           if(nargin >1)
               for i = 1:length(p)
                   p(i) =p(i)+ bias + dev*randn;
               end
           end
           
       end
       
      function v = getVel(obj,dev,bias)
           v = obj.state(4:6);
           if(nargin >1)
              for i = 1:length(v)
                   v(i) =v(i)+ bias + dev*randn;
              end
           end
      end
       
      function [] = draw(obj)
          obj.poly = nsidedpoly(obj.numSides,'center',obj.state(1:2)','radius',obj.rad);
          plot(obj.poly, 'FaceColor',obj.color,'FaceAlpha',.8);
          plot(obj.state_hist(1,:),obj.state_hist(2,:),'Color', obj.color)
      end
      
      function [] = tick(obj,world)
          obj.controller.dt = world.dt;
          obj.controller.t = world.t;
          obj.state = obj.controller.move(obj.state);
          obj.state_hist = [obj.state_hist, obj.state];
          obj.draw;
      end
   end
end