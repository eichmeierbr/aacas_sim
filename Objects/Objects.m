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
      controller = dontMove;
      A = [zeros(3), eye(3), zeros(3,6); zeros(3,12); zeros(3,9) eye(3); zeros(3,12)];
      B = [zeros(3,6); eye(3), zeros(3); zeros(3,6); zeros(3), eye(3)];
      K = [eye(3) 1.7321*eye(3) zeros(3,6); zeros(3,6) eye(3) 1.7321*eye(3)];
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
              for i = 1:length(p)
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
          dt = world.dt;
           
          xdes = obj.controller.move(obj);
          xdot = (obj.A - obj.B*obj.K)*(obj.state - xdes);
          obj.state = obj.state + xdot * dt;
          obj.state_hist = [obj.state_hist, obj.state];
          obj.draw;
      end
   end
end